#!/usr/bin/env python3
"""
THACO persistent companion runtime test (plain MAVLink / Pymavlink)

State machine
-------------
WAIT_TRIGGER / IDLE
    - Always stream ZERO BODY_NED velocity at 10 Hz.
    - Wait for THACO_EXTERNAL_XYZ_TRIGGER(N).

WAIT_OFFBOARD
    - Immediately request OFFBOARD when a new trigger is received.
    - Keep streaming ZERO BODY_NED velocity at 10 Hz.
    - Start the square only after PX4 HEARTBEAT confirms OFFBOARD.
    - Also require fresh LOCAL_POSITION_NED + ATTITUDE feedback.

FLY_SQUARE
    - Fly a 10 m x 10 m square aligned with vehicle yaw at trigger time.
    - The square is converted from initial body axes to LOCAL_NED targets:
          P1: +10 m body X
          P2: +10 m body X, +10 m body Y
          P3:          0 m body X, +10 m body Y
          P4: start point
    - Send LOCAL_NED position target at 10 Hz.
    - Hold the initial yaw.
    - Advance to the next corner when position error <= tolerance.

ACTUATOR
    - Stream ZERO BODY_NED velocity at 10 Hz.
    - Send MAV_CMD_THACO_ACTUATOR_CONTROL (44001) at 10 Hz for 1 second.
    - Default: actuator index 1 (A2), value 200.

WAIT_FINAL
    - Stop 44001.
    - Continue ZERO BODY_NED velocity at 10 Hz.
    - Send MAV_CMD_THACO_EXTERNAL_XYZ_COMPLETE (44002) with param1=N.
    - IN_PROGRESS: keep waiting and keep zero setpoint alive.
    - ACCEPTED: clear transaction and return to WAIT_TRIGGER.
    - Retry the SAME COMPLETE(N) on timeout.

Important
---------
- Persistent process: does NOT exit after one successful transaction.
- Flight setpoint streaming never stops while process is running.
- Outside FLY_SQUARE, flight setpoint is always ZERO BODY_NED velocity.
- Custom dialect is required to decode THACO_EXTERNAL_XYZ_TRIGGER (msgid 32000).
"""

import argparse
import math
import os
import sys
import threading
import time

PX4_REPO = os.environ.get(
    "PX4_REPO",
    os.path.expanduser("~/agridrone-fc-firmware"),
)
DIALECT = os.environ.get("THACO_MAVLINK_DIALECT", "thaco_common")

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = DIALECT
os.environ["MDEF"] = os.path.join(
    PX4_REPO,
    "src/modules/mavlink/mavlink/message_definitions",
)

from pymavlink import mavutil

CMD_DO_SET_MODE = 176
CMD_SET_MESSAGE_INTERVAL = 511
CMD_THACO_ACTUATOR = 44001
CMD_THACO_COMPLETE = 44002

MSG_ID_ATTITUDE = 30
MSG_ID_LOCAL_POSITION_NED = 32

MAV_RESULT_ACCEPTED = 0
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_RESULT_DENIED = 2
MAV_RESULT_FAILED = 4
MAV_RESULT_IN_PROGRESS = 5
MAV_RESULT_CANCELLED = 6

MAV_FRAME_LOCAL_NED = 1
MAV_FRAME_BODY_NED = 8

PX4_MAIN_OFFBOARD = 6

# SET_POSITION_TARGET_LOCAL_NED:
# active vx/vy/vz, ignore x/y/z, accel, yaw, yaw-rate
VELOCITY_ONLY_MASK = 3527

# active x/y/z + yaw, ignore velocity, accel and yaw-rate
POSITION_YAW_MASK = 2552

WAIT_TRIGGER = "WAIT_TRIGGER"
WAIT_OFFBOARD = "WAIT_OFFBOARD"
FLY_SQUARE = "FLY_SQUARE"
ACTUATOR = "ACTUATOR"
WAIT_FINAL = "WAIT_FINAL"


def now():
    return time.monotonic()


def boot_ms():
    return int(time.monotonic() * 1000.0) & 0xFFFFFFFF


def px4_main_mode(heartbeat):
    custom_mode = int(getattr(heartbeat, "custom_mode", 0))
    return (custom_mode >> 16) & 0xFF


def result_name(result):
    return {
        0: "ACCEPTED",
        1: "TEMPORARILY_REJECTED",
        2: "DENIED",
        3: "UNSUPPORTED",
        4: "FAILED",
        5: "IN_PROGRESS",
        6: "CANCELLED",
    }.get(int(result), f"RESULT_{result}")


class App:
    def __init__(self, args, master, target_system, target_component):
        self.args = args
        self.master = master
        self.target_system = target_system
        self.target_component = target_component

        self.state = WAIT_TRIGGER
        self.state_since = now()

        self.trigger_id = None
        self.is_offboard = False
        self.offboard_confirmed = False

        self.local_x = None
        self.local_y = None
        self.local_z = None
        self.local_position_time = None

        self.yaw = None
        self.attitude_time = None

        self.square_yaw = None
        self.square_targets = []
        self.corner_index = 0
        self.corner_inside_since = None

        self.last_complete = None

        self.stop = threading.Event()
        self.state_lock = threading.RLock()
        self.tx_lock = threading.Lock()

    def set_state(self, new_state):
        with self.state_lock:
            old_state = self.state
            if old_state == new_state:
                return
            self.state = new_state
            self.state_since = now()

        print(f"[STATE] {old_state} -> {new_state}", flush=True)

    def reset_transaction(self):
        with self.state_lock:
            old_n = self.trigger_id
            self.trigger_id = None
            self.offboard_confirmed = False
            self.square_yaw = None
            self.square_targets = []
            self.corner_index = 0
            self.corner_inside_since = None
            self.last_complete = None

        self.set_state(WAIT_TRIGGER)
        print(
            f"[READY] transaction N={old_n} cleared; waiting for next trigger",
            flush=True,
        )

    def send_zero_velocity(self):
        with self.tx_lock:
            self.master.mav.set_position_target_local_ned_send(
                boot_ms(),
                self.target_system,
                self.target_component,
                MAV_FRAME_BODY_NED,
                VELOCITY_ONLY_MASK,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0,
            )

    def send_square_target(self):
        with self.state_lock:
            if not self.square_targets:
                return
            if self.corner_index >= len(self.square_targets):
                return

            x, y, z = self.square_targets[self.corner_index]
            yaw = self.square_yaw

        with self.tx_lock:
            self.master.mav.set_position_target_local_ned_send(
                boot_ms(),
                self.target_system,
                self.target_component,
                MAV_FRAME_LOCAL_NED,
                POSITION_YAW_MASK,
                float(x), float(y), float(z),
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                float(yaw), 0.0,
            )

    def send_offboard_request(self):
        with self.tx_lock:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                CMD_DO_SET_MODE,
                0,
                1.0,
                6.0,
                0.0,
                0.0, 0.0, 0.0, 0.0,
            )

    def send_actuator(self):
        with self.tx_lock:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                CMD_THACO_ACTUATOR,
                0,
                float(self.args.actuator_index),
                float(self.args.actuator_value),
                0.0, 0.0, 0.0, 0.0, 0.0,
            )

    def send_complete(self):
        with self.state_lock:
            trigger_id = self.trigger_id

        if trigger_id is None:
            return

        with self.tx_lock:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                CMD_THACO_COMPLETE,
                0,
                float(trigger_id),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            )

    def send_heartbeat(self):
        with self.tx_lock:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )

    def request_message_interval(self, message_id, rate_hz):
        interval_us = int(1_000_000.0 / rate_hz)

        with self.tx_lock:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                CMD_SET_MESSAGE_INTERVAL,
                0,
                float(message_id),
                float(interval_us),
                0.0, 0.0, 0.0, 0.0, 0.0,
            )

    def tx_loop(self):
        flight_period = 1.0 / self.args.rate
        actuator_period = 1.0 / self.args.actuator_rate

        next_flight = now()
        next_actuator = now()
        next_heartbeat = now()

        while not self.stop.is_set():
            t = now()

            if t >= next_heartbeat:
                self.send_heartbeat()
                next_heartbeat = t + 1.0

            if t >= next_flight:
                with self.state_lock:
                    state = self.state

                if state == FLY_SQUARE:
                    self.send_square_target()
                else:
                    self.send_zero_velocity()

                next_flight = t + flight_period

            with self.state_lock:
                state = self.state

            if state == ACTUATOR:
                if t >= next_actuator:
                    self.send_actuator()
                    next_actuator = t + actuator_period
            else:
                next_actuator = t

            self.stop.wait(0.002)

    def feedback_is_fresh(self):
        t = now()

        if self.local_position_time is None or self.attitude_time is None:
            return False

        return (
            t - self.local_position_time <= self.args.feedback_timeout
            and t - self.attitude_time <= self.args.feedback_timeout
        )

    def build_square(self):
        if not self.feedback_is_fresh():
            return False

        x0 = float(self.local_x)
        y0 = float(self.local_y)
        z0 = float(self.local_z)
        yaw0 = float(self.yaw)
        edge = float(self.args.edge)

        forward_x = math.cos(yaw0)
        forward_y = math.sin(yaw0)

        right_x = -math.sin(yaw0)
        right_y = math.cos(yaw0)

        p1 = (x0 + edge * forward_x, y0 + edge * forward_y, z0)
        p2 = (p1[0] + edge * right_x, p1[1] + edge * right_y, z0)
        p3 = (p2[0] - edge * forward_x, p2[1] - edge * forward_y, z0)
        p4 = (x0, y0, z0)

        with self.state_lock:
            self.square_yaw = yaw0
            self.square_targets = [p1, p2, p3, p4]
            self.corner_index = 0
            self.corner_inside_since = None

        print(
            f"[SQUARE] origin=({x0:.2f}, {y0:.2f}, {z0:.2f}) "
            f"yaw={math.degrees(yaw0):.1f} deg edge={edge:.1f} m",
            flush=True,
        )

        for i, p in enumerate(self.square_targets, start=1):
            print(
                f"[SQUARE] P{i}=({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})",
                flush=True,
            )

        return True

    def try_start_square(self):
        with self.state_lock:
            if self.state != WAIT_OFFBOARD:
                return
            if not self.offboard_confirmed:
                return

        if not self.build_square():
            return

        self.set_state(FLY_SQUARE)
        print(
            "[SQUARE] OFFBOARD confirmed and feedback ready; "
            "start P1 -> P2 -> P3 -> P4",
            flush=True,
        )

    def update_square_progress(self):
        with self.state_lock:
            if self.state != FLY_SQUARE:
                return
            if self.corner_index >= len(self.square_targets):
                return

            target = self.square_targets[self.corner_index]
            x = self.local_x
            y = self.local_y
            z = self.local_z

        if x is None or y is None or z is None:
            return

        dx = float(target[0]) - float(x)
        dy = float(target[1]) - float(y)
        dz = float(target[2]) - float(z)
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        t = now()

        if distance <= self.args.position_tolerance:
            if self.corner_inside_since is None:
                self.corner_inside_since = t

            if t - self.corner_inside_since >= self.args.corner_hold:
                with self.state_lock:
                    reached = self.corner_index + 1
                    self.corner_index += 1
                    self.corner_inside_since = None
                    finished = self.corner_index >= len(self.square_targets)

                print(
                    f"[SQUARE] reached P{reached}, error={distance:.2f} m",
                    flush=True,
                )

                if finished:
                    self.set_state(ACTUATOR)
                    print(
                        f"[ACTUATOR] square complete; zero flight setpoint, "
                        f"A{self.args.actuator_index + 1}="
                        f"{self.args.actuator_value} @ "
                        f"{self.args.actuator_rate:.1f} Hz for "
                        f"{self.args.actuator_duration:.1f} s",
                        flush=True,
                    )
        else:
            self.corner_inside_since = None

    def tick(self):
        t = now()

        with self.state_lock:
            state = self.state
            state_since = self.state_since
            last_complete = self.last_complete

        if state == WAIT_OFFBOARD:
            self.try_start_square()

            if t - state_since > self.args.offboard_timeout:
                print(
                    "[FAIL] OFFBOARD/feedback not ready before timeout; "
                    "return to WAIT_TRIGGER",
                    flush=True,
                )
                self.reset_transaction()

        elif state == FLY_SQUARE:
            self.update_square_progress()

        elif state == ACTUATOR:
            if t - state_since >= self.args.actuator_duration:
                self.set_state(WAIT_FINAL)

                self.send_zero_velocity()

                print(
                    f"[TX] COMPLETE({self.trigger_id})",
                    flush=True,
                )

                self.send_complete()

                with self.state_lock:
                    self.last_complete = now()

        elif state == WAIT_FINAL:
            if (
                last_complete is not None
                and t - last_complete >= self.args.complete_retry
            ):
                print(
                    f"[RETRY] SAME COMPLETE({self.trigger_id})",
                    flush=True,
                )

                self.send_complete()

                with self.state_lock:
                    self.last_complete = now()

    def handle_heartbeat(self, msg):
        if (
            msg.get_srcSystem() != self.target_system
            or msg.get_srcComponent() != self.target_component
        ):
            return

        new_is_offboard = (
            px4_main_mode(msg) == PX4_MAIN_OFFBOARD
        )

        if new_is_offboard != self.is_offboard:
            self.is_offboard = new_is_offboard
            print(
                f"[PX4] OFFBOARD={'YES' if new_is_offboard else 'NO'}",
                flush=True,
            )

        with self.state_lock:
            state = self.state

        if state == WAIT_OFFBOARD and new_is_offboard:
            self.offboard_confirmed = True
            print("[PASS] OFFBOARD confirmed", flush=True)
            self.try_start_square()

        elif state in (FLY_SQUARE, ACTUATOR) and not new_is_offboard:
            print(
                "[CANCEL] PX4 left OFFBOARD before external task completed",
                flush=True,
            )
            self.reset_transaction()

        # OFFBOARD -> MISSION during WAIT_FINAL is expected.

    def handle_trigger(self, msg):
        n = int(msg.trigger_id)

        with self.state_lock:
            current_n = self.trigger_id
            state = self.state

        if current_n is None and state == WAIT_TRIGGER:
            with self.state_lock:
                self.trigger_id = n
                self.offboard_confirmed = False

            print(f"[RX] TRIGGER({n})", flush=True)

            self.set_state(WAIT_OFFBOARD)

            print(
                "[TX] request OFFBOARD immediately",
                flush=True,
            )
            self.send_offboard_request()
            return

        if n == current_n:
            print(f"[RX] retry TRIGGER({n})", flush=True)
            return

        print(
            f"[WARN] TRIGGER({n}) ignored while transaction "
            f"N={current_n} state={state}",
            flush=True,
        )

    def handle_ack(self, msg):
        cmd = int(msg.command)
        res = int(msg.result)

        if cmd == CMD_DO_SET_MODE:
            print(
                f"[ACK] OFFBOARD request: {result_name(res)}({res})",
                flush=True,
            )
            return

        if cmd == CMD_THACO_ACTUATOR:
            if res != MAV_RESULT_ACCEPTED:
                print(
                    f"[ACK] 44001: {result_name(res)}({res})",
                    flush=True,
                )
            return

        if cmd != CMD_THACO_COMPLETE:
            return

        print(
            f"[ACK] 44002: {result_name(res)}({res})",
            flush=True,
        )

        with self.state_lock:
            if self.state != WAIT_FINAL:
                return

        if res == MAV_RESULT_IN_PROGRESS:
            print(
                "[WAIT_FINAL] IN_PROGRESS -> keep ZERO setpoint @10Hz",
                flush=True,
            )
            return

        if res == MAV_RESULT_ACCEPTED:
            print("[PASS] transaction complete", flush=True)
            self.reset_transaction()
            return

        if res == MAV_RESULT_TEMPORARILY_REJECTED:
            print(
                "[WAIT_FINAL] TEMPORARILY_REJECTED -> retry SAME N",
                flush=True,
            )
            return

        if res in (
            MAV_RESULT_DENIED,
            MAV_RESULT_FAILED,
            MAV_RESULT_CANCELLED,
        ):
            print(
                f"[FAIL] COMPLETE rejected: {result_name(res)}({res})",
                flush=True,
            )
            self.reset_transaction()

    def handle(self, msg):
        mtype = msg.get_type()

        if mtype == "BAD_DATA":
            return

        if mtype == "HEARTBEAT":
            self.handle_heartbeat(msg)
            return

        if mtype == "LOCAL_POSITION_NED":
            if msg.get_srcSystem() != self.target_system:
                return

            self.local_x = float(msg.x)
            self.local_y = float(msg.y)
            self.local_z = float(msg.z)
            self.local_position_time = now()

            if self.state == WAIT_OFFBOARD:
                self.try_start_square()
            return

        if mtype == "ATTITUDE":
            if msg.get_srcSystem() != self.target_system:
                return

            self.yaw = float(msg.yaw)
            self.attitude_time = now()

            if self.state == WAIT_OFFBOARD:
                self.try_start_square()
            return

        if mtype == "THACO_EXTERNAL_XYZ_TRIGGER":
            self.handle_trigger(msg)
            return

        if mtype == "COMMAND_ACK":
            self.handle_ack(msg)


def parse_args():
    p = argparse.ArgumentParser(
        description="Persistent THACO companion square-flight test"
    )

    p.add_argument("--listen-port", type=int, default=14601)
    p.add_argument("--source-system", type=int, default=245)
    p.add_argument("--source-component", type=int, default=191)

    p.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Persistent flight setpoint rate [Hz], default 10",
    )
    p.add_argument(
        "--edge",
        type=float,
        default=10.0,
        help="Square edge length [m], default 10",
    )
    p.add_argument(
        "--position-tolerance",
        type=float,
        default=0.5,
        help="Corner acceptance radius [m], default 0.5",
    )
    p.add_argument(
        "--corner-hold",
        type=float,
        default=0.3,
        help="Time inside corner tolerance before advancing [s]",
    )
    p.add_argument(
        "--feedback-rate",
        type=float,
        default=10.0,
        help="Requested ATTITUDE/LOCAL_POSITION_NED rate [Hz]",
    )
    p.add_argument(
        "--feedback-timeout",
        type=float,
        default=1.0,
        help="Maximum feedback age accepted when starting square [s]",
    )
    p.add_argument(
        "--offboard-timeout",
        type=float,
        default=5.0,
        help="Timeout waiting for OFFBOARD + navigation feedback [s]",
    )
    p.add_argument(
        "--actuator-index",
        type=int,
        default=1,
        help="0=A1 ... 7=A8; default 1=A2",
    )
    p.add_argument(
        "--actuator-value",
        type=int,
        default=200,
        help="Actuator value 0..255; default 200",
    )
    p.add_argument(
        "--actuator-rate",
        type=float,
        default=10.0,
        help="44001 rate during ACTUATOR [Hz], default 10",
    )
    p.add_argument(
        "--actuator-duration",
        type=float,
        default=1.0,
        help="44001 duration [s], default 1",
    )
    p.add_argument(
        "--complete-retry",
        type=float,
        default=3.0,
        help="Retry interval for SAME COMPLETE(N) [s]",
    )

    return p.parse_args()


def validate_args(args):
    if args.rate <= 2.0:
        raise SystemExit("--rate must be > 2 Hz; use 10 Hz")

    if args.edge <= 0:
        raise SystemExit("--edge must be > 0")

    if args.position_tolerance <= 0:
        raise SystemExit("--position-tolerance must be > 0")

    if args.corner_hold < 0:
        raise SystemExit("--corner-hold must be >= 0")

    if args.feedback_rate <= 0:
        raise SystemExit("--feedback-rate must be > 0")

    if args.feedback_timeout <= 0:
        raise SystemExit("--feedback-timeout must be > 0")

    if args.offboard_timeout <= 0:
        raise SystemExit("--offboard-timeout must be > 0")

    if not 0 <= args.actuator_index <= 7:
        raise SystemExit("--actuator-index must be 0..7")

    if not 0 <= args.actuator_value <= 255:
        raise SystemExit("--actuator-value must be 0..255")

    if args.actuator_rate <= 0:
        raise SystemExit("--actuator-rate must be > 0")

    if args.actuator_duration <= 0:
        raise SystemExit("--actuator-duration must be > 0")

    if args.complete_retry <= 0:
        raise SystemExit("--complete-retry must be > 0")


def verify_dialect():
    custom_id = getattr(
        mavutil.mavlink,
        "MAVLINK_MSG_ID_THACO_EXTERNAL_XYZ_TRIGGER",
        None,
    )

    if custom_id != 32000:
        raise SystemExit(
            "[FATAL] THACO dialect not loaded correctly. "
            f"Expected trigger msgid=32000, got {custom_id!r}\n"
            f"DIALECT={DIALECT}\n"
            f"MDEF={os.environ.get('MDEF')}"
        )


def main():
    args = parse_args()
    validate_args(args)
    verify_dialect()

    endpoint = f"udpin:0.0.0.0:{args.listen_port}"

    print(
        f"[INIT] {endpoint}, "
        f"source={args.source_system}/{args.source_component}, "
        f"dialect={DIALECT}",
        flush=True,
    )

    master = mavutil.mavlink_connection(
        endpoint,
        source_system=args.source_system,
        source_component=args.source_component,
        dialect=DIALECT,
    )

    print("[WAIT] PX4 HEARTBEAT ...", flush=True)

    hb = master.wait_heartbeat(timeout=20)

    if hb is None:
        raise SystemExit("[FATAL] no PX4 heartbeat")

    target_system = hb.get_srcSystem()
    target_component = hb.get_srcComponent()

    print(
        f"[LINK] PX4 {target_system}/{target_component}",
        flush=True,
    )

    app = App(
        args,
        master,
        target_system,
        target_component,
    )

    # Ask PX4 for square-navigation feedback.
    app.request_message_interval(
        MSG_ID_LOCAL_POSITION_NED,
        args.feedback_rate,
    )
    app.request_message_interval(
        MSG_ID_ATTITUDE,
        args.feedback_rate,
    )

    print(
        f"[IDLE] ZERO BODY_NED velocity @ {args.rate:.1f} Hz continuously",
        flush=True,
    )
    print("[WAIT] THACO trigger ...", flush=True)

    worker = threading.Thread(
        target=app.tx_loop,
        daemon=True,
        name="thaco_tx",
    )
    worker.start()

    try:
        while not app.stop.is_set():
            app.tick()

            msg = master.recv_match(
                blocking=True,
                timeout=0.05,
            )

            if msg is not None:
                app.handle(msg)

    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C", flush=True)

    finally:
        app.stop.set()
        worker.join(timeout=2.0)

    return 0


if __name__ == "__main__":
    sys.exit(main())
