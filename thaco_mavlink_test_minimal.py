#!/usr/bin/env python3
"""
Minimal THACO PX4 <-> Companion MAVLink test (NO MAVROS)

Logic:
  WAIT_TRIGGER
    -> receive THACO_EXTERNAL_XYZ_TRIGGER(N)
    -> pre-stream ZERO BODY_NED
    -> request OFFBOARD
    -> wait until PX4 HEARTBEAT confirms main_mode == OFFBOARD
    -> send BODY_NED velocity (+ optional 44001 actuator refresh)
    -> after duration, switch BODY velocity to ZERO
    -> send COMPLETE(N) = command 44002
    -> keep ZERO BODY_NED while waiting
    -> IN_PROGRESS: keep waiting/streaming
    -> final ACCEPTED: stop stream and finish

Important:
- The script does NOT track full PX4 modes such as MISSION/RTL/LOITER.
- It only needs one flight-state fact: "is PX4 currently OFFBOARD?"
- During ACTIVE, leaving OFFBOARD cancels the local test transaction.
- During WAIT_FINAL, leaving OFFBOARD is expected because PX4 is resuming Mission.
"""

import argparse
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
    PX4_REPO, "src/modules/mavlink/mavlink/message_definitions"
)

from pymavlink import mavutil

CMD_DO_SET_MODE = 176
CMD_THACO_ACTUATOR = 44001
CMD_THACO_COMPLETE = 44002

MAV_RESULT_ACCEPTED = 0
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_RESULT_DENIED = 2
MAV_RESULT_FAILED = 4
MAV_RESULT_IN_PROGRESS = 5
MAV_RESULT_CANCELLED = 6

FRAME_BODY_NED = 8
VELOCITY_ONLY_MASK = 3527

PX4_MAIN_OFFBOARD = 6

WAIT_TRIGGER = "WAIT_TRIGGER"
PRESTREAM = "PRESTREAM"
WAIT_OFFBOARD = "WAIT_OFFBOARD"
ACTIVE = "ACTIVE"
WAIT_FINAL = "WAIT_FINAL"
DONE = "DONE"


def now():
    return time.monotonic()


def boot_ms():
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


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
    }.get(int(result), str(result))


class App:
    def __init__(self, args, master, target_system, target_component):
        self.args = args
        self.master = master
        self.target_system = target_system
        self.target_component = target_component

        self.state = WAIT_TRIGGER
        self.trigger_id = None
        self.state_since = now()
        self.active_since = None
        self.last_complete = None

        self.is_offboard = False

        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.tx_lock = threading.Lock()

    def set_state(self, state):
        if self.state != state:
            print(f"[STATE] {self.state} -> {state}", flush=True)
            self.state = state
            self.state_since = now()

    def send_setpoint(self, vx, vy, vz):
        with self.tx_lock:
            self.master.mav.set_position_target_local_ned_send(
                boot_ms(),
                self.target_system,
                self.target_component,
                FRAME_BODY_NED,
                VELOCITY_ONLY_MASK,
                0, 0, 0,
                float(vx), float(vy), float(vz),
                0, 0, 0,
                0, 0,
            )

    def send_offboard(self):
        with self.tx_lock:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                CMD_DO_SET_MODE,
                0,
                1.0, 6.0, 0.0,
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
                0, 0, 0, 0, 0,
            )

    def send_complete(self):
        with self.tx_lock:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                CMD_THACO_COMPLETE,
                0,
                float(self.trigger_id),
                0, 0, 0, 0, 0, 0,
            )

    def send_heartbeat(self):
        with self.tx_lock:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )

    def tx_loop(self):
        sp_period = 1.0 / self.args.rate
        act_period = 1.0 / self.args.actuator_rate

        next_sp = now()
        next_act = now()
        next_hb = now()

        while not self.stop.is_set():
            t = now()

            if t >= next_hb:
                self.send_heartbeat()
                next_hb = t + 1.0

            if self.state in (PRESTREAM, WAIT_OFFBOARD, ACTIVE, WAIT_FINAL):
                if t >= next_sp:
                    if self.state == ACTIVE:
                        self.send_setpoint(
                            self.args.vx,
                            self.args.vy,
                            self.args.vz,
                        )
                    else:
                        self.send_setpoint(0.0, 0.0, 0.0)
                    next_sp = t + sp_period
            else:
                next_sp = t

            if (
                self.state == ACTIVE
                and not self.args.no_actuator
                and t >= next_act
            ):
                self.send_actuator()
                next_act = t + act_period
            elif self.state != ACTIVE:
                next_act = t

            self.stop.wait(0.002)

    def tick(self):
        t = now()

        if self.state == PRESTREAM:
            if t - self.state_since >= self.args.prestream:
                print("[TX] request OFFBOARD", flush=True)
                self.send_offboard()
                self.set_state(WAIT_OFFBOARD)

        elif self.state == WAIT_OFFBOARD:
            if t - self.state_since > self.args.offboard_timeout:
                print("[FAIL] OFFBOARD not confirmed", flush=True)
                self.stop.set()

        elif self.state == ACTIVE:
            if self.active_since is not None and t - self.active_since >= self.args.duration:
                self.set_state(WAIT_FINAL)
                self.send_setpoint(0, 0, 0)
                print(f"[TX] COMPLETE({self.trigger_id})", flush=True)
                self.send_complete()
                self.last_complete = t

        elif self.state == WAIT_FINAL:
            if (
                self.last_complete is not None
                and t - self.last_complete >= self.args.complete_retry
            ):
                print(f"[RETRY] COMPLETE({self.trigger_id})", flush=True)
                self.send_complete()
                self.last_complete = t

    def handle(self, msg):
        mtype = msg.get_type()

        if mtype == "BAD_DATA":
            return

        if mtype == "HEARTBEAT":
            # Only the PX4 autopilot component that established the link.
            if (
                msg.get_srcSystem() != self.target_system
                or msg.get_srcComponent() != self.target_component
            ):
                return

            new_is_offboard = (px4_main_mode(msg) == PX4_MAIN_OFFBOARD)

            if new_is_offboard != self.is_offboard:
                self.is_offboard = new_is_offboard
                print(
                    f"[PX4] OFFBOARD={'YES' if self.is_offboard else 'NO'}",
                    flush=True,
                )

            if self.state == WAIT_OFFBOARD and self.is_offboard:
                self.active_since = now()
                self.set_state(ACTIVE)
                print(
                    f"[PASS] OFFBOARD active; BODY velocity "
                    f"({self.args.vx}, {self.args.vy}, {self.args.vz})",
                    flush=True,
                )

            elif self.state == ACTIVE and not self.is_offboard:
                print(
                    "[CANCEL] PX4 left OFFBOARD during external task",
                    flush=True,
                )
                self.trigger_id = None
                self.active_since = None
                self.set_state(WAIT_TRIGGER)

            return

        if mtype == "THACO_EXTERNAL_XYZ_TRIGGER":
            n = int(msg.trigger_id)

            if self.trigger_id is None:
                self.trigger_id = n
                print(f"[RX] TRIGGER({n})", flush=True)
                self.set_state(PRESTREAM)

            elif n == self.trigger_id:
                print(f"[RX] retry TRIGGER({n})", flush=True)

            else:
                print(
                    f"[WARN] new TRIGGER({n}) while local N={self.trigger_id}",
                    flush=True,
                )
            return

        if mtype != "COMMAND_ACK":
            return

        cmd = int(msg.command)
        res = int(msg.result)

        if cmd == CMD_DO_SET_MODE:
            print(
                f"[ACK] OFFBOARD request: {result_name(res)}({res})",
                flush=True,
            )
            return

        if cmd == CMD_THACO_ACTUATOR:
            # Print only failures to keep log small.
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

        if self.state != WAIT_FINAL:
            return

        if res == MAV_RESULT_IN_PROGRESS:
            # Keep ZERO BODY_NED streaming.
            return

        if res == MAV_RESULT_ACCEPTED:
            self.set_state(DONE)
            print("[PASS] transaction complete", flush=True)
            self.stop.set()
            return

        if res == MAV_RESULT_TEMPORARILY_REJECTED:
            # Keep zero stream and retry same COMPLETE(N).
            return

        if res in (
            MAV_RESULT_DENIED,
            MAV_RESULT_FAILED,
            MAV_RESULT_CANCELLED,
        ):
            print("[FAIL] transaction rejected/cancelled", flush=True)
            self.stop.set()


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--listen-port", type=int, default=14601)
    p.add_argument("--source-system", type=int, default=245)
    p.add_argument("--source-component", type=int, default=191)

    p.add_argument("--vx", type=float, default=0.2)
    p.add_argument("--vy", type=float, default=0.0)
    p.add_argument("--vz", type=float, default=0.0)

    p.add_argument("--duration", type=float, default=8.0)
    p.add_argument("--prestream", type=float, default=2.0)
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--offboard-timeout", type=float, default=5.0)

    p.add_argument("--no-actuator", action="store_true")
    p.add_argument("--actuator-index", type=int, default=1)
    p.add_argument("--actuator-value", type=int, default=200)
    p.add_argument("--actuator-rate", type=float, default=10.0)

    p.add_argument("--complete-retry", type=float, default=3.0)
    return p.parse_args()


def main():
    args = parse_args()

    custom_id = getattr(
        mavutil.mavlink,
        "MAVLINK_MSG_ID_THACO_EXTERNAL_XYZ_TRIGGER",
        None,
    )
    if custom_id != 32000:
        raise SystemExit(
            f"[FATAL] THACO dialect not loaded; custom msg id={custom_id!r}"
        )

    endpoint = f"udpin:0.0.0.0:{args.listen_port}"

    print(
        f"[INIT] {endpoint}, source="
        f"{args.source_system}/{args.source_component}",
        flush=True,
    )

    master = mavutil.mavlink_connection(
        endpoint,
        source_system=args.source_system,
        source_component=args.source_component,
        dialect=DIALECT,
    )

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

    worker = threading.Thread(
        target=app.tx_loop,
        daemon=True,
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
        print("[STOP] Ctrl+C", flush=True)

    finally:
        app.stop.set()
        worker.join(timeout=2)

    return 0


if __name__ == "__main__":
    sys.exit(main())
