#include "thaco_actuator.hpp"

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>


UavcanThacoActuatorBridge::UavcanThacoActuatorBridge(uavcan::INode &node) :
	ModuleParams(nullptr),
	_node(node),
	_uavcan_pub_actuator_cmd(node)
{
	_uavcan_pub_actuator_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);

	_uavcan_pub_actuator_cmd.setTxTimeout(
		uavcan::MonotonicDuration::fromMSec(20)
	);
}


int UavcanThacoActuatorBridge::init()
{
	update_params();

	return _uavcan_pub_actuator_cmd.init();
}


void UavcanThacoActuatorBridge::update_params()
{
	ModuleParams::updateParams();
}


uint8_t UavcanThacoActuatorBridge::get_actuator_id(unsigned slot) const
{
	switch (slot) {
	case 0:
		return static_cast<uint8_t>(_param_thaco_id_a1.get());

	case 1:
		return static_cast<uint8_t>(_param_thaco_id_a2.get());

	case 2:
		return static_cast<uint8_t>(_param_thaco_id_a3.get());

	case 3:
		return static_cast<uint8_t>(_param_thaco_id_a4.get());

	case 4:
		return static_cast<uint8_t>(_param_thaco_id_a5.get());

	case 5:
		return static_cast<uint8_t>(_param_thaco_id_a6.get());

	case 6:
		return static_cast<uint8_t>(_param_thaco_id_a7.get());

	case 7:
		return static_cast<uint8_t>(_param_thaco_id_a8.get());

	default:
		return ACTUATOR_ID_NONE;
	}
}


int32_t UavcanThacoActuatorBridge::get_rc_channel(unsigned slot) const
{
	switch (slot) {
	case 0:
		return _param_thaco_rc_a1.get();

	case 1:
		return _param_thaco_rc_a2.get();

	case 2:
		return _param_thaco_rc_a3.get();

	case 3:
		return _param_thaco_rc_a4.get();

	case 4:
		return _param_thaco_rc_a5.get();

	case 5:
		return _param_thaco_rc_a6.get();

	case 6:
		return _param_thaco_rc_a7.get();

	case 7:
		return _param_thaco_rc_a8.get();

	default:
		return 0;
	}
}


uint8_t UavcanThacoActuatorBridge::rc_to_value(
	uint8_t actuator_id,
	float rc_value) const
{
	rc_value = math::constrain(rc_value, -1.0f, 1.0f);

	/*
	 * GRIPPER1..4
	 *
	 * 0 = close
	 * 1 = open
	 */
	if (actuator_id >= GRIPPER_ID_MIN
	    && actuator_id <= GRIPPER_ID_MAX) {

		return rc_value > 0.0f ? 1 : 0;
	}

	/*
	 * PUMP1..4
	 *
	 * 0 = off
	 * 1 = on
	 */
	if (actuator_id >= PUMP_ID_MIN
	    && actuator_id <= PUMP_ID_MAX) {

		return rc_value > 0.0f ? 1 : 0;
	}

	/*
	 * SERVO1..4
	 *
	 * RC -1 ... +1
	 * ->
	 * value 0 ... 255
	 */
	if (actuator_id >= SERVO_ID_MIN
	    && actuator_id <= SERVO_ID_MAX) {

		const float value =
			(rc_value + 1.0f) * 127.5f;

		return static_cast<uint8_t>(
			       math::constrain(value, 0.0f, 255.0f) + 0.5f
		       );
	}

	return 0;
}


void UavcanThacoActuatorBridge::broadcast_actuator_command(
	uint8_t enabled_mask,
	hrt_abstime now)
{
	thaco::equipment::actuator::ArrayCommand out{};

	out.timestamp_ms =
		static_cast<uint32_t>(now / 1000ULL);

	bool has_command = false;

	for (unsigned slot = 0; slot < 8; slot++) {

		/*
		 * THACO_ACTUATOR bit not enabled
		 */
		if ((enabled_mask & (1u << slot)) == 0) {
			continue;
		}

		const uint8_t actuator_id =
			get_actuator_id(slot);

		/*
		 * Invalid / NONE actuator
		 */
		if (actuator_id == ACTUATOR_ID_NONE
		    || actuator_id > SERVO_ID_MAX) {

			continue;
		}

		const int32_t rc_channel =
			get_rc_channel(slot);

		/*
		 * RC = 0 means no RC source assigned.
		 *
		 * For now do not send this actuator.
		 * Later another command source can be added here.
		 */
		if (rc_channel <= 0 || rc_channel > 18) {
			continue;
		}

		const unsigned rc_index =
			static_cast<unsigned>(rc_channel - 1);

		if (rc_index >= _rc_channels.channel_count) {
			continue;
		}

		const float rc_value =
			_rc_channels.channels[rc_index];

		if (!PX4_ISFINITE(rc_value)) {
			continue;
		}

		thaco::equipment::actuator::Command cmd{};

		cmd.actuator_id = actuator_id;

		cmd.value =
			rc_to_value(actuator_id, rc_value);

		cmd.command_id = ++_command_id;

		out.commands.push_back(cmd);

		has_command = true;
	}

	if (has_command) {
		(void)_uavcan_pub_actuator_cmd.broadcast(out);
	}
}


void UavcanThacoActuatorBridge::update()
{
	rc_channels_s rc{};

	if (_rc_channels_sub.update(&rc)) {
		_rc_channels = rc;
		_has_rc = true;
	}

	int32_t mask =
		_param_thaco_actuator.get();

	/*
	 * Main enable condition
	 */
	if (mask <= 0) {
		_last_publish_time = 0;
		return;
	}

	if (mask > 255) {
		mask = 255;
	}

	/*
	 * Currently RC is the only command source.
	 */
	if (!_has_rc) {
		return;
	}

	/*
	 * Never command actuators from lost RC signal.
	 */
	if (_rc_channels.signal_lost) {
		return;
	}

	const hrt_abstime now =
		hrt_absolute_time();

	/*
	 * Reject stale RC data.
	 */
	if (_rc_channels.timestamp_last_valid == 0
	    || (now - _rc_channels.timestamp_last_valid) > 500000) {

		return;
	}

	/*
	 * Max 50 Hz broadcast rate.
	 */
	if (_last_publish_time != 0
	    && (now - _last_publish_time)
	    < (1000000 / MAX_RATE_HZ)) {

		return;
	}

	_last_publish_time = now;

	broadcast_actuator_command(
		static_cast<uint8_t>(mask),
		now
	);
}
