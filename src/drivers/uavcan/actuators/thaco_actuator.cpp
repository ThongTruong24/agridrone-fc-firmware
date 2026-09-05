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


void UavcanThacoActuatorBridge::broadcast_actuator_command(
	uint8_t enabled_mask,
	hrt_abstime now)
{
	thaco::equipment::actuator::ArrayCommand out{};

	out.timestamp_ms =
		static_cast<uint32_t>(now / 1000ULL);

	bool publish_slot[8]{};
	bool has_command = false;

	for (unsigned slot = 0; slot < 8; slot++) {

		/*
		 * THACO_ACTUATOR bit not enabled
		 */
		if ((enabled_mask & (1u << slot)) == 0) {
			_gripper_states[slot] = GripperState{};
			continue;
		}

		const uint8_t actuator_id =
			get_actuator_id(slot);

		/*
		 * Only GRIPPER1..4 are implemented.
		 */
		if (actuator_id < GRIPPER_ID_MIN
		    || actuator_id > GRIPPER_ID_MAX) {
			_gripper_states[slot] = GripperState{};
			continue;
		}

		const int32_t rc_channel =
			get_rc_channel(slot);

		if (rc_channel <= 0 || rc_channel > 18) {
			_gripper_states[slot] = GripperState{};
			continue;
		}

		const unsigned rc_index = static_cast<unsigned>(rc_channel - 1);

		if (rc_index >= _rc_channels.channel_count
		    || !PX4_ISFINITE(_rc_channels.channels[rc_index])) {
			continue;
		}

		/* Scale the normalized RC input [-1, 1] directly to [0, 255]. */
		const float rc_value = math::constrain(_rc_channels.channels[rc_index], -1.0f, 1.0f);
		const uint8_t value = static_cast<uint8_t>((rc_value + 1.0f) * 127.5f + 0.5f);
		GripperState &state = _gripper_states[slot];

		if (!state.initialized || state.actuator_id != actuator_id) {
			state = GripperState{};
			state.initialized = true;
			state.actuator_id = actuator_id;
			state.value = value;
			state.command_id = _command_id;
			state.publish_pending = true;

		} else if (state.value != value) {
			state.value = value;
			state.command_id = ++_command_id;
			state.publish_pending = true;
		}

		const bool should_publish = state.publish_pending
					    || state.last_publish_time == 0
					    || (now - state.last_publish_time) >= REFRESH_INTERVAL_US;

		if (!should_publish) {
			continue;
		}

		thaco::equipment::actuator::Command cmd{};

		cmd.actuator_id = actuator_id;

		cmd.value = state.value;
		cmd.command_id = state.command_id;

		out.commands.push_back(cmd);

		has_command = true;
		publish_slot[slot] = true;
	}

	if (!has_command) {
		return;
	}

	/*
	 * Limit both state-change transmissions and refresh transmissions
	 * to one ArrayCommand transfer every 20 ms (50 Hz maximum).
	 */
	if (_last_publish_time != 0
	    && (now - _last_publish_time) < (1000000 / MAX_RATE_HZ)) {
		return;
	}

	_last_publish_time = now;

	if (_uavcan_pub_actuator_cmd.broadcast(out) >= 0) {
		for (unsigned slot = 0; slot < 8; slot++) {
			if (publish_slot[slot]) {
				_gripper_states[slot].publish_pending = false;
				_gripper_states[slot].last_publish_time = now;
			}
		}
	}
}


void UavcanThacoActuatorBridge::update()
{
	rc_channels_s rc{};

	if (_rc_channels_sub.update(&rc)) {
		_rc_channels = rc;
	}

	int32_t mask =
		_param_thaco_actuator.get();

	/*
	 * Main enable condition
	 */
	if (mask <= 0) {
		_last_publish_time = 0;

		for (GripperState &state : _gripper_states) {
			state = GripperState{};
		}

		return;
	}

	if (mask > 255) {
		mask = 255;
	}

	const hrt_abstime now =
		hrt_absolute_time();

	broadcast_actuator_command(
		static_cast<uint8_t>(mask),
		now
	);
}
