/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ThacoActuator.hpp"

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

ThacoActuator::ThacoActuator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool ThacoActuator::init()
{
	_command_pub.advertise();
	updateParams();
	update_limits();
	update_output_policy(hrt_absolute_time());
	publish_setpoint();

	if (!_command_sub.registerCallback()) {
		PX4_ERR("command callback registration failed");
		return false;
	}

	if (!_rc_channels_sub.registerCallback()) {
		PX4_ERR("RC callback registration failed");
		_command_sub.unregisterCallback();
		return false;
	}

	if (!_actuator_armed_sub.registerCallback()) {
		PX4_ERR("actuator armed callback registration failed");
		_command_sub.unregisterCallback();
		_rc_channels_sub.unregisterCallback();
		return false;
	}

	ScheduleOnInterval(100_ms);
	return true;
}

int32_t ThacoActuator::get_rc_channel(unsigned actuator_index) const
{
	switch (actuator_index) {
	case 0: return _param_thaco_rc_a1.get();

	case 1: return _param_thaco_rc_a2.get();

	case 2: return _param_thaco_rc_a3.get();

	case 3: return _param_thaco_rc_a4.get();

	case 4: return _param_thaco_rc_a5.get();

	case 5: return _param_thaco_rc_a6.get();

	case 6: return _param_thaco_rc_a7.get();

	case 7: return _param_thaco_rc_a8.get();

	default: return 0;
	}
}

uint8_t ThacoActuator::scale_rc_value(float rc_value)
{
	const float normalized = math::constrain(rc_value, -1.0f, 1.0f);
	return static_cast<uint8_t>((normalized + 1.0f) * 127.5f + 0.5f);
}

bool ThacoActuator::rc_channel_valid(unsigned actuator_index, hrt_abstime now) const
{
	const hrt_abstime last_valid_timestamp = rc_last_valid_timestamp();

	if (!_rc_channels_received || _rc_channels.signal_lost || last_valid_timestamp == 0
	    || now < last_valid_timestamp
	    || (now - last_valid_timestamp) > RC_TIMEOUT_US) {
		return false;
	}

	const int32_t rc_channel = get_rc_channel(actuator_index);

	if (rc_channel <= 0 || rc_channel > 18) {
		return false;
	}

	const unsigned rc_index = static_cast<unsigned>(rc_channel - 1);
	return rc_index < _rc_channels.channel_count && PX4_ISFINITE(_rc_channels.channels[rc_index]);
}

hrt_abstime ThacoActuator::rc_last_valid_timestamp() const
{
	// rc_update in this PX4 branch puts input_rc.timestamp_last_signal in timestamp,
	// but leaves timestamp_last_valid unset. Prefer the dedicated field when a
	// publisher provides it and retain compatibility with this branch otherwise.
	return _rc_channels.timestamp_last_valid != 0 ? _rc_channels.timestamp_last_valid : _rc_channels.timestamp;
}

void ThacoActuator::publish_rc_commands(hrt_abstime now)
{
	for (unsigned actuator_index = 0; actuator_index < thaco_actuator_setpoint_s::NUM_ACTUATORS; ++actuator_index) {
		if (!rc_channel_valid(actuator_index, now)) {
			continue;
		}

		const unsigned rc_index = static_cast<unsigned>(get_rc_channel(actuator_index) - 1);
		thaco_actuator_command_s command{};
		command.timestamp = now;
		command.actuator_index = actuator_index;
		command.value = scale_rc_value(_rc_channels.channels[rc_index]);
		command.source = thaco_actuator_command_s::SOURCE_RC;
		_command_pub.publish(command);
	}
}

void ThacoActuator::print_debug(hrt_abstime now)
{
	if (!_param_thaco_debug.get()
	    || (_last_debug_print != 0 && (now - _last_debug_print) < DEBUG_INTERVAL_US)) {
		return;
	}

	_last_debug_print = now;
	bool mapped_channel = false;

	for (unsigned actuator_index = 0; actuator_index < thaco_actuator_setpoint_s::NUM_ACTUATORS; ++actuator_index) {
		const int32_t rc_channel = get_rc_channel(actuator_index);

		if (rc_channel <= 0 || rc_channel > input_rc_s::RC_INPUT_MAX_CHANNELS) {
			continue;
		}

		mapped_channel = true;
		const unsigned rc_index = static_cast<unsigned>(rc_channel - 1);
		const bool pwm_valid = _input_rc_received && rc_index < _input_rc.channel_count;
		const bool normalized_valid = _rc_channels_received && rc_index < _rc_channels.channel_count
					      && PX4_ISFINITE(_rc_channels.channels[rc_index]);
		const uint16_t pwm = pwm_valid ? _input_rc.values[rc_index] : 0;
		const float normalized = normalized_valid ? math::constrain(_rc_channels.channels[rc_index], -1.0f, 1.0f) : 0.0f;
		const uint8_t scaled_value = normalized_valid ? scale_rc_value(normalized) : 0;
		const bool output_valid = _setpoint.valid[actuator_index];

		PX4_INFO("THACO RX A%u CH%ld: pwm=%u(%u) norm=%.3f value=%u rc_ok=%u output=%u(%u)",
			 actuator_index + 1, static_cast<long>(rc_channel), pwm, pwm_valid, static_cast<double>(normalized),
			 scaled_value, rc_channel_valid(actuator_index, now), _setpoint.value[actuator_index], output_valid);
	}

	if (!mapped_channel) {
		PX4_INFO("THACO RX: no RC channel mapped");
	}
}

bool ThacoActuator::safety_inhibited() const
{
	return _actuator_armed.lockdown || _actuator_armed.kill || _actuator_armed.termination
	       || _vehicle_status.failsafe;
}

void ThacoActuator::set_limits(unsigned actuator_index, int32_t minimum, int32_t maximum, int32_t default_value)
{
	ActuatorLimits &limits = _limits[actuator_index];
	limits.valid = minimum >= 0 && maximum <= 255 && minimum <= maximum;

	if (limits.valid) {
		limits.minimum = minimum;
		limits.maximum = maximum;

	} else {
		// An inverted interval cannot be clamped. Retain the last valid bounds
		// (0..255 at startup), apply DEF, and inhibit commands for this slot.
		PX4_WARN("A%u invalid MIN/MAX: %ld/%ld; using default", actuator_index + 1,
			 static_cast<long>(minimum), static_cast<long>(maximum));
	}

	limits.default_value = static_cast<uint8_t>(math::constrain(default_value, limits.minimum, limits.maximum));
}

void ThacoActuator::update_limits()
{
	set_limits(0, _param_thaco_a1_min.get(), _param_thaco_a1_max.get(), _param_thaco_a1_def.get());
	set_limits(1, _param_thaco_a2_min.get(), _param_thaco_a2_max.get(), _param_thaco_a2_def.get());
	set_limits(2, _param_thaco_a3_min.get(), _param_thaco_a3_max.get(), _param_thaco_a3_def.get());
	set_limits(3, _param_thaco_a4_min.get(), _param_thaco_a4_max.get(), _param_thaco_a4_def.get());
	set_limits(4, _param_thaco_a5_min.get(), _param_thaco_a5_max.get(), _param_thaco_a5_def.get());
	set_limits(5, _param_thaco_a6_min.get(), _param_thaco_a6_max.get(), _param_thaco_a6_def.get());
	set_limits(6, _param_thaco_a7_min.get(), _param_thaco_a7_max.get(), _param_thaco_a7_def.get());
	set_limits(7, _param_thaco_a8_min.get(), _param_thaco_a8_max.get(), _param_thaco_a8_def.get());
}

bool ThacoActuator::process_command(const thaco_actuator_command_s &command, hrt_abstime now)
{
	if (command.actuator_index > thaco_actuator_command_s::ACTUATOR_INDEX_MAX) {
		PX4_WARN("invalid actuator index: %u", command.actuator_index);
		return false;
	}

	if (command.source > thaco_actuator_command_s::SOURCE_INTERNAL) {
		PX4_WARN("invalid command source: %u", command.source);
		return false;
	}

	if (!_actuator_armed_received || safety_inhibited()) {
		return false;
	}

	if (!_limits[command.actuator_index].valid || command.timestamp == 0 || command.timestamp > now
	    || (now - command.timestamp) > COMMAND_TIMEOUT_US) {
		return false;
	}

	if (command.source == thaco_actuator_command_s::SOURCE_RC
	    && !rc_channel_valid(command.actuator_index, now)) {
		return false;
	}

	if (!_actuator_armed.armed
	    && (_param_thaco_arm_mode.get() != 1 || command.source != thaco_actuator_command_s::SOURCE_RC)) {
		return false;
	}

	const ActuatorLimits &limits = _limits[command.actuator_index];
	_setpoint.value[command.actuator_index] = static_cast<uint8_t>(math::constrain<int32_t>(
				command.value, limits.minimum, limits.maximum));
	_setpoint.valid[command.actuator_index] = true;
	_source[command.actuator_index] = command.source;
	_command_value[command.actuator_index] = command.value;
	_command_timestamp[command.actuator_index] = command.timestamp;
	return true;
}

bool ThacoActuator::update_output_policy(hrt_abstime now)
{
	bool setpoint_changed = false;

	for (unsigned actuator_index = 0; actuator_index < thaco_actuator_setpoint_s::NUM_ACTUATORS; ++actuator_index) {
		const ActuatorLimits &limits = _limits[actuator_index];
		const bool rc_source = _source[actuator_index] == thaco_actuator_command_s::SOURCE_RC;
		const bool valid_rc = rc_source && rc_channel_valid(actuator_index, now);
		const bool source_allowed = _actuator_armed_received
					    && !safety_inhibited()
					    && (_actuator_armed.armed
						|| (_param_thaco_arm_mode.get() == 1 && rc_source && valid_rc));

		const hrt_abstime timestamp = _command_timestamp[actuator_index];
		const bool fresh_command = timestamp != 0 && timestamp <= now && (now - timestamp) <= COMMAND_TIMEOUT_US;
		const bool valid = _setpoint.valid[actuator_index] && limits.valid && source_allowed
				   && fresh_command && (!rc_source || valid_rc);
		const uint8_t value = valid ? static_cast<uint8_t>(math::constrain<int32_t>(
					      _command_value[actuator_index], limits.minimum, limits.maximum)) : limits.default_value;
		setpoint_changed |= _setpoint.valid[actuator_index] != valid || _setpoint.value[actuator_index] != value;
		_setpoint.valid[actuator_index] = valid;
		_setpoint.value[actuator_index] = value;
	}

	return setpoint_changed;
}

void ThacoActuator::publish_setpoint()
{
	_setpoint.timestamp = hrt_absolute_time();
	_setpoint_pub.publish(_setpoint);
}

void ThacoActuator::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_command_sub.unregisterCallback();
		_rc_channels_sub.unregisterCallback();
		_actuator_armed_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update{};
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
		update_limits();
	}

	const hrt_abstime now = hrt_absolute_time();
	_vehicle_status_sub.update(&_vehicle_status);
	actuator_armed_s actuator_armed{};

	if (_actuator_armed_sub.update(&actuator_armed)) {
		_actuator_armed = actuator_armed;
		_actuator_armed_received = true;
	}

	if (_input_rc_sub.update(&_input_rc)) {
		_input_rc_received = true;
	}

	if (_rc_channels_sub.update(&_rc_channels)) {
		_rc_channels_received = true;
		publish_rc_commands(now);
	}

	bool setpoint_changed = false;

	// Handle MAVLink THACO actuator control commands (44001)
	vehicle_command_s vcmd{};
	while (_vehicle_command_sub.update(&vcmd)) {
		if (vcmd.command == THACO_MAV_CMD_ACTUATOR_CONTROL) {
			uint8_t result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

			// param1: actuator_index (0-7)
			if (!PX4_ISFINITE(vcmd.param1)) {
				result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
			} else {
				float p1 = vcmd.param1;
				float p1_trunc = truncf(p1);
				if (fabsf(p1 - p1_trunc) > 1e-6f || p1 < 0.0f || p1 > 7.0f) {
					result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
				}
			}

			// param2: value (0-255)
			if (result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED && !PX4_ISFINITE(vcmd.param2)) {
				result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
			} else if (result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
				float p2 = vcmd.param2;
				float p2_trunc = truncf(p2);
				if (fabsf(p2 - p2_trunc) > 1e-6f || p2 < 0.0f || p2 > 255.0f) {
					result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
				}
			}

			// param3..7: reserved - DO NOT VALIDATE

			if (result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
				thaco_actuator_command_s cmd{};
				cmd.timestamp = vcmd.timestamp;
				cmd.actuator_index = static_cast<uint8_t>(vcmd.param1);
				cmd.value = static_cast<uint8_t>(vcmd.param2);
				cmd.source = thaco_actuator_command_s::SOURCE_MAVLINK;

				bool accepted = process_command(cmd, now);
				result = accepted ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED :
						    vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
				setpoint_changed = true; // ensure fresh setpoint publication after MAVLink command
			}

			if (vcmd.from_external) {
				vehicle_command_ack_s ack{};
				ack.timestamp = hrt_absolute_time();
				ack.command = vcmd.command;
				ack.result = result;
				ack.target_system = vcmd.source_system;
				ack.target_component = vcmd.source_component;
				ack.from_external = false;
				ack.result_param1 = 0;
				ack.result_param2 = 0;
				_vehicle_command_ack_pub.publish(ack);
			}
		}
	}

	thaco_actuator_command_s command{};

	while (_command_sub.update(&command)) {
		setpoint_changed |= process_command(command, now);
	}

	setpoint_changed |= update_output_policy(now);

	if (setpoint_changed) {
		publish_setpoint();
	}

	print_debug(now);
}

int ThacoActuator::task_spawn(int argc, char *argv[])
{
	ThacoActuator *instance = new ThacoActuator();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (!instance->init()) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int ThacoActuator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ThacoActuator::print_status()
{
	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime rc_timestamp = rc_last_valid_timestamp();
	const unsigned long rc_age_ms = (rc_timestamp != 0 && now >= rc_timestamp) ?
					static_cast<unsigned long>((now - rc_timestamp) / 1000) : 0;

	PX4_INFO("armed=%u received=%u lockdown=%u kill=%u termination=%u arm_mode=%ld",
		 _actuator_armed.armed, _actuator_armed_received, _actuator_armed.lockdown, _actuator_armed.kill,
		 _actuator_armed.termination,
		 static_cast<long>(_param_thaco_arm_mode.get()));
	PX4_INFO("RC: received=%u lost=%u channels=%u age=%lu ms timestamp_last_valid=%llu timestamp=%llu",
		 _rc_channels_received, _rc_channels.signal_lost, _rc_channels.channel_count, rc_age_ms,
		 static_cast<unsigned long long>(_rc_channels.timestamp_last_valid),
		 static_cast<unsigned long long>(_rc_channels.timestamp));

	for (unsigned i = 0; i < thaco_actuator_setpoint_s::NUM_ACTUATORS; ++i) {
		PX4_INFO("A%u: valid=%u value=%u source=%u rc_channel=%ld rc_valid=%u", i + 1, _setpoint.valid[i],
			 _setpoint.value[i], _source[i], static_cast<long>(get_rc_channel(i)), rc_channel_valid(i, now));
	}

	return 0;
}

int ThacoActuator::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Owns THACO actuator state and publishes the validated eight-channel setpoint.
RC is currently the only input adapter; MAVLink and DDS adapters can publish
thaco_actuator_command without coupling to the DroneCAN backend.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("thaco_actuator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int thaco_actuator_main(int argc, char *argv[])
{
	return ThacoActuator::main(argc, argv);
}
