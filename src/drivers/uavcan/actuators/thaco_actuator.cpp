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

#include "thaco_actuator.hpp"

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>

UavcanThacoActuatorController::UavcanThacoActuatorController(uavcan::INode &node) :
	ModuleParams(nullptr),
	_uavcan_pub_actuator_cmd(node)
{
	_uavcan_pub_actuator_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);
	_uavcan_pub_actuator_cmd.setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
}

int UavcanThacoActuatorController::init()
{
	update_params();
	const int result = _uavcan_pub_actuator_cmd.init();
	_initialized = result >= 0;
	return result;
}

void UavcanThacoActuatorController::update_params()
{
	ModuleParams::updateParams();
}

uint8_t UavcanThacoActuatorController::get_actuator_id(unsigned slot) const
{
	switch (slot) {
	case 0: return static_cast<uint8_t>(_param_thaco_id_a1.get());

	case 1: return static_cast<uint8_t>(_param_thaco_id_a2.get());

	case 2: return static_cast<uint8_t>(_param_thaco_id_a3.get());

	case 3: return static_cast<uint8_t>(_param_thaco_id_a4.get());

	case 4: return static_cast<uint8_t>(_param_thaco_id_a5.get());

	case 5: return static_cast<uint8_t>(_param_thaco_id_a6.get());

	case 6: return static_cast<uint8_t>(_param_thaco_id_a7.get());

	case 7: return static_cast<uint8_t>(_param_thaco_id_a8.get());

	default: return ACTUATOR_ID_NONE;
	}
}

void UavcanThacoActuatorController::update()
{
	if (!_initialized) {
		return;
	}

	thaco_actuator_setpoint_s setpoint{};

	if (_setpoint_sub.update(&setpoint)) {
		_setpoint = setpoint;
		_setpoint_received = true;
	}

	if (!_setpoint_received) {
		return;
	}

	const int32_t parameter_mask = _param_thaco_actuator.get();

	if (parameter_mask <= 0) {
		for (ActuatorState &state : _actuator_states) {
			state = ActuatorState{};
		}

		return;
	}

	const uint8_t enabled_mask = static_cast<uint8_t>(math::constrain(parameter_mask, int32_t{0}, int32_t{255}));
	const hrt_abstime now = hrt_absolute_time();

	thaco::equipment::actuator::ArrayCommand message{};
	message.timestamp_ms = static_cast<uint32_t>(now / 1000ULL);

	bool publish_slot[MAX_ACTUATORS] {};
	bool has_command = false;

	for (unsigned slot = 0; slot < MAX_ACTUATORS; ++slot) {
		if ((enabled_mask & (1u << slot)) == 0) {
			_actuator_states[slot] = ActuatorState{};
			continue;
		}

		const uint8_t actuator_id = get_actuator_id(slot);

		// Preserve the current backend behavior: only Gripper 1..4 are emitted.
		if (actuator_id < GRIPPER_ID_MIN || actuator_id > GRIPPER_ID_MAX) {
			_actuator_states[slot] = ActuatorState{};
			continue;
		}

		const uint8_t value = _setpoint.value[slot];
		ActuatorState &state = _actuator_states[slot];

		if (!state.initialized || state.actuator_id != actuator_id) {
			state = ActuatorState{};
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

		thaco::equipment::actuator::Command command{};
		command.actuator_id = actuator_id;
		command.value = state.value;
		command.command_id = state.command_id;
		message.commands.push_back(command);

		publish_slot[slot] = true;
		has_command = true;
	}

	if (!has_command) {
		return;
	}

	if (_param_thaco_refresh_id.get()) {
		unsigned command_index = 0;

		for (unsigned slot = 0; slot < MAX_ACTUATORS; ++slot) {
			if (!publish_slot[slot]) {
				continue;
			}

			ActuatorState &state = _actuator_states[slot];

			if (!state.publish_pending) {
				state.command_id = ++_command_id;
				message.commands[command_index].command_id = state.command_id;
			}

			++command_index;
		}
	}

	if (_uavcan_pub_actuator_cmd.broadcast(message) >= 0) {
		const bool print_debug = _param_thaco_debug.get()
					 && (_last_debug_print == 0 || (now - _last_debug_print) >= DEBUG_INTERVAL_US);

		for (unsigned slot = 0; slot < MAX_ACTUATORS; ++slot) {
			if (publish_slot[slot]) {
				if (print_debug) {
					PX4_INFO("THACO TX A%u: id=%u value=%u command_id=%u type=%s", slot + 1,
						 _actuator_states[slot].actuator_id, _actuator_states[slot].value,
						 _actuator_states[slot].command_id,
						 _actuator_states[slot].publish_pending ? "new" : "refresh");
				}

				_actuator_states[slot].publish_pending = false;
				_actuator_states[slot].last_publish_time = now;
			}
		}

		if (print_debug) {
			_last_debug_print = now;
		}
	}
}
