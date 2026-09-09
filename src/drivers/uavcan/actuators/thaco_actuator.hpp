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

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/thaco_actuator_setpoint.h>

#include <uavcan/uavcan.hpp>

#include <thaco/equipment/actuator/Command.hpp>
#include <thaco/equipment/actuator/ArrayCommand.hpp>

class UavcanThacoActuatorController : public ModuleParams
{
public:
	static constexpr int MAX_ACTUATORS = thaco_actuator_setpoint_s::NUM_ACTUATORS;
	static constexpr hrt_abstime REFRESH_INTERVAL_US = 100000;
	static constexpr hrt_abstime DEBUG_INTERVAL_US = 100000;
	static constexpr unsigned UAVCAN_COMMAND_TRANSFER_PRIORITY = 6;

	explicit UavcanThacoActuatorController(uavcan::INode &node);

	int init();
	void update_params();
	void update();

private:
	static constexpr uint8_t ACTUATOR_ID_NONE = 0;
	static constexpr uint8_t GRIPPER_ID_MIN = 1;
	static constexpr uint8_t GRIPPER_ID_MAX = 4;

	struct ActuatorState {
		bool initialized{false};
		uint8_t actuator_id{ACTUATOR_ID_NONE};
		uint8_t value{0};
		uint16_t command_id{0};
		bool publish_pending{false};
		hrt_abstime last_publish_time{0};
	};

	uint8_t get_actuator_id(unsigned slot) const;

	bool _initialized{false};
	uavcan::Publisher<thaco::equipment::actuator::ArrayCommand> _uavcan_pub_actuator_cmd;
	uORB::Subscription _setpoint_sub{ORB_ID(thaco_actuator_setpoint)};

	thaco_actuator_setpoint_s _setpoint{};
	bool _setpoint_received{false};
	ActuatorState _actuator_states[MAX_ACTUATORS] {};
	uint16_t _command_id{0};
	hrt_abstime _last_debug_print{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::THACO_ACTUATOR>) _param_thaco_actuator,
		(ParamBool<px4::params::THACO_DEBUG>) _param_thaco_debug,
		(ParamBool<px4::params::THACO_RFRSH_ID>) _param_thaco_refresh_id,

		(ParamInt<px4::params::THACO_ID_A1>) _param_thaco_id_a1,
		(ParamInt<px4::params::THACO_ID_A2>) _param_thaco_id_a2,
		(ParamInt<px4::params::THACO_ID_A3>) _param_thaco_id_a3,
		(ParamInt<px4::params::THACO_ID_A4>) _param_thaco_id_a4,
		(ParamInt<px4::params::THACO_ID_A5>) _param_thaco_id_a5,
		(ParamInt<px4::params::THACO_ID_A6>) _param_thaco_id_a6,
		(ParamInt<px4::params::THACO_ID_A7>) _param_thaco_id_a7,
		(ParamInt<px4::params::THACO_ID_A8>) _param_thaco_id_a8
	)
};
