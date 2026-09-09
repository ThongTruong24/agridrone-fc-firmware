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
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/thaco_actuator_command.h>
#include <uORB/topics/thaco_actuator_setpoint.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class ThacoActuator : public ModuleBase<ThacoActuator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ThacoActuator();
	~ThacoActuator() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;
	bool init();

private:
	static constexpr hrt_abstime RC_TIMEOUT_US = 500_ms;
	static constexpr hrt_abstime COMMAND_TIMEOUT_US = 500_ms;
	static constexpr hrt_abstime DEBUG_INTERVAL_US = 100_ms;

	void Run() override;
	static uint8_t scale_rc_value(float rc_value);
	int32_t get_rc_channel(unsigned actuator_index) const;
	hrt_abstime rc_last_valid_timestamp() const;
	bool rc_channel_valid(unsigned actuator_index, hrt_abstime now) const;
	void publish_rc_commands(hrt_abstime now);
	void print_debug(hrt_abstime now);
	bool process_command(const thaco_actuator_command_s &command, hrt_abstime now);
	bool update_output_policy(hrt_abstime now);
	bool safety_inhibited() const;
	void update_limits();
	void set_limits(unsigned actuator_index, int32_t minimum, int32_t maximum, int32_t default_value);
	void publish_setpoint();

	struct ActuatorLimits {
		int32_t minimum{0};
		int32_t maximum{255};
		uint8_t default_value{0};
		bool valid{true};
	};

	uORB::SubscriptionCallbackWorkItem _command_sub{this, ORB_ID(thaco_actuator_command)};
	uORB::SubscriptionCallbackWorkItem _rc_channels_sub{this, ORB_ID(rc_channels)};
	uORB::SubscriptionCallbackWorkItem _actuator_armed_sub{this, ORB_ID(actuator_armed)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<thaco_actuator_command_s> _command_pub{ORB_ID(thaco_actuator_command)};
	uORB::Publication<thaco_actuator_setpoint_s> _setpoint_pub{ORB_ID(thaco_actuator_setpoint)};

	thaco_actuator_setpoint_s _setpoint{};
	uint8_t _source[thaco_actuator_setpoint_s::NUM_ACTUATORS] {};
	uint8_t _command_value[thaco_actuator_setpoint_s::NUM_ACTUATORS] {};
	hrt_abstime _command_timestamp[thaco_actuator_setpoint_s::NUM_ACTUATORS] {};
	ActuatorLimits _limits[thaco_actuator_setpoint_s::NUM_ACTUATORS] {};
	actuator_armed_s _actuator_armed{};
	vehicle_status_s _vehicle_status{};
	input_rc_s _input_rc{};
	rc_channels_s _rc_channels{};
	bool _actuator_armed_received{false};
	bool _input_rc_received{false};
	bool _rc_channels_received{false};
	hrt_abstime _last_debug_print{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::THACO_ARM_MODE>) _param_thaco_arm_mode,
		(ParamBool<px4::params::THACO_DEBUG>) _param_thaco_debug,
		(ParamInt<px4::params::THACO_RC_A1>) _param_thaco_rc_a1,
		(ParamInt<px4::params::THACO_RC_A2>) _param_thaco_rc_a2,
		(ParamInt<px4::params::THACO_RC_A3>) _param_thaco_rc_a3,
		(ParamInt<px4::params::THACO_RC_A4>) _param_thaco_rc_a4,
		(ParamInt<px4::params::THACO_RC_A5>) _param_thaco_rc_a5,
		(ParamInt<px4::params::THACO_RC_A6>) _param_thaco_rc_a6,
		(ParamInt<px4::params::THACO_RC_A7>) _param_thaco_rc_a7,
		(ParamInt<px4::params::THACO_RC_A8>) _param_thaco_rc_a8,

		(ParamInt<px4::params::THACO_A1_MIN>) _param_thaco_a1_min,
		(ParamInt<px4::params::THACO_A1_MAX>) _param_thaco_a1_max,
		(ParamInt<px4::params::THACO_A1_DEF>) _param_thaco_a1_def,
		(ParamInt<px4::params::THACO_A2_MIN>) _param_thaco_a2_min,
		(ParamInt<px4::params::THACO_A2_MAX>) _param_thaco_a2_max,
		(ParamInt<px4::params::THACO_A2_DEF>) _param_thaco_a2_def,
		(ParamInt<px4::params::THACO_A3_MIN>) _param_thaco_a3_min,
		(ParamInt<px4::params::THACO_A3_MAX>) _param_thaco_a3_max,
		(ParamInt<px4::params::THACO_A3_DEF>) _param_thaco_a3_def,
		(ParamInt<px4::params::THACO_A4_MIN>) _param_thaco_a4_min,
		(ParamInt<px4::params::THACO_A4_MAX>) _param_thaco_a4_max,
		(ParamInt<px4::params::THACO_A4_DEF>) _param_thaco_a4_def,
		(ParamInt<px4::params::THACO_A5_MIN>) _param_thaco_a5_min,
		(ParamInt<px4::params::THACO_A5_MAX>) _param_thaco_a5_max,
		(ParamInt<px4::params::THACO_A5_DEF>) _param_thaco_a5_def,
		(ParamInt<px4::params::THACO_A6_MIN>) _param_thaco_a6_min,
		(ParamInt<px4::params::THACO_A6_MAX>) _param_thaco_a6_max,
		(ParamInt<px4::params::THACO_A6_DEF>) _param_thaco_a6_def,
		(ParamInt<px4::params::THACO_A7_MIN>) _param_thaco_a7_min,
		(ParamInt<px4::params::THACO_A7_MAX>) _param_thaco_a7_max,
		(ParamInt<px4::params::THACO_A7_DEF>) _param_thaco_a7_def,
		(ParamInt<px4::params::THACO_A8_MIN>) _param_thaco_a8_min,
		(ParamInt<px4::params::THACO_A8_MAX>) _param_thaco_a8_max,
		(ParamInt<px4::params::THACO_A8_DEF>) _param_thaco_a8_def
	)
};
