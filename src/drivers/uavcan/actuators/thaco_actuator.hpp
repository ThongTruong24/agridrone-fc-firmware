#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/rc_channels.h>

#include <uavcan/uavcan.hpp>

#include <thaco/equipment/actuator/Command.hpp>
#include <thaco/equipment/actuator/ArrayCommand.hpp>

#include <stdint.h>

class UavcanThacoActuatorBridge : public ModuleParams
{
public:
	static constexpr unsigned MAX_RATE_HZ = 50;
	static constexpr hrt_abstime REFRESH_INTERVAL_US = 100000;
	static constexpr unsigned UAVCAN_COMMAND_TRANSFER_PRIORITY = 6;

	UavcanThacoActuatorBridge(uavcan::INode &node);

	int init();

	void update_params();

	void update();

private:

	static constexpr uint8_t ACTUATOR_ID_NONE = 0;

	static constexpr uint8_t GRIPPER_ID_MIN = 1;
	static constexpr uint8_t GRIPPER_ID_MAX = 4;

	struct GripperState {
		bool initialized{false};
		uint8_t actuator_id{ACTUATOR_ID_NONE};
		uint8_t value{0};
		uint16_t command_id{0};
		bool publish_pending{false};
		hrt_abstime last_publish_time{0};
	};

	uint8_t get_actuator_id(unsigned slot) const;

	int32_t get_rc_channel(unsigned slot) const;

	void broadcast_actuator_command(uint8_t enabled_mask, hrt_abstime now);

	uavcan::INode &_node;

	uavcan::Publisher<thaco::equipment::actuator::ArrayCommand>
	_uavcan_pub_actuator_cmd;

	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};

	rc_channels_s _rc_channels{};
	GripperState _gripper_states[8]{};

	uint16_t _command_id{0};

	hrt_abstime _last_publish_time{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::THACO_ACTUATOR>) _param_thaco_actuator,

		(ParamInt<px4::params::THACO_ID_A1>) _param_thaco_id_a1,
		(ParamInt<px4::params::THACO_ID_A2>) _param_thaco_id_a2,
		(ParamInt<px4::params::THACO_ID_A3>) _param_thaco_id_a3,
		(ParamInt<px4::params::THACO_ID_A4>) _param_thaco_id_a4,
		(ParamInt<px4::params::THACO_ID_A5>) _param_thaco_id_a5,
		(ParamInt<px4::params::THACO_ID_A6>) _param_thaco_id_a6,
		(ParamInt<px4::params::THACO_ID_A7>) _param_thaco_id_a7,
		(ParamInt<px4::params::THACO_ID_A8>) _param_thaco_id_a8,

		(ParamInt<px4::params::THACO_RC_A1>) _param_thaco_rc_a1,
		(ParamInt<px4::params::THACO_RC_A2>) _param_thaco_rc_a2,
		(ParamInt<px4::params::THACO_RC_A3>) _param_thaco_rc_a3,
		(ParamInt<px4::params::THACO_RC_A4>) _param_thaco_rc_a4,
		(ParamInt<px4::params::THACO_RC_A5>) _param_thaco_rc_a5,
		(ParamInt<px4::params::THACO_RC_A6>) _param_thaco_rc_a6,
		(ParamInt<px4::params::THACO_RC_A7>) _param_thaco_rc_a7,
		(ParamInt<px4::params::THACO_RC_A8>) _param_thaco_rc_a8
	)
};
