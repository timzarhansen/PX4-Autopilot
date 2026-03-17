/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "USVControl.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

using namespace time_literals;

namespace usv_control
{

USVControl::USVControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

USVControl::~USVControl()
{
	perf_free(_loop_perf);
}

int USVControl::task_spawn(int argc, char *argv[])
{
	_usv_control = new USVControl();

	if (_usv_control == nullptr) {
		PX4_ERR("alloc failed");
		return -1;
	}

	if (_usv_control->init() != true) {
		delete _usv_control;
		_usv_control = nullptr;
		PX4_ERR("init failed");
		return -1;
	}

	return 0;
}

int USVControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int USVControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		"USV Control - Manual control module for surface vehicles");

	PRINT_MODULE_USAGE_NAME("usv_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

bool USVControl::init()
{
	if (!_manual_control_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void USVControl::Run()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	if (_vehicle_status_sub.updated()) {
		_vehicle_status_sub.copy(&_vehicle_status);
	}

	if (_manual_control_sub.updated()) {
		manual_control_setpoint_s manual_control{};
		_manual_control_sub.copy(&manual_control);

		// Check if armed and in manual mode
		if (_vehicle_control_mode.flag_armed &&
		    (_vehicle_control_mode.flag_control_manual_enabled ||
		     _vehicle_control_mode.flag_control_rates_enabled)) {
			generateThrustSetpoint();
		}
	}

	perf_count(_loop_perf);
}

void USVControl::updateParams()
{
	ModuleParams::updateParams();
}

float USVControl::applyExpo(float value, float expo)
{
	if (value < 0.0f) {
		return -applyExpo(-value, expo);
	}

	return powf(value, expo);
}

void USVControl::generateThrustSetpoint()
{
	manual_control_setpoint_s manual_control{};
	_manual_control_sub.copy(&manual_control);

	vehicle_thrust_setpoint_s thrust_setpoint{};
	thrust_setpoint.timestamp = hrt_absolute_time();

	// Apply exponential to stick inputs
	float thrust_input = applyExpo(manual_control.throttle, _param_thrust_expo.get());
	float yaw_input = applyExpo(manual_control.yaw, _param_yaw_expo.get());

	// Scale thrust to maximum
	float thrust_max = _param_thrust_max.get();
	float thrust_x = thrust_input * thrust_max;

	// Scale yaw to maximum rate
	float yaw_max = _param_yaw_rate_max.get() * M_DEG_TO_RAD_F;
	float yaw_rate = yaw_input * yaw_max;

	// Set thrust setpoints
	// For surface vehicle: X = forward thrust, Y = lateral thrust
	thrust_setpoint.x = thrust_x;
	thrust_setpoint.y = 0.0f;  // No lateral thrust in basic mode
	thrust_setpoint.z = 0.0f;  // No vertical thrust (surface vehicle)

	// Set yaw rate setpoint
	vehicle_attitude_setpoint_s attitude_setpoint{};
	attitude_setpoint.timestamp = thrust_setpoint.timestamp;
	attitude_setpoint.yaw_sp_move = yaw_rate;

	// Publish setpoints
	_thrust_setpoint_pub.publish(thrust_setpoint);
	_attitude_setpoint_pub.publish(attitude_setpoint);
}

} // namespace usv_control

extern "C" __EXPORT int usv_control_main(int argc, char *argv[])
{
	return usv_control::USVControl::main(argc, argv);
}
