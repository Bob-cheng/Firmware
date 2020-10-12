/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "log_compressor.h"

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_outputs.h>


int LogCompressor::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int LogCompressor::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int LogCompressor::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("log_compressor",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      30000, //stack size
				      (px4_main_t)&run_trampoline, //entry
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

LogCompressor *LogCompressor::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	LogCompressor *instance = new LogCompressor(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

LogCompressor::LogCompressor(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void LogCompressor::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	int angular_velocity_sub 	= orb_subscribe(ORB_ID(vehicle_angular_velocity));
	int attitude_sub 		= orb_subscribe(ORB_ID(vehicle_attitude));
	int local_position_sub 		= orb_subscribe(ORB_ID(vehicle_local_position));
	int actuator_outputs_sub 	= orb_subscribe(ORB_ID(actuator_outputs));

	px4_pollfd_struct_t fds[4];
	fds[0].fd = angular_velocity_sub; 	fds[0].events = POLLIN;
	fds[1].fd = attitude_sub; 		fds[1].events = POLLIN;
	fds[2].fd = local_position_sub; 	fds[2].events = POLLIN;
	fds[3].fd = actuator_outputs_sub; 	fds[3].events = POLLIN;

	// initialize parameters
	parameters_update(true);
	// PX4_INFO("start info");
	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			struct vehicle_angular_velocity_s angular_velocity;
			struct vehicle_attitude_s attitude;
			struct vehicle_local_position_s local_position;
			struct actuator_outputs_s actuator;
			orb_copy(ORB_ID(vehicle_angular_velocity), 	angular_velocity_sub, 	&angular_velocity);
			orb_copy(ORB_ID(vehicle_attitude), 		attitude_sub, 		&attitude);
			orb_copy(ORB_ID(vehicle_local_position), 	local_position_sub, 	&local_position);
			orb_copy(ORB_ID(actuator_outputs), 		actuator_outputs_sub, 	&actuator);
			//TODO: add log information
		}
		parameters_update();
		usleep(10000); //10 ms, 100 Hz
	}

	orb_unsubscribe(angular_velocity_sub);
	orb_unsubscribe(attitude_sub);
	orb_unsubscribe(local_position_sub);
	orb_unsubscribe(actuator_outputs_sub);
}

void LogCompressor::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int LogCompressor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("LogCompressor", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int log_compressor_main(int argc, char *argv[])
{
	return LogCompressor::main(argc, argv);
}
