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
#include <lib/matrix/matrix/Quaternion.hpp>
#include <lib/matrix/matrix/Euler.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/clog_states.h>


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
	struct clog_states_s clog_states_out;
	memset(&clog_states_out, 0, sizeof(clog_states_out));
	orb_advert_t clog_states_pub = orb_advertise(ORB_ID(clog_states), &clog_states_out);

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
	PX4_INFO("start log compressor");

	float actuator_norm[4] = {0};

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
			orb_copy(ORB_ID(vehicle_angular_velocity), 	angular_velocity_sub, 	&angular_velocity);
			orb_copy(ORB_ID(vehicle_attitude), 		attitude_sub, 		&attitude);
			orb_copy(ORB_ID(vehicle_local_position), 	local_position_sub, 	&local_position);
			orb_copy(ORB_ID(actuator_outputs), 		actuator_outputs_sub, 	&actuator);

			clog_states_out.timestamp = actuator.timestamp;
			clog_states_out.vx = local_position.vx;
			clog_states_out.vy = local_position.vy;
			clog_states_out.vz = local_position.vz;
			clog_states_out.x = local_position.x;
			clog_states_out.y = local_position.y;
			clog_states_out.z = local_position.z;
			actuator_norm[0] = pwm_normalize(actuator.output[0]);
			actuator_norm[1] = pwm_normalize(actuator.output[1]);
			actuator_norm[2] = pwm_normalize(actuator.output[2]);
			actuator_norm[3] = pwm_normalize(actuator.output[3]);
			memcpy(clog_states_out.actuators, 	actuator_norm, 		sizeof(actuator_norm));
			memcpy(clog_states_out.q, 		attitude.q, 		sizeof(attitude.q));
			memcpy(clog_states_out.gyroxyz, 	angular_velocity.xyz, 	sizeof(angular_velocity.xyz));
			orb_publish(ORB_ID(clog_states), clog_states_pub, &clog_states_out);

			compressionLog();
		}

		parameters_update();
		usleep(10000); //10 ms, 100 Hz
	}

	orb_unsubscribe(angular_velocity_sub);
	orb_unsubscribe(attitude_sub);
	orb_unsubscribe(local_position_sub);
	orb_unsubscribe(actuator_outputs_sub);
}



float LogCompressor::pwm_normalize(float pwm)
{
	float out = (pwm - 1000) / 1000;

	if (out < 0) {
		out = 0;

	} else if (out > 1) {
		out = 1;
	}

	return out;
}

void LogCompressor::compressionLog()
{
	static float x[12] = {0};
	static float true_x[12] = {0};
	static float dx[12] = {0};
	static float y[12] = {0};
	static float u[4] = {0};
	static float a = 0.128364;
	static float b = 0.128364;
	static float c = 0.128364;
	static float d = 0.128364;
	static float m = 1.5;
	static float I_x = 0.015;
	static float I_y = 0.015;
	static float I_z = 0.015;
	static float K_T = 7.21077;
	static float K_Q = 0.10472;
	static uint64_t last_time_us = 0;
	static int K_stone = 400;
	static int loopCount = -1;
	static int max_freq = 400;

	static float error_thre[12] = {0.139337476507485, 0.124396874563097, 0.0147272946765464,
				       0.00372856443706101, 0.00390555433968539, 0.00589400093019354,
				       0.0696205367431640, 0.0815225658620695, 0.0264938590124031,
				       0.00822311394035166, 0.00840920644259765, 0.0101633185357302
				      };
	static int last_log_loop[12] = {0};

	uint64_t time_us = actuator.timestamp;

	//prepare dt, unit: s
	float dt = (float)((double)(time_us - last_time_us) * 1e-6);
	last_time_us = time_us;

	//prepare input
	u[0] = transformInput(actuator.output[0]);
	u[1] = transformInput(actuator.output[1]);
	u[2] = transformInput(actuator.output[2]);
	u[3] = transformInput(actuator.output[3]);

	//1. update old states to predict current states.
	updateState(x, dx, dt);

	matrix::Quaternion<float> quat(attitude.q[0], attitude.q[1], attitude.q[2], attitude.q[3]);
	matrix::Euler<float> euler_angles(quat);

	true_x[0] = local_position.x;         true_x[1] = local_position.y;         true_x[2] = local_position.z;
	true_x[3] = euler_angles.phi();       true_x[4] = euler_angles.theta();     true_x[5] = wrap_2PI(euler_angles.psi());
	true_x[6] = local_position.vx;        true_x[7] = local_position.vy;        true_x[8] = local_position.vz;
	true_x[9] = angular_velocity.xyz[0];  true_x[10] = angular_velocity.xyz[1]; true_x[11] = angular_velocity.xyz[2];
	transfromNED2ENU(true_x);

	//2. test if we need synchronization. If so, synchronize.
	loopCount++; //loopCount change from [0, K_stone)

	if (loopCount % K_stone == 0) { //2.1 K-milestone sychronization
		loopCount = 0;

		for (int i = 0; i < 12; i++) {
			x[i] = true_x[i];
			last_log_loop[i] = loopCount;
		}

		//Write syn data
	}

	//3. get current output and calculate dx for next time.
	LogCompressor::quadrotor_m(0.0, x, u, a, b, c, d, m, I_x, I_y, I_z, K_T, K_Q, dx, y);

	//compression log
	for (int i = 0; i < 12; i++) {
		float error = abs(true_x[i] - y[i]);

		if (is_log(error, error_thre[i], last_log_loop[i], loopCount, max_freq)) {
			//need to log
			//write Clog Data
			last_log_loop[i] = loopCount;
		}
	}

}

void LogCompressor::quadrotor_m(float, const float x[12], const float u[4], float a, float b,
				float c, float d, float m, float I_x, float I_y, float I_z,
				float K_T, float K_Q, float dx[12], float y[12])
{
	int i;
	float b_x[16];

	// IDNLGREY model file (discrete-time nonlinear model) :
	// xn = x(t+Ts) : state update values in discrete-time case (A column vector with Nx entries)
	// y : outputs values (A column vector with Ny entries)
	//  gravity acceleration constant (m/s^2)
	//  inputs
	//      x(13:16)=u;
	for (i = 0; i < 12; i++) {
		b_x[i] = x[i];
	}

	for (i = 0; i < 4; i++) {
		b_x[i + 12] = u[i];
	}

	// -------------------------------------------------
	dx[0] = b_x[6];
	dx[1] = b_x[7];
	dx[2] = b_x[8];
	dx[3] = (b_x[9] + sinf(b_x[3]) * tanf(b_x[4]) * b_x[10]) + cosf(b_x[3]) * tanf(b_x[4]) * b_x[11];
	dx[4] = cosf(b_x[3]) * b_x[10] - sinf(b_x[3]) * b_x[11];
	dx[5] = sinf(b_x[3]) / cosf(b_x[4]) * b_x[10] + cosf(b_x[3]) / cosf(b_x[4]) * b_x[11];
	dx[6] = K_T / m * (((b_x[12] + b_x[13]) + b_x[14]) + b_x[15]) * (cosf(b_x[3]) * sinf(b_x[4]) * cosf(b_x[5]) + sinf(
				b_x[3]) * sinf(b_x[5]));
	dx[7] = K_T / m * (((b_x[12] + b_x[13]) + b_x[14]) + b_x[15]) * (cosf(b_x[3]) * sinf(b_x[4]) * sinf(b_x[5]) - sinf(
				b_x[3]) * cosf(b_x[5]));
	dx[8] = K_T / m * (((b_x[12] + b_x[13]) + b_x[14]) + b_x[15]) * (cosf(b_x[3]) * cosf(b_x[4])) - 9.80665F;
	dx[9] = (I_y - I_z) / I_x * b_x[10] * b_x[11] + K_T / I_x * (((-a * b_x[12] + d * b_x[13]) + a * b_x[14]) - d *
			b_x[15]);
	dx[10] = (I_x - I_z) / I_y * b_x[9] * b_x[11] + K_T / I_y * (((-b * b_x[12] + c * b_x[13]) - b * b_x[14]) + c *
			b_x[15]);
	dx[11] = (I_x - I_y) / I_z * b_x[9] * b_x[10] + K_Q / I_z * (((-b_x[12] - b_x[13]) + b_x[14]) + b_x[15]);


	//     %% addtional effects here
	//  air resistance
	//  *0.6538
	// rotational air resitance
	//  *0.2778
	for (i = 0; i < 3; i++) {
		dx[6 + i] += -x[6 + i] * 9.80665F / 15.0F;
		dx[9 + i] += -x[9 + i] * 6.98131704F / 25.1327419F;
	}

	float frame_height = 0.1;

	if ((x[2] - frame_height < 0.001F) && (dx[8] <= 0.0F)) {
		//  on ground
		dx[8] = 0.0F;
	}

	//
	for (i = 0; i < 12; i++) {
		y[i] = x[i];
	}

	//  update outputs
}

void LogCompressor::transfromNED2ENU(float state[12])
{
	float x[12] = {0};
	x[0] = state[1];     x[1] = state[0];     x[2] = -state[2];
	x[3] = state[3];     x[4] = -state[4];    x[5] = wrap_2PI(-state[5] + (float)(M_PI / 2));
	x[6] = state[7];    x[7] = state[6];    x[8] = -state[8];
	x[9] = state[9];    x[10] = -state[10];  x[11] = -state[11];

	for (int i = 0; i < 12; i++) {
		state[i] = x[i];
	}

}

void LogCompressor::updateState(float x[12], float dx[12], float dt)
{
	for (int i = 0; i < 12; i++) {
		x[i] += dx[i] * dt;
	}

	x[5] = wrap_2PI(x[5]);

}


float LogCompressor::wrap_2PI(float radian)
{
	float res = std::fmod(radian, (float)(2 * M_PI));

	if (res < 0) {
		res += (float)(2 * M_PI);
	}

	return res;
}

bool LogCompressor::is_log(float error, float error_max, int last_log_loop, int current_loop, int max_freq)
{
	float scale_factor = 1;

	if (error < 1e-20F || last_log_loop == current_loop) {
		return false;
	}

	if (error >= error_max) {
		return true;

	} else {
		float desired_freq = scale_factor * (error / error_max) * max_freq;
		float current_freq = max_freq * (1 / (float)(current_loop - last_log_loop));

		if (desired_freq > current_freq) {
			return true;

		} else {
			return false;
		}

	}
}

float LogCompressor::transformInput(float actuator_val)
{
	return (((actuator_val * 1000) + 1000) - 1100) / 900;
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
