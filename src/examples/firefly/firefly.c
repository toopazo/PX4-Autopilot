/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file firefly.c
 * Minimal application for Firefly's dynamic control allocation
 *
 * @author Tomas Opazo <toopazo@protonmail.com>
 */

#include <px4_platform_common/log.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/firefly_ctrlalloc.h>
#include <uORB/topics/firefly_delta.h>
// #include <uORB/topics/rc_channels.h>

// __EXPORT int firefly_main(int argc, char *argv[]);

// int firefly_main(int argc, char *argv[])
// {
// 	PX4_INFO("Hello Sky!");
// 	return OK;
// }

__EXPORT int firefly_main(int argc, char *argv[]);

int read_firefly_delta(void)
{
	PX4_INFO("read_firefly_delta");

	/* subscribe to actuator_controls_0 */
	int firefly_delta_sub_fd = orb_subscribe(ORB_ID(firefly_delta));
	struct firefly_delta_s firefly_delta;

	/* check actuator_controls_0 */
	bool updated = false;
	orb_check(firefly_delta_sub_fd, &updated);
	if (updated) {
		orb_copy(ORB_ID(firefly_delta), firefly_delta_sub_fd, &firefly_delta);

		PX4_INFO("firefly_delta.timestamp = %8.4f",
			(double)firefly_delta.timestamp);
		PX4_INFO("firefly_delta.delta = %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f",
			(double)firefly_delta.delta[0], (double)firefly_delta.delta[1],
			(double)firefly_delta.delta[2], (double)firefly_delta.delta[3],
			(double)firefly_delta.delta[4], (double)firefly_delta.delta[5],
			(double)firefly_delta.delta[6], (double)firefly_delta.delta[7]
			);
	}

	orb_unsubscribe(firefly_delta_sub_fd);

	return 0;
}

int read_firefly_ctrlalloc(void)
{
	PX4_INFO("read_firefly_ctrlalloc");


	/* subscribe to firefly_ctrlalloc */
	int ctrlalloc_sub_fd = orb_subscribe(ORB_ID(firefly_ctrlalloc));
	struct firefly_ctrlalloc_s ctrlalloc;

	/* check firefly_ctrlalloc */
	bool updated = false;
	orb_check(ctrlalloc_sub_fd, &updated);
	if (updated) {
		orb_copy(ORB_ID(firefly_ctrlalloc), ctrlalloc_sub_fd, &ctrlalloc);

		PX4_INFO("firefly_ctrlalloc.timestamp\t%8.4f", (double)ctrlalloc.timestamp);
		PX4_INFO("firefly_ctrlalloc.status\t%8.4f", (double)ctrlalloc.status);
		PX4_INFO("firefly_ctrlalloc.status\t%8.4f", (double)ctrlalloc.noutputs);
		PX4_INFO("firefly_ctrlalloc.delta\t\t%8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f",
			(double)ctrlalloc.delta[0], (double)ctrlalloc.delta[1],
			(double)ctrlalloc.delta[2], (double)ctrlalloc.delta[3],
			(double)ctrlalloc.delta[4], (double)ctrlalloc.delta[5],
			(double)ctrlalloc.delta[6], (double)ctrlalloc.delta[7]
			);
		PX4_INFO("firefly_ctrlalloc.output\t%8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f",
			(double)ctrlalloc.output[0], (double)ctrlalloc.output[1],
			(double)ctrlalloc.output[2], (double)ctrlalloc.output[3],
			(double)ctrlalloc.output[4], (double)ctrlalloc.output[5],
			(double)ctrlalloc.output[6], (double)ctrlalloc.output[7]
			);
		PX4_INFO("firefly_ctrlalloc.pwm_limited\t%8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f",
			(double)ctrlalloc.pwm_limited[0], (double)ctrlalloc.pwm_limited[1],
			(double)ctrlalloc.pwm_limited[2], (double)ctrlalloc.pwm_limited[3],
			(double)ctrlalloc.pwm_limited[4], (double)ctrlalloc.pwm_limited[5],
			(double)ctrlalloc.pwm_limited[6], (double)ctrlalloc.pwm_limited[7]
			);
	}

	orb_unsubscribe(ctrlalloc_sub_fd);

	return 0;
}

int read_actuator_controls(void)
{
	PX4_INFO("read_actuator_controls");

	/* subscribe to actuator_controls_0 */
	int actctrl_sub_fd = orb_subscribe(ORB_ID(actuator_controls_0));
	struct actuator_controls_s actctrl;

	/* check actuator_controls_0 */
	bool updated = false;
	orb_check(actctrl_sub_fd, &updated);
	if (updated) {
		orb_copy(ORB_ID(actuator_controls_0), actctrl_sub_fd, &actctrl);

		PX4_INFO("actuator_controls_0.timestamp = %8.4f",
			(double)actctrl.timestamp);
		PX4_INFO("actuator_controls_0 = t%8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f",
			(double)actctrl.control[0], (double)actctrl.control[1],
			(double)actctrl.control[2], (double)actctrl.control[3],
			(double)actctrl.control[4], (double)actctrl.control[5],
			(double)actctrl.control[6], (double)actctrl.control[7]
			);
	}

	orb_unsubscribe(actctrl_sub_fd);

	return 0;
}

int read_actuator_outputs(void)
{
	PX4_INFO("read_actuator_outputs");

	/* subscribe to actuator_outputs */
	int actout_sub_fd = orb_subscribe(ORB_ID(actuator_outputs));
	struct actuator_outputs_s actout;

	/* check actuator_outputs */
	bool updated = false;
	orb_check(actout_sub_fd, &updated);
	if (updated) {
		orb_copy(ORB_ID(actuator_outputs), actout_sub_fd, &actout);

		PX4_INFO("actuator_outputs.timestamp = %8.4f",
			(double)actout.timestamp);
		PX4_INFO("actuator_outputs.output = %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f",
			(double)actout.output[0], (double)actout.output[1],
			(double)actout.output[2], (double)actout.output[3],
			(double)actout.output[4], (double)actout.output[5],
			(double)actout.output[6], (double)actout.output[7]
			);
	}

	orb_unsubscribe(actout_sub_fd);

	return 0;
}

int write_firefly_ctrlalloc(void)
{
	PX4_INFO("write_firefly_ctrlalloc");

	/* advertise firefly_ctrlalloc */
	struct firefly_ctrlalloc_s ctrlalloc;
	memset(&ctrlalloc, 0, sizeof(ctrlalloc));
	orb_advert_t ctrlalloc_pub = orb_advertise(ORB_ID(firefly_ctrlalloc), &ctrlalloc);

	/* Populate ctrlalloc */
	ctrlalloc.timestamp = hrt_absolute_time();
	ctrlalloc.noutputs = 8;
	ctrlalloc.status = ctrlalloc.status + 1;
	for (int i=0; i< 8; i++) {
		ctrlalloc.delta[i] = ctrlalloc.delta[i] + 1;
		ctrlalloc.output[i] = ctrlalloc.output[i] + 1;
	}

	orb_publish(ORB_ID(firefly_ctrlalloc), ctrlalloc_pub, &ctrlalloc);

	return 0;
}


int write_firefly_delta(float delta_front, float delta_back, float status)
{
	PX4_INFO("write_firefly_delta");

	struct firefly_delta_s delta;
	memset(&delta, 0, sizeof(delta));
	orb_advert_t delta_pub = orb_advertise(ORB_ID(firefly_delta), &delta);

	delta.timestamp = hrt_absolute_time();
	delta.noutputs = 8;
	delta.status = (uint32_t)status;
	delta.delta[0] = delta_front;
	delta.delta[1] = delta_front;
	delta.delta[2] = delta_back;
	delta.delta[3] = delta_back;
	delta.delta[4] = 0;
	delta.delta[5] = 0;
	delta.delta[6] = 0;
	delta.delta[7] = 0;
	// for (unsigned i = 0; i < 8; i++) {
	// 	delta.delta[i] = farr[i];
	// }
	orb_publish(ORB_ID(firefly_delta), delta_pub, &delta);

	return 0;
}

int firefly_main(int argc, char *argv[])
{
	PX4_INFO("firefly_main");

	PX4_INFO("argc\t%d", argc);
	for (int i = 0; i < argc; i++) {
		PX4_INFO("argv[%d]=%s", i, argv[i]);
	}

	bool exec_status = false;
	if ((argc == 2) && (strcmp(argv[1], "read_delta") == 0)) {
		read_firefly_delta();
		exec_status = true;
	}
	if ((argc == 5) && (strcmp(argv[1], "write_delta") == 0)) {
		float delta_front = strtof(argv[2], NULL);
		float delta_back = strtof(argv[3], NULL);
		float status = strtof(argv[4], NULL);
		write_firefly_delta(delta_front, delta_back, status);
		exec_status = true;
	}

	if ((argc == 2) && (strcmp(argv[1], "read_ctrlalloc") == 0)) {
		read_firefly_ctrlalloc();
		exec_status = true;
	}
	if ((argc == 2) && (strcmp(argv[1], "write_ctrlalloc") == 0)) {
		// ctrlalloc.timestamp = hrt_absolute_time();
		// ctrlalloc.noutputs = 8;
		// ctrlalloc.status = ctrlalloc.status + 1;
		// for (int i=0; i< 8; i++) {
		// 	ctrlalloc.delta[i] = ctrlalloc.delta[i] + 1;
		// 	ctrlalloc.output[i] = ctrlalloc.output[i] + 1;
		// }
		write_firefly_ctrlalloc();
		exec_status = true;
	}

	if (exec_status) {
		PX4_INFO("Done");
	}
	else {
		PX4_INFO("Incorrect parsing of arguments");
	}

	return 0;
}
