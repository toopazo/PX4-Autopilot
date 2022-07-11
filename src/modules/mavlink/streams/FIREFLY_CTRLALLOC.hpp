/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef FIREFLY_CTRLALLOC_HPP
#define FIREFLY_CTRLALLOC_HPP

#include <uORB/topics/firefly_ctrlalloc.h>

class MavlinkStreamFireflyCtrlalloc : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamFireflyCtrlalloc(mavlink); }

	static constexpr const char *get_name_static() { return "FIREFLY_CTRLALLOC"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_FIREFLY_CTRLALLOC; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _fctrlalloc_sub.advertised() ? MAVLINK_MSG_ID_FIREFLY_CTRLALLOC_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamFireflyCtrlalloc(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _fctrlalloc_sub{ORB_ID(firefly_ctrlalloc)};

		bool send() override
	{
		firefly_ctrlalloc_s f_ctrlalloc;

		if (_fctrlalloc_sub.update(&f_ctrlalloc)) {
			mavlink_firefly_ctrlalloc_t msg{};

			// uint64_t timestamp;
			// uint32_t status;
			// uint32_t noutputs;
			// float controls[8];
			// float output[8];
			// float pwm_limited[8];
			// float delta[8];

			msg.time_boot_ms = f_ctrlalloc.timestamp / 1000;
			msg.status = f_ctrlalloc.status;
			msg.noutputs = f_ctrlalloc.noutputs;
			for (unsigned i = 0; i < 8; i++) {
				msg.controls[i] = f_ctrlalloc.controls[i];
				msg.output[i] = f_ctrlalloc.output[i];
				msg.pwm_limited[i] = f_ctrlalloc.pwm_limited[i];
				msg.delta[i] = f_ctrlalloc.delta[i];
			}

			mavlink_msg_firefly_ctrlalloc_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // FIREFLY_CTRLALLOC_HPP
