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

#ifndef FIREFLY_DELTA_HPP
#define FIREFLY_DELTA_HPP

#include <uORB/topics/firefly_delta.h>

class MavlinkStreamFireflyDelta : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamFireflyDelta(mavlink); }

	static constexpr const char *get_name_static() { return "FIREFLY_DELTA"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_FIREFLY_DELTA; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _fdelta_sub.advertised() ? MAVLINK_MSG_ID_FIREFLY_DELTA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamFireflyDelta(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _fdelta_sub{ORB_ID(firefly_delta)};

		bool send() override
	{
		firefly_delta_s f_delta;

		if (_fdelta_sub.update(&f_delta)) {
			mavlink_firefly_delta_t msg{};

			// uint64_t timestamp;
			// uint32_t status;
			// uint32_t noutputs;
			// float delta[8];

			msg.time_boot_ms = f_delta.timestamp / 1000;
			msg.status = f_delta.status;
			msg.noutputs = f_delta.noutputs;
			for (unsigned i = 0; i < 8; i++) {
				msg.delta[i] = f_delta.delta[i];
			}

			mavlink_msg_firefly_delta_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // FIREFLY_DELTA_HPP
