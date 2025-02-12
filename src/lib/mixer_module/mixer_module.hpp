/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "actuator_test.hpp"
#include "functions.hpp"

#include <board_config.h>
#include <drivers/drv_pwm_output.h>
#include <lib/mixer/MixerGroup.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/test_motor.h>
/* toopazo */
#include <uORB/topics/firefly_ctrlalloc.h>
#include <uORB/topics/firefly_delta.h>
#include <uORB/topics/rc_channels.h>
/* toopazo */

using namespace time_literals;

/**
 * @class OutputModuleInterface
 * Base class for an output module.
 */
class OutputModuleInterface : public px4::ScheduledWorkItem, public ModuleParams
{
public:
	static constexpr int MAX_ACTUATORS = PWM_OUTPUT_MAX_CHANNELS;

	OutputModuleInterface(const char *name, const px4::wq_config_t &config)
		: px4::ScheduledWorkItem(name, config), ModuleParams(nullptr) {}

	/**
	 * Callback to update the (physical) actuator outputs in the driver
	 * @param stop_motors if true, all motors must be stopped (if false, individual motors
	 *                    might still be stopped via outputs[i] == disarmed_value)
	 * @param outputs individual actuator outputs in range [min, max] or failsafe/disarmed value
	 * @param num_outputs number of outputs (<= max_num_outputs)
	 * @param num_control_groups_updated number of actuator_control groups updated
	 * @return if true, the update got handled, and actuator_outputs can be published
	 */
	virtual bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
				   unsigned num_outputs, unsigned num_control_groups_updated) = 0;

	/** called whenever the mixer gets updated/reset */
	virtual void mixerChanged() {}
};

/**
 * @class MixingOutput
 * This handles the mixing, arming/disarming and all subscriptions required for that.
 *
 * It can also drive the scheduling of the OutputModuleInterface (via uORB callbacks
 * to reduce output latency).
 */
class MixingOutput : public ModuleParams
{
public:
	static constexpr int MAX_ACTUATORS = OutputModuleInterface::MAX_ACTUATORS;

	enum class SchedulingPolicy {
		Disabled, ///< Do not drive scheduling (the module needs to call ScheduleOnInterval() for example)
		Auto ///< Drive scheduling based on subscribed actuator controls topics (via uORB callbacks)
	};

	/**
	 * Constructor
	 * @param param_prefix for min/max/etc. params, e.g. "PWM_MAIN". This needs to match 'param_prefix' in the module.yaml
	 * @param max_num_outputs maximum number of supported outputs
	 * @param interface Parent module for scheduling, parameter updates and callbacks
	 * @param scheduling_policy
	 * @param support_esc_calibration true if the output module supports ESC calibration via max, then min setting
	 * @param ramp_up true if motor ramp up from disarmed to min upon arming is wanted
	 */
	MixingOutput(const char *param_prefix, uint8_t max_num_outputs, OutputModuleInterface &interface,
		     SchedulingPolicy scheduling_policy,
		     bool support_esc_calibration, bool ramp_up = true);

	~MixingOutput();

	void setDriverInstance(uint8_t instance) { _driver_instance = instance; }

	void printStatus() const;

	bool useDynamicMixing() const { return _use_dynamic_mixing; }

	/**
	 * Permanently disable an output function
	 */
	void disableFunction(int index) { _param_handles[index].function = PARAM_INVALID; _need_function_update = true; }

	/**
	 * Check if a function is configured, i.e. not set to Disabled and initialized
	 */
	bool isFunctionSet(int index) const { return !_use_dynamic_mixing || _functions[index] != nullptr; }

	OutputFunction outputFunction(int index) const { return _function_assignment[index]; }

	/**
	 * Call this regularly from Run(). It will call interface.updateOutputs().
	 * @return true if outputs were updated
	 */
	bool update();

	/**
	 * Check for subscription updates (e.g. after a mixer is loaded).
	 * Call this at the very end of Run() if allow_wq_switch
	 * @param allow_wq_switch if true
	 * @param limit_callbacks_to_primary set to only register callbacks for primary actuator controls (if used)
	 * @return true if subscriptions got changed
	 */
	bool updateSubscriptions(bool allow_wq_switch = false, bool limit_callbacks_to_primary = false);

	/**
	 * unregister uORB subscription callbacks
	 */
	void unregister();

	void setMaxTopicUpdateRate(unsigned max_topic_update_interval_us);

	/**
	 * Reset (unload) the complete mixer.
	 */
	void resetMixer();

	/**
	 * Load (append) a new mixer from a buffer.
	 * @return 0 on success, <0 error otherwise
	 */
	int loadMixer(const char *buf, unsigned len);

	const actuator_armed_s &armed() const { return _armed; }

	bool initialized() const { return _use_dynamic_mixing || _mixers != nullptr; }

	MixerGroup *mixers() const { return _mixers; }

	void setAllFailsafeValues(uint16_t value);
	void setAllDisarmedValues(uint16_t value);
	void setAllMinValues(uint16_t value);
	void setAllMaxValues(uint16_t value);

	uint16_t &reverseOutputMask() { return _reverse_output_mask; }
	uint16_t &failsafeValue(int index) { return _failsafe_value[index]; }
	/** Disarmed values: disarmedValue < minValue needs to hold */
	uint16_t &disarmedValue(int index) { return _disarmed_value[index]; }
	uint16_t &minValue(int index) { return _min_value[index]; }
	uint16_t &maxValue(int index) { return _max_value[index]; }

	/**
	 * Returns the actual failsafe value taking into account the assigned function
	 */
	uint16_t actualFailsafeValue(int index) const;

	/**
	 * Get the motor index that maps from PX4 convention to the configured one
	 * @param index motor index in [0, num_motors-1]
	 * @return reordered motor index. When out of range, the input index is returned
	 */
	int reorderedMotorIndex(int index) const;

	void setIgnoreLockdown(bool ignore_lockdown) { _ignore_lockdown = ignore_lockdown; }

	/**
	 * Set the maximum number of outputs. This can only be used to reduce the maximum.
	 */
	void setMaxNumOutputs(uint8_t max_num_outputs) { if (max_num_outputs < _max_num_outputs) { _max_num_outputs = max_num_outputs; } }

	const char *paramPrefix() const { return _param_prefix; }

	void setLowrateSchedulingInterval(hrt_abstime interval) { _lowrate_schedule_interval = interval; }

	/**
	 * Get the bitmask of reversible outputs (motors only).
	 * This might change at any time (while disarmed), so output drivers requiring this should query this regularly.
	 */
	uint32_t reversibleOutputs() const { return _reversible_mask; }

protected:
	void updateParams() override;

private:
	bool updateSubscriptionsStaticMixer(bool allow_wq_switch, bool limit_callbacks_to_primary);
	bool updateSubscriptionsDynamicMixer(bool allow_wq_switch, bool limit_callbacks_to_primary);

	bool updateStaticMixer();
	bool updateDynamicMixer();

	bool armNoThrottle() const
	{
		return (_armed.prearmed && !_armed.armed) || _armed.in_esc_calibration_mode;
	}

	unsigned motorTest();

	void updateOutputSlewrateMultirotorMixer();
	void updateOutputSlewrateSimplemixer();
	void setAndPublishActuatorOutputs(unsigned num_outputs, actuator_outputs_s &actuator_outputs);
	void publishMixerStatus(const actuator_outputs_s &actuator_outputs);
	void updateLatencyPerfCounter(const actuator_outputs_s &actuator_outputs);

	static int controlCallback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input);

	void cleanupFunctions();

	void initParamHandles();

	void limitAndUpdateOutputs(float outputs[MAX_ACTUATORS], bool has_updates);

	uint16_t output_limit_calc_single(int i, float value) const;

	void output_limit_calc(const bool armed, const int num_channels, const float outputs[MAX_ACTUATORS]);

	struct ParamHandles {
		param_t function{PARAM_INVALID};
		param_t disarmed{PARAM_INVALID};
		param_t min{PARAM_INVALID};
		param_t max{PARAM_INVALID};
		param_t failsafe{PARAM_INVALID};
	};

	enum class MotorOrdering : int32_t {
		PX4 = 0,
		Betaflight = 1
	};

	/**
	 * Reorder outputs according to _param_mot_ordering
	 * @param values values to reorder
	 */
	inline void reorderOutputs(uint16_t values[MAX_ACTUATORS]);

	void lock() { do {} while (px4_sem_wait(&_lock) != 0); }
	void unlock() { px4_sem_post(&_lock); }

	px4_sem_t _lock; /**< lock to protect access to work queue changes (includes ScheduleNow calls from another thread) */

	uint16_t _failsafe_value[MAX_ACTUATORS] {};
	uint16_t _disarmed_value[MAX_ACTUATORS] {};
	uint16_t _min_value[MAX_ACTUATORS] {};
	uint16_t _max_value[MAX_ACTUATORS] {};
	uint16_t _current_output_value[MAX_ACTUATORS] {}; ///< current output values (reordered)
	uint16_t _reverse_output_mask{0}; ///< reverses the interval [min, max] -> [max, min], NOT motor direction

	enum class OutputLimitState {
		OFF = 0,
		INIT,
		RAMP,
		ON
	} _output_state{OutputLimitState::INIT};

	hrt_abstime _output_time_armed{0};
	const bool _output_ramp_up; ///< if true, motors will ramp up from disarmed to min_output after arming

	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	/* toopazo */
	uORB::Subscription _delta_sub{ORB_ID(firefly_delta)};
	uORB::Subscription _rcchan_sub{ORB_ID(rc_channels)};
	/* toopazo */
	uORB::SubscriptionCallbackWorkItem _control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

	uORB::PublicationMulti<actuator_outputs_s> _outputs_pub{ORB_ID(actuator_outputs)};
	uORB::PublicationMulti<control_allocator_status_s> _control_allocator_status_pub{ORB_ID(control_allocator_status)};
	/* toopazo */
	uORB::Publication<firefly_ctrlalloc_s> _ctrlalloc_pub{ORB_ID(firefly_ctrlalloc)};
	/* toopazo */

	actuator_controls_s _controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS] {};
	actuator_armed_s _armed{};
	/* toopazo */
	firefly_delta_s _delta{};
	rc_channels_s _rcchan{};
	/* toopazo */


	hrt_abstime _time_last_dt_update_multicopter{0};
	hrt_abstime _time_last_dt_update_simple_mixer{0};
	unsigned _max_topic_update_interval_us{0}; ///< max _control_subs topic update interval (0=unlimited)

	bool _throttle_armed{false};
	bool _ignore_lockdown{false}; ///< if true, ignore the _armed.lockdown flag (for HIL outputs)

	MixerGroup *_mixers{nullptr};
	uint32_t _groups_required{0};
	uint32_t _groups_subscribed{1u << 31}; ///< initialize to a different value than _groups_required and outside of (1 << NUM_ACTUATOR_CONTROL_GROUPS)

	const SchedulingPolicy _scheduling_policy;
	const bool _support_esc_calibration;

	bool _wq_switched{false};
	uint8_t _driver_instance{0}; ///< for boards that supports multiple outputs (e.g. PX4IO + FMU)
	uint8_t _max_num_outputs;

	struct MotorTest {
		uORB::Subscription test_motor_sub{ORB_ID(test_motor)};
		bool in_test_mode{false};
		hrt_abstime timeout{0};
	};
	MotorTest _motor_test;

	OutputModuleInterface &_interface;

	perf_counter_t _control_latency_perf;

	/* SYS_CTRL_ALLOC == 1 */
	FunctionProviderBase *_function_allocated[MAX_ACTUATORS] {}; ///< unique allocated functions
	FunctionProviderBase *_functions[MAX_ACTUATORS] {}; ///< currently assigned functions
	OutputFunction _function_assignment[MAX_ACTUATORS] {};
	bool _need_function_update{true};
	bool _use_dynamic_mixing{false}; ///< set to _param_sys_ctrl_alloc on init (avoid changing after startup)
	bool _has_backup_schedule{false};
	const char *const _param_prefix;
	ParamHandles _param_handles[MAX_ACTUATORS];
	param_t _param_handle_rev_range{PARAM_INVALID};
	hrt_abstime _lowrate_schedule_interval{300_ms};
	ActuatorTest _actuator_test{_function_assignment};
	uint32_t _reversible_mask{0}; ///< per-output bits. If set, the output is configured to be reversible (motors only)

	uORB::SubscriptionCallbackWorkItem *_subscription_callback{nullptr}; ///< current scheduling callback


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MC_AIRMODE>) _param_mc_airmode,   ///< multicopter air-mode
		(ParamFloat<px4::params::MOT_SLEW_MAX>) _param_mot_slew_max,
		(ParamFloat<px4::params::THR_MDL_FAC>) _param_thr_mdl_fac, ///< thrust to motor control signal modelling factor
		(ParamInt<px4::params::MOT_ORDERING>) _param_mot_ordering,
		(ParamBool<px4::params::SYS_CTRL_ALLOC>) _param_sys_ctrl_alloc

	)
};
