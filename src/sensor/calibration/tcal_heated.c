#include "globals.h"
#include "tcal_heated.h"

#if CONFIG_SENSOR_TCAL_HEATED
#include "calibration.h"
#include "cal_sample.h"
#include "tcal_runtime.h"
#include "tcal_mls_lut.h"
#include "sensor/sensor.h"
#include "system/heater.h"
#include "system/power.h"
#include "system/system.h"
#include "system/esb_ota.h"
#include "connection/connection.h"
#include "util.h"
#include <errno.h>
#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(tcal_heated, LOG_LEVEL_INF);

#define HEAT_FRESH_MS 2000
#define HEAT_CONTROL_MS 1000

enum heat_state { HEAT_IDLE, HEAT_RESERVED, HEAT_RUNNING, HEAT_FINALIZING, HEAT_FINISHED, HEAT_OFF_FAILED };

/* The shared request mutex serializes admission, controller and publication.
 * Only the sensor owner touches the accumulator and mutable stage. Abort changes
 * the epoch, never the stage: neither user stop nor power waits for a stalled IMU.
 */
static struct {
	enum heat_state state;
	enum tcal_heated_stop_reason reason;
	bool ready;
	bool resetting;
	bool terminal;
	bool sampling;
	bool applied;
	bool start_band_covered;
	uint32_t epoch;
	uint32_t base_generation;
	uint32_t sequence;
	uint32_t ramp_elapsed_ms;
	float target;
	float start_temp;
	float setpoint;
	float integral;
	float previous_temp;
	uint16_t duty;
	uint16_t count;
	float accepted_min;
	float accepted_max;
	int64_t started_ms;
	int64_t control_ms;
	int64_t observed_ms;
	int64_t rise_sampled_ms;
	int64_t rest_since;
	int64_t stable_since;
	struct led_token feedback;
	bool warm_pending;
	uint8_t written_mask;
	uint32_t published_generation;
	uint32_t protection_id;
	enum tcal_heated_stop_reason protection_reason;
} heat;
static struct TempCalPoint stage[TCAL_BUFFER_SIZE];
static uint32_t owner_epoch;

static enum tcal_heated_stop_reason check_locked(struct sensor_temperature_observation *observation);

bool sensor_tcal_heated_busy_locked(void)
{
	return heat.state == HEAT_RESERVED || heat.state == HEAT_RUNNING ||
	       heat.state == HEAT_FINALIZING || heat.state == HEAT_OFF_FAILED;
}

bool sensor_tcal_heated_resetting_locked(void)
{
	return heat.resetting;
}

bool sensor_tcal_heated_busy(void)
{
	sensor_tcal_heated_lock();
	bool busy = sensor_tcal_heated_busy_locked();
	sensor_tcal_heated_unlock();
	return busy;
}

static bool heat_safety_reason(enum tcal_heated_stop_reason reason)
{
	return reason == TCAL_HEATED_STOP_STALE_TEMP || reason == TCAL_HEATED_STOP_OVERTEMP ||
		reason == TCAL_HEATED_STOP_RISE_FAST || reason == TCAL_HEATED_STOP_HEATER_ERROR;
}

static void heat_feedback_finish_locked(enum led_semantic semantic)
{
	sensor_calibration_result(heat.feedback, semantic);
	heat.feedback = (struct led_token){0};
	heat.warm_pending = false;
	led_operation_publish(LED_OWNER_TCAL, false, heat.state == HEAT_OFF_FAILED);
}

/* Existing warm-storage owner supplies the immutable receipt identity. */
void sensor_tcal_feedback_persisted(uint32_t identity, uint8_t written_mask, int result)
{
	sensor_tcal_heated_lock();
	if (heat.warm_pending && identity == heat.feedback.session) {
		heat.written_mask |= written_mask;
		/* The model and its reference gyro temperature are one durable result. */
		if (result < 0 || heat.written_mask == 15) {
			sensor_tcal_lock();
			bool replaced = sensor_tcal_model_generation() != heat.published_generation;
			sensor_tcal_unlock();
			heat_feedback_finish_locked(result < 0 ? LED_APPLIED_NOT_SAVED :
				replaced || heat.reason == TCAL_HEATED_STOP_USER ? LED_PARTIAL : LED_SUCCESS);
		}
	}
	sensor_tcal_heated_unlock();
}

static void heat_safety_refresh_locked(void)
{
	if (!heat.protection_id) {
		return;
	}
	bool safety = heat.state == HEAT_OFF_FAILED || !heater_hw_available();
	struct sensor_temperature_observation observation;
	int err = sensor_get_imu_temperature_observation(&observation, HEAT_FRESH_MS);
	if (heat.protection_reason == TCAL_HEATED_STOP_STALE_TEMP) {
		safety |= err != 0;
	}
	if (err == 0) {
		float ceiling = fminf(CONFIG_SENSOR_TCAL_HEATED_MAX_TEMP_C, heat.target + 2.0f);
		safety |= observation.raw_c >= ceiling || observation.filtered_c >= ceiling;
	}
	/* Clearing current truth does not erase the six-second protection record. */
	led_fault_publish(LED_OWNER_TCAL, safety ? LED_FAULT_SAFETY : LED_FAULT_NONE, 0);
}

/* Gate held; hardware off precedes any release/publication/storage operation. */
static int finish_locked(enum tcal_heated_stop_reason reason)
{
	int err = heater_hw_force_off();
	if (err) {
		LOG_ERR("Heater off failed: %d; hardware unavailable", err);
		reason = TCAL_HEATED_STOP_HEATER_ERROR;
	}
	if (sensor_tcal_heated_busy_locked()) {
		heat.reason = reason;
		heat.state = err ? HEAT_OFF_FAILED : HEAT_FINISHED;
		heat.sampling = false;
		tcal_accum_request_reset();
		heat.duty = 0;
		heat.epoch++;
		/* A failed off is NOT permission for another sensor/calibration owner.
		 * Keep exclusivity until a later explicit off succeeds (or reboot). */
		if (!err) {
			sensor_calibration_heated_release_locked();
		}
		if (heat_safety_reason(reason)) {
			if (!heat.protection_id || heat.protection_reason != reason) {
				heat.protection_id = led_event_id();
				heat.protection_reason = reason;
			}
			/* Hardware protection has already executed; this never delays off. */
			led_fault_publish(LED_OWNER_TCAL,
				err != 0 || !heater_hw_available() ? LED_FAULT_SAFETY : LED_FAULT_NONE, heat.protection_id);
		}
		heat_feedback_finish_locked(reason == TCAL_HEATED_STOP_USER || reason == TCAL_HEATED_STOP_SENSOR_STOP ||
			reason == TCAL_HEATED_STOP_POWER_DOWN ? LED_CANCELLED : LED_FAILED);
	}
	return err;
}

int sensor_tcal_heated_abort(enum tcal_heated_stop_reason reason)
{
	sensor_tcal_heated_lock();
	if (reason == TCAL_HEATED_STOP_SENSOR_STOP || reason == TCAL_HEATED_STOP_POWER_DOWN) {
		heat.ready = false;
	}
	if (reason == TCAL_HEATED_STOP_POWER_DOWN) {
		heat.terminal = true;
	}
	int err = finish_locked(reason);
	sensor_tcal_heated_unlock();
	return err;
}

int sensor_tcal_heated_stop(void)
{
	sensor_tcal_heated_lock();
	if (heat.state == HEAT_RESERVED || heat.state == HEAT_RUNNING) {
		struct sensor_temperature_observation observation;
		enum tcal_heated_stop_reason reason = check_locked(&observation);
		if (reason != TCAL_HEATED_STOP_NONE) {
			int err = finish_locked(reason);
			sensor_tcal_heated_unlock();
			return err;
		}
	}
	int err = heater_hw_force_off();
	if (err) {
		(void)finish_locked(TCAL_HEATED_STOP_HEATER_ERROR);
	} else if (heat.state == HEAT_RESERVED || heat.state == HEAT_RUNNING ||
	           heat.state == HEAT_FINALIZING) {
		/* Freeze accepted RAM slots without waiting for the sensor owner.
		 * An unfinished window is discarded, never flushed by this thread. */
		heat.duty = 0;
		heat.sampling = false;
		tcal_accum_request_reset();
		if (heat.count && owner_epoch == heat.epoch) {
			heat.state = HEAT_FINALIZING;
			heat.reason = TCAL_HEATED_STOP_USER;
		} else {
			(void)finish_locked(TCAL_HEATED_STOP_USER);
		}
	} else if (heat.state == HEAT_OFF_FAILED) {
		(void)finish_locked(TCAL_HEATED_STOP_USER);
	} else if (!heat.feedback.session) {
		led_request_event(LED_OWNER_TCAL, led_request_id(), led_event_id(), LED_CANCELLED);
	}
	sensor_tcal_heated_unlock();
	return err;
}

void sensor_tcal_heated_set_ready(bool ready)
{
	sensor_tcal_heated_lock();
	/* A resumed power owner may reopen admission; terminal shutdown keeps its
	 * atomic power latch closed. No reopening ever starts a session itself. */
	if (ready && heater_power_ready()) {
		heat.terminal = false;
	}
	heat.ready = ready && !heat.resetting && !heat.terminal && heater_power_ready();
	if (!ready) {
		(void)finish_locked(TCAL_HEATED_STOP_SENSOR_STOP);
	}
	sensor_tcal_heated_unlock();
}

void sensor_tcal_heated_clear_begin(void)
{
	sensor_tcal_heated_lock();
	heat.resetting = true;
	heat.ready = false;
	tcal_accum_request_reset();
	if (heat.warm_pending) {
		heat_feedback_finish_locked(LED_CANCELLED);
	}
	(void)finish_locked(TCAL_HEATED_STOP_SENSOR_STOP);
	sensor_tcal_heated_unlock();
}

void sensor_tcal_heated_clear_end(void)
{
	sensor_tcal_heated_lock();
	heat.resetting = false;
	sensor_tcal_heated_unlock();
}

static enum tcal_heated_stop_reason check_locked(struct sensor_temperature_observation *observation)
{
	if (esb_ota_is_active() || connection_get_ota_suppressed()) {
		return TCAL_HEATED_STOP_SENSOR_STOP;
	}
	if (!heater_power_ready() || !heater_external_power_present()) {
		return TCAL_HEATED_STOP_POWER_DOWN;
	}
	if (heater_hw_expired()) {
		return TCAL_HEATED_STOP_STALE_TEMP;
	}
	if (!heater_hw_available()) {
		return TCAL_HEATED_STOP_HEATER_ERROR;
	}
	if (sensor_get_imu_temperature_observation(observation, HEAT_FRESH_MS) ||
	    !v_finite(&observation->raw_c, 1) || !v_finite(&observation->filtered_c, 1)) {
		return TCAL_HEATED_STOP_STALE_TEMP;
	}
	float ceiling = fminf(CONFIG_SENSOR_TCAL_HEATED_MAX_TEMP_C, heat.target + 2.0f);
	if (observation->raw_c >= ceiling || observation->filtered_c >= ceiling) {
		return TCAL_HEATED_STOP_OVERTEMP;
	}
	/* Shared by owner, safety worker and explicit stop: a console command
	 * must not turn an already-observed rise fault into successful saving. */
	int64_t rise_elapsed = observation->sampled_at_ms - heat.rise_sampled_ms;
	if (rise_elapsed >= HEAT_CONTROL_MS &&
	    (observation->raw_c - heat.previous_temp) * 1000000.0f >
	        CONFIG_SENSOR_TCAL_HEATED_MAX_RISE_MCPS * (float)rise_elapsed) {
		return TCAL_HEATED_STOP_RISE_FAST;
	}
	if (k_uptime_get() - heat.started_ms >= (int64_t)CONFIG_SENSOR_TCAL_HEATED_TIMEOUT_MIN * 60000) {
		return TCAL_HEATED_STOP_TIMEOUT;
	}
	if (heat.state == HEAT_RESERVED || heat.state == HEAT_RUNNING) {
		int64_t observation_elapsed = observation->sampled_at_ms - heat.observed_ms;
		if (k_uptime_get() - heat.control_ms >= HEAT_FRESH_MS ||
		    (observation->sequence != heat.sequence &&
		     (observation_elapsed <= 0 || observation_elapsed > HEAT_FRESH_MS))) {
			return TCAL_HEATED_STOP_STALE_TEMP;
		}
	}
	return TCAL_HEATED_STOP_NONE;
}

int sensor_tcal_heated_start(float target_temp)
{
	if (!v_finite(&target_temp, 1) || target_temp < CONFIG_SENSOR_POLY_TEMP_MIN ||
	    target_temp > CONFIG_SENSOR_TCAL_HEATED_MAX_TEMP_C - 2.0f ||
	    target_temp >= CONFIG_SENSOR_POLY_TEMP_MAX || target_temp == 0.0f) {
		return sensor_operation_result(LED_OWNER_TCAL, -EINVAL, false);
	}
	sensor_tcal_heated_lock();
	int err = 0;
	struct sensor_temperature_observation observation;
	float accel[3];
	if (heat.terminal || !heater_power_ready()) {
		err = -ESHUTDOWN;
	} else if (heat.resetting || sensor_tcal_heated_busy_locked() ||
	           esb_ota_is_active() || connection_get_ota_suppressed()) {
		err = -EBUSY;
	} else if (!heat.ready) {
		err = -EAGAIN;
	} else if (!heater_hw_available()) {
		err = -ENODEV;
	} else if (!heater_external_power_present()) {
		err = -ENODEV;
	} else if (sensor_get_imu_temperature_observation(&observation, HEAT_FRESH_MS) ||
	           !v_finite(&observation.raw_c, 1) || !v_finite(&observation.filtered_c, 1)) {
		err = -ENODATA;
	} else if (!sensor_peek_accel_fresh(accel, HEAT_FRESH_MS)) {
		err = -ENODATA;
	} else if (observation.raw_c >= fminf(CONFIG_SENSOR_TCAL_HEATED_MAX_TEMP_C, target_temp + 2.0f) ||
	           observation.filtered_c >= fminf(CONFIG_SENSOR_TCAL_HEATED_MAX_TEMP_C, target_temp + 2.0f) ||
	           observation.raw_c < CONFIG_SENSOR_POLY_TEMP_MIN ||
	           observation.filtered_c < CONFIG_SENSOR_POLY_TEMP_MIN ||
	           observation.filtered_c >= CONFIG_SENSOR_POLY_TEMP_MAX || observation.filtered_c == 0.0f ||
	           TEMP_TO_IDX(observation.filtered_c) < 0 ||
	           TEMP_TO_IDX(observation.filtered_c) >= TCAL_BUFFER_SIZE ||
	           target_temp - fmaxf(observation.raw_c, observation.filtered_c) <
	               (MLS_MIN_POINTS_FOR_FIT - 1.0f) / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE) {
		err = -ERANGE;
	} else {
		err = sensor_calibration_heated_reserve_locked();
	}
	if (!err) {
		tcal_accum_request_reset();
		heat_feedback_finish_locked(LED_CANCELLED);
		heat.protection_id = 0;
		heat.written_mask = 0;
		led_operation_publish(LED_OWNER_TCAL, false, true);
		heat.epoch++;
		/* Generation zero is not a valid hardware session lease. */
		if (!heat.epoch) {
			heat.epoch++;
		}
		heat.state = HEAT_RESERVED;
		heat.reason = TCAL_HEATED_STOP_NONE;
		heat.applied = false;
		heat.sampling = false;
		heat.count = 0;
		heat.start_band_covered = false;
		heat.accepted_min = INFINITY;
		heat.accepted_max = -INFINITY;
		heat.target = target_temp;
		heat.start_temp = observation.filtered_c;
		heat.setpoint = observation.filtered_c;
		heat.ramp_elapsed_ms = 0;
		heat.integral = 0;
		heat.duty = 0;
		heat.started_ms = k_uptime_get();
		heat.control_ms = heat.started_ms;
		heat.observed_ms = observation.sampled_at_ms;
		heat.rise_sampled_ms = observation.sampled_at_ms;
		heat.previous_temp = observation.raw_c;
		heat.sequence = observation.sequence;
		heat.rest_since = -1;
		heat.stable_since = -1;
		sensor_tcal_lock();
		heat.base_generation = sensor_tcal_model_generation();
		sensor_tcal_unlock();
		err = heater_hw_arm(heat.epoch);
		if (err) {
			(void)finish_locked(TCAL_HEATED_STOP_HEATER_ERROR);
		}
		if (!err) {
			heat.feedback = led_begin(LED_OWNER_TCAL, led_request_id());
			sensor_calibration_result(heat.feedback, LED_ACCEPTED);
			sensor_calibration_stage(heat.feedback, LED_PROCESSING);
			led_fault_publish(LED_OWNER_TCAL, LED_FAULT_NONE, 0);
		}
	}
	sensor_tcal_heated_unlock();
	return err ? sensor_operation_result(LED_OWNER_TCAL, err, false) : 0;
}

/* Called only by sensor owner, gate held. */
static void sync_owner_locked(void)
{
	tcal_accum_apply_reset();
	if (owner_epoch != heat.epoch) {
		tcal_accum_reset();
		owner_epoch = heat.epoch;
		if (heat.state == HEAT_RESERVED) {
			memset(stage, 0, sizeof(stage));
			heat.state = HEAT_RUNNING;
			sensor_calibration_stage(heat.feedback, LED_HEATED_ACTIVE);
		}
	}
}

void sensor_tcal_heated_accept_point(int idx, const float bias[3], float temp)
{
	if (heat.state != HEAT_RUNNING || !heat.sampling || owner_epoch != heat.epoch ||
	    !v_finite(&temp, 1) || !v_finite(bias, 3) || temp == 0.0f ||
	    temp < CONFIG_SENSOR_POLY_TEMP_MIN || temp >= CONFIG_SENSOR_POLY_TEMP_MAX ||
	    idx < 0 || idx >= TCAL_BUFFER_SIZE || TEMP_TO_IDX(temp) != idx) {
		return;
	}
	if (stage[idx].temp != 0.0f) {
		return;
	}
	heat.count++;
	stage[idx].temp = temp;
	memcpy(stage[idx].bias, bias, sizeof(stage[idx].bias));
	heat.accepted_min = fminf(heat.accepted_min, temp);
	heat.accepted_max = fmaxf(heat.accepted_max, temp);
	heat.start_band_covered |= fabsf(temp - heat.start_temp) <= 1.0f / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE;
}

bool sensor_tcal_heated_slot_accepted(int idx)
{
	return owner_epoch == heat.epoch && idx >= 0 && idx < TCAL_BUFFER_SIZE &&
	       stage[idx].temp != 0.0f;
}

bool sensor_tcal_heated_start_window(float temp)
{
	return !heat.start_band_covered &&
	       fabsf(temp - heat.start_temp) <= 1.0f / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE;
}

bool sensor_tcal_heated_feed(const float g[3], float temp)
{
	sensor_tcal_heated_lock();
	sync_owner_locked();
	bool busy = sensor_tcal_heated_busy_locked();
	if (heat.state == HEAT_RUNNING && heat.sampling) {
		sensor_tcal_heated_accum_feed(g, temp);
	}
	sensor_tcal_heated_unlock();
	return busy;
}

void sensor_tcal_heated_update(bool is_resting)
{
	sensor_tcal_heated_lock();
	sync_owner_locked();
	if (heat.state != HEAT_RUNNING) {
		sensor_tcal_heated_unlock();
		return;
	}
	struct sensor_temperature_observation observation;
	enum tcal_heated_stop_reason reason = check_locked(&observation);
	int64_t now = k_uptime_get();
	if (reason != TCAL_HEATED_STOP_NONE) {
		(void)finish_locked(reason);
		sensor_tcal_heated_unlock();
		return;
	}
	if (!is_resting) {
		heat.rest_since = -1;
		heat.stable_since = -1;
		heat.sampling = false;
		tcal_accum_reset();
	} else {
		if (heat.rest_since < 0) {
			heat.rest_since = now;
		}
		heat.sampling = now - heat.rest_since >= CONFIG_SENSOR_TCAL_HEATED_RESUME_STABLE_MS;
	}
	bool new_observation = observation.sequence != heat.sequence;
	if (new_observation) {
		int64_t elapsed = observation.sampled_at_ms - heat.observed_ms;
		if (elapsed <= 0 || elapsed > HEAT_FRESH_MS) {
			reason = TCAL_HEATED_STOP_STALE_TEMP;
		}
		/* Raw codes are quantized: adjacent 2.5/10ms differences are not a
		 * physical rate estimate. Check positive rise over >=1s of actual
		 * observations; the absolute raw ceiling above is still immediate. */
		int64_t rise_elapsed = observation.sampled_at_ms - heat.rise_sampled_ms;
		if (rise_elapsed >= HEAT_CONTROL_MS) {
			heat.previous_temp = observation.raw_c;
			heat.rise_sampled_ms = observation.sampled_at_ms;
		}
		heat.observed_ms = observation.sampled_at_ms;
		heat.sequence = observation.sequence;
	}
	int64_t elapsed = now - heat.control_ms;
	if (elapsed >= HEAT_FRESH_MS) {
		reason = TCAL_HEATED_STOP_STALE_TEMP;
	}
	if (reason != TCAL_HEATED_STOP_NONE) {
		(void)finish_locked(reason);
		sensor_tcal_heated_unlock();
		return;
	}
	if (elapsed >= HEAT_CONTROL_MS && new_observation) {
		float dt = (float)elapsed / 1000.0f;
		/* Collect the starting band before ramping away from it. */
		if (heat.sampling && heat.start_band_covered) {
			/* Anchor to total active ramp time: repeated float increments
			 * drift over a long session. Paused control intervals add no time.
			 * Each configured interval advances the reference by 0.5 C. */
			heat.ramp_elapsed_ms += (uint32_t)elapsed;
			heat.setpoint = fminf(heat.target, heat.start_temp +
				0.5f * heat.ramp_elapsed_ms / CONFIG_SENSOR_TCAL_HEATED_RAMP_HALF_DEGREE_MS);
		}
		float error = heat.setpoint - observation.filtered_c;
		float p = CONFIG_SENSOR_TCAL_HEATED_DEFAULT_KP * error;
		float ff = CONFIG_SENSOR_TCAL_HEATED_DEFAULT_KFF * (heat.setpoint - heat.start_temp);
		float integral_limit = CONFIG_SYSTEM_IMU_HEATER_MAX_DUTY_PPTT;
		float next_i = CLAMP(heat.integral + CONFIG_SENSOR_TCAL_HEATED_DEFAULT_KI * error * dt,
		                    -integral_limit, integral_limit);
		float desired = p + ff + next_i;
		/* Signed integral can cancel an explicitly configured feedforward.
		 * Integrate inside either actuator bound, or back toward that range. */
		if ((desired >= 0.0f && desired <= CONFIG_SYSTEM_IMU_HEATER_MAX_DUTY_PPTT) ||
		    (desired > CONFIG_SYSTEM_IMU_HEATER_MAX_DUTY_PPTT && error < 0.0f) ||
		    (desired < 0.0f && error > 0.0f)) {
			heat.integral = next_i;
		}
		desired = CLAMP(p + ff + heat.integral, 0.0f, (float)CONFIG_SYSTEM_IMU_HEATER_MAX_DUTY_PPTT);
		float slew = CONFIG_SENSOR_TCAL_HEATED_SLEW_PPTT_PER_S * dt;
		/* Limit power increases, never delay a requested decrease. */
		desired = fminf(desired, heat.duty + slew);
		if (!v_finite(&desired, 1)) {
			(void)finish_locked(TCAL_HEATED_STOP_HEATER_ERROR);
			sensor_tcal_heated_unlock();
			return;
		}
		heat.duty = (uint16_t)desired;
		heat.control_ms = now;
		if (heater_hw_write(heat.epoch, observation.sequence, observation.sampled_at_ms, observation.raw_c, heat.duty)) {
			(void)finish_locked(TCAL_HEATED_STOP_HEATER_ERROR);
			sensor_tcal_heated_unlock();
			return;
		}
	}
	if (heat.sampling && heat.setpoint >= heat.target &&
	    fabsf(observation.filtered_c - heat.target) <= CONFIG_SENSOR_TCAL_HEATED_STABLE_BAND_MC / 1000.0f) {
		if (heat.stable_since < 0) {
			heat.stable_since = now;
		}
		if (now - heat.stable_since >= CONFIG_SENSOR_TCAL_HEATED_STABLE_MS) {
			/* Final eligible partial bucket is allowed ONLY on normal completion. */
			int err = heater_hw_force_off();
			heat.duty = 0;
			if (err) {
				(void)finish_locked(TCAL_HEATED_STOP_HEATER_ERROR);
			} else {
				sensor_tcal_heated_accum_finish();
				heat.sampling = false;
				heat.state = HEAT_FINALIZING;
				heat.reason = TCAL_HEATED_STOP_COMPLETE;
			}
		}
	} else {
		heat.stable_since = -1;
	}
	sensor_tcal_heated_unlock();
}

static bool qualified_locked(void)
{
	unsigned int count = 0;
	bool start_covered = false;
	bool target_covered = false;
	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (stage[i].temp == 0.0f) {
			continue;
		}
		float temp = stage[i].temp;
		if (!v_finite(&temp, 1) || !v_finite(stage[i].bias, 3) ||
		    temp < CONFIG_SENSOR_POLY_TEMP_MIN || temp >= CONFIG_SENSOR_POLY_TEMP_MAX || TEMP_TO_IDX(temp) != i) {
			return false;
		}
		count++;
		float band = 1.0f / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE;
		start_covered |= fabsf(temp - heat.start_temp) <= band;
		target_covered |= fabsf(temp - heat.target) <= band;
	}
	if (heat.reason == TCAL_HEATED_STOP_USER) {
		return count > 0 && count == heat.count && owner_epoch == heat.epoch;
	}
	return count >= MLS_MIN_POINTS_FOR_FIT && count == heat.count && start_covered && target_covered;
}

void sensor_tcal_heated_finalize(void)
{
	/* Safety polling uses existing worker, never stage mutations or sensor waits. */
	sensor_tcal_heated_lock();
	if (heat.state == HEAT_RESERVED || heat.state == HEAT_RUNNING) {
		struct sensor_temperature_observation observation;
		enum tcal_heated_stop_reason reason = check_locked(&observation);
		if (reason != TCAL_HEATED_STOP_NONE) {
			(void)finish_locked(reason);
		}
	}
	heat_safety_refresh_locked();
	bool finalize = heat.state == HEAT_FINALIZING;
	uint32_t epoch = heat.epoch;
	sensor_tcal_heated_unlock();
	if (!finalize) {
		return;
	}
	/* Storage prevents pointer-based warm flush from observing a partial merge. */
	sys_warm_transaction_begin();
	sensor_tcal_heated_lock();
	bool applied = false;
	if (heat.state == HEAT_FINALIZING && heat.epoch == epoch && !heat.resetting && !heat.terminal) {
		/* Hardware is intentionally disarmed, but fault latches, fresh
		 * observations and lifecycle/timeout limits still apply after waiting
		 * for storage. Disarm does not itself mark the lease expired. */
		struct sensor_temperature_observation observation;
		enum tcal_heated_stop_reason reason = check_locked(&observation);
		if (reason != TCAL_HEATED_STOP_NONE || !heat.ready) {
			(void)finish_locked(reason != TCAL_HEATED_STOP_NONE ? reason : TCAL_HEATED_STOP_SENSOR_STOP);
		} else if (!qualified_locked()) {
			(void)finish_locked(TCAL_HEATED_STOP_INSUFFICIENT_COVERAGE);
		} else {
			sensor_tcal_lock();
			if (sensor_tcal_model_generation() == heat.base_generation) {
				for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
					if (stage[i].temp != 0.0f) {
						retained->tempCalPoints[i] = stage[i];
					}
				}
				retained->tempCalState.count = 0;
				for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
					retained->tempCalState.count += retained->tempCalPoints[i].temp != 0.0f;
				}
				/* Availability follows the merged table, not session coverage. */
				retained->tempCalState.valid = retained->tempCalState.count >= 1;
				retained->tempCalState.degree = 0;
				memset(retained->tempCalCoeffs, 0, sizeof(retained->tempCalCoeffs));
				retained->gyroTemp = heat.accepted_max;
				sensor_tcal_refresh_model();
				applied = true;
			}
			sensor_tcal_unlock();
			/* This RAM publication is completion's linearization point. */
			heat.applied = applied;
			if (applied) {
				heat.warm_pending = true;
				heat.written_mask = 0;
				sensor_tcal_lock();
				heat.published_generation = sensor_tcal_model_generation();
				sensor_tcal_unlock();
				sensor_calibration_stage(heat.feedback, LED_HEATED_ACTIVE);
				/* Bound before marking: capacity flush may complete immediately. */
				sys_warm_feedback_arm(heat.feedback.session);
			}
			if (!applied) {
				heat.reason = TCAL_HEATED_STOP_INSUFFICIENT_COVERAGE;
				heat_feedback_finish_locked(LED_FAILED);
			}
			heat.state = HEAT_FINISHED;
			heat.epoch++;
			sensor_calibration_heated_release_locked();
		}
	}
	sensor_tcal_heated_unlock();
	if (applied) {
		/* Mark can flush an overflowing dirty table: no request/model lock held. */
		sys_warm_transaction_mark(MAIN_GYRO_TEMP_ID, &retained->gyroTemp, sizeof(retained->gyroTemp));
		sys_warm_transaction_mark(MAIN_GYRO_TCAL_STATE_ID, &retained->tempCalState, sizeof(retained->tempCalState));
		sys_warm_transaction_mark(MAIN_GYRO_TCAL_POINTS_ID, retained->tempCalPoints, sizeof(retained->tempCalPoints));
		sys_warm_transaction_mark(MAIN_GYRO_TCAL_COEFFS_ID, retained->tempCalCoeffs, sizeof(retained->tempCalCoeffs));
	}
	sys_warm_transaction_end(applied);
	if (applied) {
		LOG_INF("Heated T-Cal model applied in RAM; warm persistence pending (coverage, not accuracy proof)");
	}
}

void sensor_tcal_heated_report(void)
{
	static const char *const states[] = {"idle", "reserved", "running", "finalizing", "finished", "off-failed"};
	static const char *const reasons[] = {"none", "sensor-stop", "power-down", "user", "timeout", "stale-temperature",
		"overtemperature", "rise-too-fast", "heater-error", "insufficient-coverage", "complete"};
	sensor_tcal_heated_lock();
	enum heat_state state = heat.state;
	enum tcal_heated_stop_reason reason = heat.reason;
	bool busy = sensor_tcal_heated_busy_locked();
	bool applied = heat.applied;
	bool sampling = heat.sampling;
	float target = heat.target, setpoint = heat.setpoint;
	float minimum = heat.accepted_min, maximum = heat.accepted_max;
	uint16_t duty = heat.duty, count = heat.count;
	struct sensor_temperature_observation observation;
	bool fresh = sensor_get_imu_temperature_observation(&observation, HEAT_FRESH_MS) == 0;
	struct heater_hw_status hardware;
	heater_hw_get_status(&hardware);
	sensor_tcal_heated_unlock();
	printk("Heated T-Cal: active=%s state=%s reason=%s sampling=%s\n", busy ? "yes" : "no", states[state], reasons[reason], sampling ? "yes" : "no");
	printk("  target=%.2fC setpoint=%.2fC duty=%u/10000 temperature=%s", (double)target, (double)setpoint, duty, fresh ? "fresh" : "unavailable/stale");
	if (fresh) {
		printk(" raw=%.2fC filtered=%.2fC age=%lldms", (double)observation.raw_c, (double)observation.filtered_c, k_uptime_get() - observation.sampled_at_ms);
	}
	printk("\n  staged=%u", count);
	if (count) {
		printk(" accepted=%.2f..%.2fC", (double)minimum, (double)maximum);
	}
	printk(" model-applied=%s (RAM; asynchronous warm persistence)\n", applied ? "yes" : "no");
	printk("  hardware armed=%s expired=%s fault=%s error=%d; motion pauses ramp/sampling, may maintain heat\n",
	       hardware.armed ? "yes" : "no", hardware.expired ? "yes" : "no", hardware.faulted ? "yes" : "no", hardware.last_error);
}
#endif
