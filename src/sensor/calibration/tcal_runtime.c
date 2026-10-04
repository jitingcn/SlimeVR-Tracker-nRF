/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include "connection/tracker_events.h"
#include "sensor/sensor.h"
#include "system/system.h"
#include "system/power.h"
#include "system/uptime.h"
#include "system/watchdog.h"
#include "util.h"

#include <math.h>
#include <errno.h>
#include <string.h>
#include <zephyr/sys/atomic.h>

#include "bias_collect.h"
#include "calibration.h"
#include "tcal_mls_lut.h"
#include "tcal_runtime.h"
#if CONFIG_SENSOR_TCAL_HEATED
#include "tcal_heated.h"
#endif

#if CONFIG_SENSOR_USE_TCAL

LOG_MODULE_REGISTER(cal_tcal_runtime, LOG_LEVEL_INF);

#define TCAL_ACCUM_FLUSH_INTERVAL_MS 25000
#define TCAL_ACCUM_MIN_SAMPLES 2000
#define TCAL_ACCUM_TEMP_DRIFT_MAX 0.53f
#define TCAL_ACCUM_GYRO_RANGE_THRESHOLD 5.0f /* dps, windowed-mean range method */
#define TCAL_ACCUM_GYRO_MOTION_WINDOW_MS 250 /* ms - smooth raw gyro noise before range check */
/* Local, offset-invariant block rejection, not a zero-motion detector.
 * The floor is an engineering tolerance, not sensor hardware qualification. */
#define TCAL_ACCUM_ROBUST_BLOCKS 5
#define TCAL_ACCUM_ROBUST_MIN_BLOCKS 3
#define TCAL_ACCUM_ROBUST_FLOOR_DPS 0.05f
#define TCAL_ACCUM_ROBUST_MAD_SCALE 8.8956f /* six Gaussian-equivalent sigmas */
#define TCAL_ACCUM_RETAIN_PERCENT 80
#if CONFIG_SENSOR_TCAL_HEATED
/* Ignore brief boundary chatter; discarded candidate samples never enter
 * either bin. Confirmation is time-based, independent of the gyro ODR. */
#define TCAL_ACCUM_SLOT_CONFIRM_MS 250
#endif

#define TCAL_SAVE_SIGNIFICANCE_THRESHOLD 0.002f

static bool runtime_cal_enabled = false;
static int64_t runtime_cal_last_time = 0;
static int64_t runtime_cal_rest_start = 0;
static bool runtime_cal_rest_tracking = false;
float runtime_cal_last_temp = NAN;

static atomic_t tcal_auto_calibration_enabled;
static bool tcal_compensation_enabled = true; /* until init_from_retained */
static int tcal_compensation_storage_error;
/* Hot-path cache: avoid count/enable re-check every gyro sample. */
static bool tcal_curve_apply_ready;
static uint32_t reference_generation;
static bool measured_bias_reset_pending;
/* Boot/runtime corrections are session-only, not NVS calibration points.
 * Keep one immutable user receipt until the sensor actually uses that offset. */
static struct {
	struct led_token token;
	uint32_t reference_generation;
	uint32_t calibration_generation;
} tcal_feedback;

static void tcal_feedback_finish_locked(enum led_semantic semantic)
{
	if (!tcal_feedback.token.session) {
		return;
	}
	sensor_calibration_result(tcal_feedback.token, semantic);
	memset(&tcal_feedback, 0, sizeof(tcal_feedback));
}

void sensor_tcal_feedback_cancel(void)
{
	sensor_tcal_lock();
	tcal_feedback_finish_locked(LED_CANCELLED);
	sensor_tcal_unlock();
}

/* Sensor owner, T-Cal lock held, after the actual gyro subtraction. A fallback
 * to static bias did not apply the promised correction and is not success. */
void sensor_tcal_feedback_applied(uint32_t generation, bool offset_applied)
{
	if (!tcal_feedback.token.session) {
		return;
	}
	if (!sensor_calibration_generation_valid(tcal_feedback.calibration_generation) ||
	    generation != tcal_feedback.reference_generation) {
		tcal_feedback_finish_locked(LED_CANCELLED);
		return;
	}
	tcal_feedback_finish_locked(offset_applied ? LED_SUCCESS : LED_PARTIAL);
}

static void tcal_result(struct led_token feedback, uint32_t generation,
	enum led_semantic semantic)
{
	if (!feedback.session) {
		return;
	}
	bool valid = sensor_calibration_generation_valid(generation);
	sensor_calibration_result(feedback, valid ? semantic : LED_CANCELLED);
}

uint32_t sensor_tcal_reference_generation(void)
{
	return reference_generation;
}

bool sensor_tcal_take_bias_reset(void)
{
	bool reset = measured_bias_reset_pending;
	measured_bias_reset_pending = false;
	return reset;
}

void sensor_tcal_mark_measured_bias(void)
{
	tcal_feedback_finish_locked(LED_CANCELLED);
	measured_bias_reset_pending = true;
	reference_generation++;
	retained->fusion_id = 0;
}

void sensor_tcal_clear_doffset(void)
{
	sensor_tcal_lock();
	tcal_feedback_finish_locked(LED_CANCELLED);
	if (tcal_curve_apply_ready && retained->bootCalState.doffset_valid) {
		reference_generation++;
		retained->fusion_id = 0;
	}
	retained->bootCalState.doffset_valid = false;
	memset(retained->bootCalState.doffset, 0, sizeof(retained->bootCalState.doffset));
	measured_bias_reset_pending = false;
	sensor_tcal_unlock();
}

void sensor_tcal_refresh_model(void)
{
	sensor_tcal_clear_doffset();
	sensor_tcal_model_changed();
	if (tcal_compensation_enabled &&
	    (tcal_curve_apply_ready || retained->tempCalState.count >= MLS_MIN_POINTS_FOR_FIT)) {
		reference_generation++;
		retained->fusion_id = 0;
	}
	sensor_tcal_refresh_apply_cache();
	float temp = sensor_get_current_imu_temperature();
	if (v_finite(&temp, 1)) {
		sensor_tcal_build_lut_priority(temp);
	}
}

tcal_temp_direction_t tcal_current_direction = TCAL_DIR_UNKNOWN;
float tcal_direction_ref_temp = NAN;

static struct {
	double gyro_sum[3];
	double temp_sum;
	int sample_count;
	int observed_count;
	/* Gyro motion gate uses the range of short-window MEANS, not raw samples:
	 * a large but stable zero-rate offset plus high-frequency noise (e.g.
	 * >10 dps spread while stationary) must not abort accumulation. */
	double gyro_win_sum[3];
	double temp_win_sum;
	/* Only full ODR-sized blocks enter this provisional group. Column 3 is
	 * their paired temperature mean; no raw FIFO or learned curve needed. */
	float block_mean[TCAL_ACCUM_ROBUST_BLOCKS][4];
	uint8_t block_count;
	int gyro_win_count;
	int gyro_win_samples;
	bool gyro_win_tracked;
	float min_g[3];
	float max_g[3];
	float min_a[3];
	float max_a[3];
	bool accel_tracking;
	uint8_t accel_peek_div;
	float temp_min;
	float temp_max;
	bool active;
	int64_t start_time;
#if CONFIG_SENSOR_TCAL_HEATED
	uint32_t reset_generation;
	int slot;
	int pending_slot;
	int64_t pending_since;
	int64_t last_sample_time;
#endif
} tcal_accum;

#if CONFIG_SENSOR_TCAL_HEATED
/* A consumed reset must still invalidate an average blocked on storage.
 * Only the sensor owner advances seen; publishers compare the saved epoch. */
static atomic_t tcal_accum_reset_generation;
static uint32_t tcal_accum_seen_reset_generation;
#else
static int64_t tcal_accum_last_commit_time = 0;
#endif

static void tcal_accum_flush(bool heated);
static void tcal_save_point(int idx, const float bias[3], float measured_temp, uint32_t reset_generation);
static int sensor_boot_bias_collect(float *dest_bias, float *avg_temp);
static int sensor_runtime_bias_collect(float *dest_bias, float *avg_temp);
static int sensor_tcal_calculate_doffset(const float measured_bias[3], float temp, uint16_t operation,
	struct led_token feedback, uint32_t generation);

void sensor_tcal_refresh_apply_cache(void)
{
	sensor_tcal_lock();
	tcal_curve_apply_ready =
		tcal_compensation_enabled && (retained->tempCalState.count >= MLS_MIN_POINTS_FOR_FIT);
	sensor_tcal_unlock();
}

bool sensor_tcal_curve_apply_ready(void)
{
	sensor_tcal_lock();
	bool ready = tcal_curve_apply_ready;
	sensor_tcal_unlock();
	return ready;
}

sensor_tcal_apply_mode_t sensor_tcal_get_apply_mode(void)
{
	sensor_tcal_lock();
	sensor_tcal_apply_mode_t mode = !tcal_compensation_enabled ? SENSOR_TCAL_APPLY_DISABLED :
		tcal_curve_apply_ready ? SENSOR_TCAL_APPLY_CURVE : SENSOR_TCAL_APPLY_ZRO_FALLBACK;
	sensor_tcal_unlock();
	return mode;
}

sensor_tcal_apply_mode_t sensor_tcal_snapshot(bool *enabled, struct TempCalPoint *points,
					    uint16_t *count)
{
	sensor_tcal_lock();
	*enabled = tcal_compensation_enabled;
	sensor_tcal_apply_mode_t mode = !tcal_compensation_enabled ? SENSOR_TCAL_APPLY_DISABLED :
		tcal_curve_apply_ready ? SENSOR_TCAL_APPLY_CURVE : SENSOR_TCAL_APPLY_ZRO_FALLBACK;
	*count = 0;
	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (retained->tempCalPoints[i].temp != 0.0f) {
			points[(*count)++] = retained->tempCalPoints[i];
		}
	}
	sensor_tcal_unlock();
	return mode;
}

const char *sensor_tcal_get_apply_mode_name(void)
{
	switch (sensor_tcal_get_apply_mode()) {
	case SENSOR_TCAL_APPLY_CURVE:
		return "curve";
	case SENSOR_TCAL_APPLY_ZRO_FALLBACK:
		return "zro-fallback";
	case SENSOR_TCAL_APPLY_DISABLED:
	default:
		return "disabled-zro";
	}
}

void sensor_tcal_runtime_init_from_retained(void)
{
	/* Heal NaN/±inf points persisted by older builds: one bad point breaks
	 * every MLS/LUT lookup (NaN weights never fall below the min-weight
	 * gate), so clear it and recompute the point count. */
	uint8_t healed_points = 0;
	uint8_t valid_points = 0;
	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (!v_finite(&retained->tempCalPoints[i].temp, 1)
		    || !v_finite(retained->tempCalPoints[i].bias, 3)) {
			LOG_WRN("T-Cal: clearing non-finite calibration point at slot %d", i);
			memset(&retained->tempCalPoints[i], 0, sizeof(retained->tempCalPoints[i]));
			healed_points++;
		}
		if (retained->tempCalPoints[i].temp != 0.0f) {
			valid_points++;
		}
	}
	/* POINTS, COEFFS and STATE are separate storage writes. The table is
	 * authoritative even when power failed with an entirely finite table. */
	bool repaired = healed_points > 0 || retained->tempCalState.count != valid_points
		|| retained->tempCalState.valid != (valid_points != 0)
		|| retained->tempCalState.degree != 0;
	retained->tempCalState.count = valid_points;
	retained->tempCalState.valid = valid_points != 0;
	retained->tempCalState.degree = 0;
	memset(retained->tempCalCoeffs, 0, sizeof(retained->tempCalCoeffs));
	if (repaired) {
		LOG_WRN("T-Cal: reconstructed state from %u surviving points", valid_points);
	}
	retained_update();

	tcal_compensation_enabled = retained->tcal_enabled;
	sensor_calibration_reset_gyro_reference();
	sensor_tcal_refresh_apply_cache();
	LOG_INF(
		"T-Cal compensation: %s | apply=%s (points=%u, need>=%d)",
		tcal_compensation_enabled ? "enabled" : "disabled",
		sensor_tcal_get_apply_mode_name(),
		retained->tempCalState.count,
		MLS_MIN_POINTS_FOR_FIT
	);
}

void update_tcal_state(void)
{
	/* Callers publish the complete model under the T-Cal lock before storage. */
	// Polynomial coefficients are no longer used; keep persisted storage zeroed
	memset(retained->tempCalCoeffs, 0, sizeof(retained->tempCalCoeffs));
	retained->tempCalState.degree = 0;

	// Mark MLS availability based on point count
	retained->tempCalState.valid = (retained->tempCalState.count >= 1);

	if (retained->tempCalState.valid) {
		LOG_INF("T-Cal: MLS state refreshed with %u points", retained->tempCalState.count);
		printk("T-Cal: MLS data refreshed successfully.\n");

		// Start incremental LUT build for O(1) runtime lookup
		// Priority zone (current temp ±3°C) is built immediately
		// Remaining entries are built in background by calibration_thread
		float current_temp = sensor_get_current_imu_temperature();
		if (!isnan(current_temp)) {
			sensor_tcal_build_lut_priority(current_temp);
		}
	} else {
		LOG_INF("T-Cal: No points available");
		printk("T-Cal: No points available.\n");
	}

	/* Warm: retained now; NVS on sys_flush_warm (WoM / reboot / system-off).
	 * User-initiated callers should follow with sys_flush_warm().
	 */
	sys_write_warm(
		MAIN_GYRO_TEMP_ID,
		&retained->gyroTemp,
		&retained->gyroTemp,
		sizeof(retained->gyroTemp)
	);
	sys_write_warm(
		MAIN_GYRO_TCAL_STATE_ID,
		&retained->tempCalState,
		&retained->tempCalState,
		sizeof(retained->tempCalState)
	);
	sys_write_warm(
		MAIN_GYRO_TCAL_POINTS_ID,
		retained->tempCalPoints,
		retained->tempCalPoints,
		sizeof(retained->tempCalPoints)
	);
	sys_write_warm(
		MAIN_GYRO_TCAL_COEFFS_ID,
		retained->tempCalCoeffs,
		retained->tempCalCoeffs,
		sizeof(retained->tempCalCoeffs)
	);

}

int sensor_tcal_set_auto_calibration(bool enabled)
{
#if CONFIG_SENSOR_TCAL_HEATED
	if (sensor_calibration_maintenance_begin() != 0) {
		printk("T-Cal auto-calibration unchanged: calibration busy.\n");
		return sensor_operation_result(LED_OWNER_TCAL, -EBUSY, false);
	}
#endif
	atomic_set(&tcal_auto_calibration_enabled, enabled);
	if (!enabled) {
		tcal_accum_request_reset();
	}
#if CONFIG_SENSOR_TCAL_HEATED
	sensor_calibration_maintenance_end();
#endif
	/* Publish before cancelling reversible sleep; never hold the calibration
	 * maintenance lock while acquiring the power policy mutex. A physical
	 * shutdown that already committed cannot be withdrawn. */
	if (enabled) {
		sys_cancel_WOM();
	}
	LOG_INF("T-Cal Auto-calibration %s", enabled ? "enabled" : "disabled");
	return sensor_operation_result(LED_OWNER_TCAL, 0, true);
}

#if !CONFIG_SENSOR_TCAL_HEATED
void sensor_tcal_feedback_persisted(uint32_t identity, uint8_t written_mask, int result)
{
	/* Non-heated user operations synchronously observe their own flush result.
	 * Background points intentionally have no local success notification. */
	(void)identity;
	(void)written_mask;
	(void)result;
}
#endif

// Get auto-calibration enabled status
bool sensor_tcal_get_auto_calibration(void)
{
	return atomic_get(&tcal_auto_calibration_enabled) != 0;
}

// =============================================================================
// Continuous Accumulator Implementation
// =============================================================================

void tcal_accum_reset(void)
{
	memset(&tcal_accum, 0, sizeof(tcal_accum));
	tcal_accum.temp_min = INFINITY;
	tcal_accum.temp_max = -INFINITY;
}

void tcal_accum_request_reset(void)
{
#if CONFIG_SENSOR_TCAL_HEATED
	atomic_inc(&tcal_accum_reset_generation);
#else
	tcal_accum_reset();
#endif
}

void tcal_accum_apply_reset(void)
{
#if CONFIG_SENSOR_TCAL_HEATED
	uint32_t generation = (uint32_t)atomic_get(&tcal_accum_reset_generation);
	if (generation != tcal_accum_seen_reset_generation) {
		tcal_accum_reset();
		tcal_accum_seen_reset_generation = generation;
	}
#endif
}

/**
 * Save a calibration point with hysteresis-aware blending.
 *
 * Slot = TEMP_TO_IDX(measured_temp). Store the measured average temperature
 * (not bucket center): bucket only addresses collisions; we do not assume
 * samples are uniform inside the bin.
 */
static void tcal_save_point(int idx, const float bias[3], float measured_temp, uint32_t reset_generation)
{
	if (idx < 0 || idx >= TCAL_BUFFER_SIZE) {
		LOG_WRN("T-Cal: Index %d out of range, skipping", idx);
		return;
	}
	if (!v_finite(&measured_temp, 1)) {
		LOG_WRN("T-Cal: Invalid measured temperature, skipping save");
		return;
	}
	/* One NaN point breaks every MLS/LUT lookup (NaN weights pass the
	 * min-weight gate), so never commit a non-finite bias. */
	if (!v_finite(bias, 3)) {
		LOG_WRN("T-Cal: Non-finite bias, skipping save");
		return;
	}

#if CONFIG_SENSOR_TCAL_HEATED
	/* Storage precedes the session gate, which precedes model ownership. */
	sys_warm_transaction_begin();
	sensor_tcal_heated_lock();
	if (sensor_tcal_heated_busy_locked() || sensor_tcal_heated_resetting_locked()
	    || sensor_calibration_maintenance_active_locked()
	    || reset_generation != (uint32_t)atomic_get(&tcal_accum_reset_generation)) {
		sensor_tcal_heated_unlock();
		sys_warm_transaction_end(false);
		return;
	}
#else
	(void)reset_generation;
#endif
	sensor_tcal_lock();
	/* Update direction from measured temps (same-slot revisits may be ~equal). */
	if (!isnan(tcal_direction_ref_temp)) {
		float delta = measured_temp - tcal_direction_ref_temp;
		if (delta > 0.2f) {
			tcal_current_direction = TCAL_DIR_RISING;
		} else if (delta < -0.2f) {
			tcal_current_direction = TCAL_DIR_FALLING;
		}
	}
	tcal_direction_ref_temp = measured_temp;

	float final_bias[3];
	memcpy(final_bias, bias, sizeof(final_bias));

	/* Rising preferred; falling still blends in (weak α), never rejected. */
	bool is_new_point = (retained->tempCalPoints[idx].temp == 0.0f);
	if (!is_new_point) {
		float ema_alpha;
		switch (tcal_current_direction) {
		case TCAL_DIR_RISING:
			ema_alpha = TCAL_HYSTERESIS_EMA_RISING;
			break;
		case TCAL_DIR_FALLING:
			ema_alpha = TCAL_HYSTERESIS_EMA_FALLING;
			break;
		default:
			ema_alpha = TCAL_HYSTERESIS_EMA_UNKNOWN;
			break;
		}
		LOG_INF(
			"T-Cal: Blending at idx %d (dir: %s, alpha: %.2f)",
			idx,
			tcal_current_direction == TCAL_DIR_RISING    ? "rising"
			: tcal_current_direction == TCAL_DIR_FALLING ? "falling"
														 : "unknown",
			(double)ema_alpha
		);
		for (int axis = 0; axis < 3; axis++) {
			final_bias[axis]
				= ema_alpha * final_bias[axis] + (1.0f - ema_alpha) * retained->tempCalPoints[idx].bias[axis];
		}
		LOG_INF(
			"T-Cal: Blended bias: %.5f %.5f %.5f",
			(double)final_bias[0],
			(double)final_bias[1],
			(double)final_bias[2]
		);
	} else {
		retained->tempCalState.count++;
	}

	float max_delta = 0.0f;
	if (!is_new_point) {
		for (int axis = 0; axis < 3; axis++) {
			float d = fabsf(final_bias[axis] - retained->tempCalPoints[idx].bias[axis]);
			if (d > max_delta) {
				max_delta = d;
			}
		}
	}

#if CONFIG_SENSOR_TCAL_HEATED
	retained->gyroTemp = measured_temp;
#endif
	if (!is_new_point && retained->tempCalPoints[idx].temp == measured_temp &&
	    memcmp(retained->tempCalPoints[idx].bias, final_bias, sizeof(final_bias)) == 0) {
		sensor_tcal_unlock();
#if CONFIG_SENSOR_TCAL_HEATED
		sensor_tcal_heated_unlock();
		sys_warm_transaction_end(true);
#endif
		return;
	}
	retained->tempCalPoints[idx].temp = measured_temp;
	memcpy(retained->tempCalPoints[idx].bias, final_bias, sizeof(float) * 3);
	retained->tempCalState.valid = false;
#if CONFIG_SENSOR_TCAL_HEATED
	bool mark_warm = is_new_point || max_delta >= TCAL_SAVE_SIGNIFICANCE_THRESHOLD;
	if (mark_warm) {
		memset(retained->tempCalCoeffs, 0, sizeof(retained->tempCalCoeffs));
		retained->tempCalState.degree = 0;
		retained->tempCalState.valid = (retained->tempCalState.count >= 1);
	}
#endif
	sensor_tcal_refresh_model();
	sensor_tcal_unlock();
#if CONFIG_SENSOR_TCAL_HEATED
	sensor_tcal_heated_unlock();
	/* Dirty-table overflow may flush storage. Release the request/model
	 * locks first, but keep storage ownership through the warm marks. */
	if (mark_warm) {
		sys_warm_transaction_mark(MAIN_GYRO_TEMP_ID, &retained->gyroTemp, sizeof(retained->gyroTemp));
		sys_warm_transaction_mark(MAIN_GYRO_TCAL_STATE_ID, &retained->tempCalState,
		                          sizeof(retained->tempCalState));
		sys_warm_transaction_mark(MAIN_GYRO_TCAL_POINTS_ID, retained->tempCalPoints,
		                          sizeof(retained->tempCalPoints));
		sys_warm_transaction_mark(MAIN_GYRO_TCAL_COEFFS_ID, retained->tempCalCoeffs,
		                          sizeof(retained->tempCalCoeffs));
	}
	sys_warm_transaction_end(true);
#endif

	LOG_INF(
		"T-Cal: Committed point at idx %d (%.2fC): [%.5f, %.5f, %.5f] (delta: %.4f dps)",
		idx,
		(double)measured_temp,
		(double)final_bias[0],
		(double)final_bias[1],
		(double)final_bias[2],
		(double)max_delta
	);

#if !CONFIG_SENSOR_TCAL_HEATED
	if (is_new_point || max_delta >= TCAL_SAVE_SIGNIFICANCE_THRESHOLD) {
		update_tcal_state();
	} else {
		LOG_DBG(
			"T-Cal: Change below threshold (%.4f < %.4f), skipping warm NVS mark",
			(double)max_delta,
			(double)TCAL_SAVE_SIGNIFICANCE_THRESHOLD
		);
		/* Keep CRC valid for soft-reset without dirtying NVS for churn. */
		retained_update();
	}
#endif
}


/* Share the peak-to-peak gate across initialization, periodic sampling and
 * heated final acceptance. Ordinary sampling retains its optional accel. */
static bool tcal_accum_check_accel(bool heated)
{
	float a[3];
#if CONFIG_SENSOR_TCAL_HEATED
	bool have_accel = heated ? sensor_peek_accel_fresh(a, 2000) : sensor_peek_accel(a);
#else
	bool have_accel = sensor_peek_accel(a);
#endif
	if (heated && (!have_accel || !v_finite(a, 3))) {
		tcal_accum_reset();
		return false;
	}
	if (!have_accel) {
		return true;
	}
	if (!tcal_accum.accel_tracking) {
		memcpy(tcal_accum.min_a, a, sizeof(tcal_accum.min_a));
		memcpy(tcal_accum.max_a, a, sizeof(tcal_accum.max_a));
		tcal_accum.accel_tracking = true;
		return true;
	}
	for (int j = 0; j < 3; j++) {
		if (a[j] < tcal_accum.min_a[j]) {
			tcal_accum.min_a[j] = a[j];
		}
		if (a[j] > tcal_accum.max_a[j]) {
			tcal_accum.max_a[j] = a[j];
		}
		if (tcal_accum.max_a[j] - tcal_accum.min_a[j] > TCAL_ACCUM_ACCEL_MOTION_THRESHOLD) {
			LOG_DBG(
				"T-Cal: Accel motion in accumulator, axis %d (range: %.4f G), resetting",
				j,
				(double)(tcal_accum.max_a[j] - tcal_accum.min_a[j])
			);
			tcal_accum_reset();
			return false;
		}
	}
	return true;
}

static float tcal_accum_median(float values[TCAL_ACCUM_ROBUST_BLOCKS], int count)
{
	for (int i = 1; i < count; i++) {
		float value = values[i];
		int j = i;
		while (j > 0 && values[j - 1] > value) {
			values[j] = values[j - 1];
			j--;
		}
		values[j] = value;
	}
	return count & 1 ? values[count / 2] : (values[count / 2 - 1] + values[count / 2]) * 0.5f;
}

/* Classify before committing anything. A common vector mask keeps temperature
 * and all three gyro axes on exactly the same retained sample population. */
static bool tcal_accum_commit_blocks(void)
{
	int count = tcal_accum.block_count;
	tcal_accum.block_count = 0;
	if (count < TCAL_ACCUM_ROBUST_MIN_BLOCKS) {
		return true; /* An unsupported tail is not evidence. */
	}
	uint8_t accepted = (1U << count) - 1U;
	for (int axis = 0; axis < 3; axis++) {
		float values[TCAL_ACCUM_ROBUST_BLOCKS];
		for (int i = 0; i < count; i++) {
			values[i] = tcal_accum.block_mean[i][axis];
		}
		float center = tcal_accum_median(values, count);
		for (int i = 0; i < count; i++) {
			values[i] = fabsf(tcal_accum.block_mean[i][axis] - center);
		}
		float limit = MAX(TCAL_ACCUM_ROBUST_FLOOR_DPS,
		                  TCAL_ACCUM_ROBUST_MAD_SCALE * tcal_accum_median(values, count));
		for (int i = 0; i < count; i++) {
			if (fabsf(tcal_accum.block_mean[i][axis] - center) > limit) {
				accepted &= ~(1U << i);
			}
		}
	}
	int support = 0;
	for (int i = 0; i < count; i++) {
		support += !!(accepted & (1U << i));
	}
	if (support < TCAL_ACCUM_ROBUST_MIN_BLOCKS) {
		return true;
	}
	for (int i = 0; i < count; i++) {
		if (!(accepted & (1U << i))) {
			continue;
		}
		for (int axis = 0; axis < 3; axis++) {
			float mean = tcal_accum.block_mean[i][axis];
			if (!tcal_accum.gyro_win_tracked) {
				tcal_accum.min_g[axis] = mean;
				tcal_accum.max_g[axis] = mean;
			} else {
				tcal_accum.min_g[axis] = fminf(tcal_accum.min_g[axis], mean);
				tcal_accum.max_g[axis] = fmaxf(tcal_accum.max_g[axis], mean);
			}
			if (tcal_accum.max_g[axis] - tcal_accum.min_g[axis] >
			    TCAL_ACCUM_GYRO_RANGE_THRESHOLD) {
				/* Sustained changes remain motion, not disposable outliers. */
				tcal_accum_reset();
				return false;
			}
		}
		tcal_accum.gyro_win_tracked = true;
	}
	for (int i = 0; i < count; i++) {
		if (!(accepted & (1U << i))) {
			continue;
		}
		for (int axis = 0; axis < 3; axis++) {
			tcal_accum.gyro_sum[axis] +=
				(double)tcal_accum.block_mean[i][axis] * tcal_accum.gyro_win_samples;
		}
		tcal_accum.temp_sum += (double)tcal_accum.block_mean[i][3] * tcal_accum.gyro_win_samples;
		tcal_accum.sample_count += tcal_accum.gyro_win_samples;
	}
	return true;
}

/**
 * Flush the accumulator: compute average bias/temperature, save to the
 * appropriate temperature bucket.
 */
static void tcal_accum_flush(bool heated)
{
	if (!tcal_accum.active || tcal_accum.observed_count < TCAL_ACCUM_MIN_SAMPLES ||
	    !tcal_accum_commit_blocks()) {
		return;
	}
	/* Never accept an unchecked partial raw block. Coverage includes those
	 * discarded samples and unsupported groups, not just classified blocks. */
	tcal_accum.gyro_win_count = 0;
	memset(tcal_accum.gyro_win_sum, 0, sizeof(tcal_accum.gyro_win_sum));
	tcal_accum.temp_win_sum = 0.0;
	if (tcal_accum.sample_count < TCAL_ACCUM_MIN_SAMPLES ||
	    (int64_t)tcal_accum.sample_count * 100 <
	    (int64_t)tcal_accum.observed_count * TCAL_ACCUM_RETAIN_PERCENT) {
		tcal_accum_reset();
		return;
	}
#if CONFIG_SENSOR_TCAL_HEATED
	if (heated && !tcal_accum_check_accel(true)) {
		return;
	}
#else
	(void)heated;
#endif

	float avg_bias[3];
	for (int axis = 0; axis < 3; axis++) {
		avg_bias[axis] = (float)(tcal_accum.gyro_sum[axis] / tcal_accum.sample_count);
	}

	float avg_temp = (float)(tcal_accum.temp_sum / tcal_accum.sample_count);

	if (!v_finite(&avg_temp, 1) || !v_finite(avg_bias, 3)) {
		LOG_WRN("T-Cal: Invalid accumulator average, discarding");
		tcal_accum_reset();
		return;
	}

	/* Quasi-steady gate: reject fast thermal ramps so buckets stay near-static. */
	int64_t end_time = k_uptime_get();
#if CONFIG_SENSOR_TCAL_HEATED
	if (heated) {
		/* Boundary confirmation time is not part of the accepted window. */
		end_time = tcal_accum.last_sample_time;
	}
#endif
	float elapsed_s = (float)(end_time - tcal_accum.start_time) / 1000.0f;
	if (elapsed_s < 0.001f) {
		elapsed_s = 0.001f;
	}
	float temp_span = tcal_accum.temp_max - tcal_accum.temp_min;
	float dtdt = temp_span / elapsed_s;
	if (dtdt > TCAL_WRITE_DTDT_MAX_C_PER_S) {
		LOG_INF(
			"T-Cal: Discarding flush — |dT/dt| %.3f C/s > %.3f (span %.2fC / %.1fs)",
			(double)dtdt,
			(double)TCAL_WRITE_DTDT_MAX_C_PER_S,
			(double)temp_span,
			(double)elapsed_s
		);
		tcal_accum_reset();
		return;
	}

	int idx = TEMP_TO_IDX(avg_temp);
	if (idx < 0 || idx >= TCAL_BUFFER_SIZE) {
		LOG_WRN("T-Cal: Average temperature %.2fC outside calibration range, discarding", (double)avg_temp);
		tcal_accum_reset();
		return;
	}

	LOG_INF(
		"T-Cal: Flushing accumulator: %d samples, avg temp %.2fC (span %.2fC, dT/dt %.3f), slot idx %d",
		tcal_accum.sample_count,
		(double)avg_temp,
		(double)temp_span,
		(double)dtdt,
		idx
	);

#if CONFIG_SENSOR_TCAL_HEATED
	if (heated) {
		/* Reset requests also invalidate already-computed heated windows. */
		if (tcal_accum.reset_generation == (uint32_t)atomic_get(&tcal_accum_reset_generation) &&
		    idx == tcal_accum.slot) {
			sensor_tcal_heated_accept_point(idx, avg_bias, avg_temp);
		}
		tcal_accum_reset();
		return;
	}
	tcal_save_point(idx, avg_bias, avg_temp, tcal_accum.reset_generation);
#else
	/* Preserve the existing ordinary feature-off retained update ordering. */
	retained->gyroTemp = avg_temp;
	retained_update();
	tcal_save_point(idx, avg_bias, avg_temp, 0);
	tcal_accum_last_commit_time = k_uptime_get();
#endif

	tcal_accum_reset();
}

/**
 * Feed a gyro sample into the continuous accumulator.
 * Called from sensor_calibration_process_gyro for every raw gyro sample.
 *
 * The accumulator collects data continuously. Periodically it is flushed
 * (by time or by temperature drift) to save the averaged result.
 *
 * @param g Raw gyro reading (before bias subtraction)
 * @param temp Current IMU temperature
 */
static void tcal_accum_feed(const float g[3], float temp, bool heated)
{
	// Validate temperature
	if (!v_finite(&temp, 1) || temp < (float)CONFIG_SENSOR_POLY_TEMP_MIN || temp > (float)CONFIG_SENSOR_POLY_TEMP_MAX) {
		return;
	}

	/* Never let non-finite samples poison the accumulated sums. */
	if (!v_finite(g, 3)) {
		return;
	}
#if CONFIG_SENSOR_TCAL_HEATED
	if (heated) {
		int slot = TEMP_TO_IDX(temp);
		if (temp >= CONFIG_SENSOR_POLY_TEMP_MAX || slot < 0 || slot >= TCAL_BUFFER_SIZE) {
			tcal_accum_reset();
			return;
		}
		if (tcal_accum.active && slot != tcal_accum.slot) {
			if (tcal_accum.pending_slot != slot) {
				tcal_accum.pending_slot = slot;
				tcal_accum.pending_since = k_uptime_get();
				return;
			}
			if (k_uptime_get() - tcal_accum.pending_since < TCAL_ACCUM_SLOT_CONFIRM_MS) {
				return;
			}
			/* Finish the old slot BEFORE admitting the confirmed new sample.
			 * Short/rejected windows are discarded, not marked accepted. */
			tcal_accum_flush(true);
			tcal_accum_reset();
		} else if (tcal_accum.active) {
			tcal_accum.pending_slot = -1;
		}
		if (sensor_tcal_heated_slot_accepted(slot)) {
			return;
		}
	}
#endif

	// Initialize accumulator on first sample
	if (!tcal_accum.active) {
		tcal_accum_reset();
		tcal_accum.active = true;
#if CONFIG_SENSOR_TCAL_HEATED
		tcal_accum.reset_generation = tcal_accum_seen_reset_generation;
		tcal_accum.slot = TEMP_TO_IDX(temp);
		tcal_accum.pending_slot = -1;
#endif
		tcal_accum.start_time = k_uptime_get();
		tcal_accum.gyro_win_sum[0] = 0.0;
		tcal_accum.gyro_win_sum[1] = 0.0;
		tcal_accum.gyro_win_sum[2] = 0.0;
		tcal_accum.gyro_win_count = 0;
		tcal_accum.gyro_win_tracked = false;
		tcal_accum.gyro_win_samples = MAX(1, (int)(sensor_get_gyro_odr() * TCAL_ACCUM_GYRO_MOTION_WINDOW_MS / 1000.0f));
		tcal_accum.temp_min = temp;
		tcal_accum.temp_max = temp;
		tcal_accum.accel_peek_div = 0;
		if (!tcal_accum_check_accel(heated)) {
			return;
		}
	}

	/* Quarantine raw data until its full block has robust local support. */
	for (int j = 0; j < 3; j++) {
		tcal_accum.gyro_win_sum[j] += (double)g[j];
	}
	tcal_accum.temp_win_sum += (double)temp;
	tcal_accum.observed_count++;
	tcal_accum.gyro_win_count++;
	if (tcal_accum.gyro_win_count >= tcal_accum.gyro_win_samples) {
		int block = tcal_accum.block_count++;
		for (int j = 0; j < 3; j++) {
			tcal_accum.block_mean[block][j] =
				(float)(tcal_accum.gyro_win_sum[j] / tcal_accum.gyro_win_count);
			tcal_accum.gyro_win_sum[j] = 0.0;
		}
		tcal_accum.block_mean[block][3] = (float)(tcal_accum.temp_win_sum / tcal_accum.gyro_win_count);
		tcal_accum.temp_win_sum = 0.0;
		tcal_accum.gyro_win_count = 0;
		if (tcal_accum.block_count == TCAL_ACCUM_ROBUST_BLOCKS && !tcal_accum_commit_blocks()) {
			return;
		}
	}

	/*
	 * Accel peak–peak: peek every TCAL_ACCUM_ACCEL_PEEK_DIV samples only.
	 * Accel updates ~100Hz; checking every raw gyro sample was wasted work.
	 */
	if (++tcal_accum.accel_peek_div >= TCAL_ACCUM_ACCEL_PEEK_DIV) {
		tcal_accum.accel_peek_div = 0;
		if (!tcal_accum_check_accel(heated)) {
			return;
		}
	}

	/* Observed thermal extrema remain independent of statistical rejection. */
	if (temp < tcal_accum.temp_min) {
		tcal_accum.temp_min = temp;
	}
	if (temp > tcal_accum.temp_max) {
		tcal_accum.temp_max = temp;
	}
#if CONFIG_SENSOR_TCAL_HEATED
	if (heated) {
		tcal_accum.last_sample_time = k_uptime_get();
		/* The held starting band cannot exit until this first point exists.
		 * All later slots finish on confirmed exit or normal target dwell. */
		if (sensor_tcal_heated_start_window(temp) &&
		    k_uptime_get() - tcal_accum.start_time >= TCAL_ACCUM_FLUSH_INTERVAL_MS &&
		    tcal_accum.observed_count >= TCAL_ACCUM_MIN_SAMPLES) {
			tcal_accum_flush(true);
		}
		return;
	}
#endif

	// Check flush conditions
	int64_t elapsed = k_uptime_get() - tcal_accum.start_time;
	float temp_drift = tcal_accum.temp_max - tcal_accum.temp_min;

	// Condition 1: Temperature drifted beyond one bucket width — flush early
	// to avoid cross-bucket contamination, then start a new accumulation window
	if (temp_drift > TCAL_ACCUM_TEMP_DRIFT_MAX && tcal_accum.observed_count >= TCAL_ACCUM_MIN_SAMPLES) {
		LOG_INF("T-Cal: Temperature drift %.2fC exceeded threshold, early flush", (double)temp_drift);
		tcal_accum_flush(heated);
		return;
	}

	// Condition 2: Flush interval reached
	if (elapsed >= TCAL_ACCUM_FLUSH_INTERVAL_MS && tcal_accum.observed_count >= TCAL_ACCUM_MIN_SAMPLES) {
		tcal_accum_flush(heated);
		return;
	}
}

void sensor_tcal_feed_continuous_sample(const float g[3], float temp)
{
	tcal_accum_apply_reset();
	if (sensor_tcal_get_auto_calibration()) {
		tcal_accum_feed(g, temp, false);
	}
}

#if CONFIG_SENSOR_TCAL_HEATED
/* Sensor owner, request lock held. These paths stage RAM points only. */
void sensor_tcal_heated_accum_feed(const float g[3], float temp)
{
	tcal_accum_feed(g, temp, true);
}

void sensor_tcal_heated_accum_finish(void)
{
	tcal_accum_flush(true);
	tcal_accum_reset();
}
#endif

/**
 * Called when motion is detected — flush if enough data, then reset.
 */
void sensor_tcal_continuous_motion_detected(void)
{
#if CONFIG_SENSOR_TCAL_HEATED
	sensor_tcal_heated_lock();
	tcal_accum_apply_reset();
	if (sensor_tcal_heated_busy_locked() || sensor_tcal_heated_resetting_locked()) {
		tcal_accum_reset();
		sensor_tcal_heated_unlock();
		return;
	}
	sensor_tcal_heated_unlock();
#else
	tcal_accum_apply_reset();
#endif
	if (tcal_accum.active) {
		if (tcal_accum.observed_count >= TCAL_ACCUM_MIN_SAMPLES) {
			tcal_accum_flush(false);
		} else {
			tcal_accum_reset();
		}
	}
}

// Check and request auto calibration if conditions are met.
// With continuous accumulator sampling, this is only used as a fallback
// to trigger initial calibration when no T-Cal data exists at all.
void sensor_tcal_check_auto_calibration(float current_temp)
{
	static int64_t last_calibration_time = 0;

	int64_t now = k_uptime_get();

	if (!sensor_tcal_get_auto_calibration()) {
		return;
	}

	// Continuous accumulator handles the normal case.
	// This fallback only triggers initial manual calibration when there
	// are zero temperature calibration points (device first use).
	if (retained->tempCalState.count > 0) {
		return;
	}

	const int64_t calibration_cooldown_ms = BIAS_COLLECT_MAX_SAMPLE_TIME_MS + 10000;
	if ((now - last_calibration_time) < calibration_cooldown_ms) {
		return;
	}

	if (isnan(current_temp) || current_temp < -20.0f || current_temp > 60.0f) {
		return;
	}

	LOG_INF("T-Cal Auto: No calibration data exists, requesting initial calibration at %.2fC", (double)current_temp);

	int request_result = sensor_calibration_request(CAL_REQUEST_IMU, CAL_REQUEST_AUTO_SILENT);
	if (request_result == 0) {
		last_calibration_time = now;
	}
}

// =============================================================================
// Boot Calibration Implementation
// =============================================================================

/**
 * Assess temperature calibration quality for boot calibration
 * Checks if MLS has enough points with significant weight at the current temperature
 */
bool sensor_tcal_assess_quality(float current_temp, tcal_quality_t *quality)
{
	if (!quality) {
		return false;
	}

	// Initialize quality structure
	quality->curve_valid = retained->tempCalState.valid;
	quality->point_count = retained->tempCalState.count;
	quality->curve_error = 0.0f;
	quality->temp_min = INFINITY;
	quality->temp_max = -INFINITY;
	quality->temp_in_range = false;

	// Check minimum global point count
	if (quality->point_count < BOOT_CAL_MIN_CURVE_POINTS) {
		LOG_DBG("T-Cal: Insufficient points (%u < %d)", quality->point_count, BOOT_CAL_MIN_CURVE_POINTS);
		return false;
	}

	// Count points with significant weight at current temperature (MLS bandwidth check)
	// This ensures MLS will actually have usable data at this temperature
	float bandwidth_sq = MLS_BANDWIDTH * MLS_BANDWIDTH;
	int points_with_weight = 0;

	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (retained->tempCalPoints[i].temp != 0.0f) {
			float point_temp = retained->tempCalPoints[i].temp;

			// Track temperature range
			if (point_temp < quality->temp_min) {
				quality->temp_min = point_temp;
			}
			if (point_temp > quality->temp_max) {
				quality->temp_max = point_temp;
			}

			// Calculate weight at current temperature
			float d = point_temp - current_temp;
			float d_sq = d * d;
			float weight = 1.0f / (1.0f + d_sq / bandwidth_sq);

			// Count if weight is significant (>= MLS_MIN_WEIGHT)
			if (weight >= MLS_MIN_WEIGHT) {
				points_with_weight++;
			}
		}
	}

	// Check if current temperature is within or near the calibrated range
	if (current_temp >= quality->temp_min && current_temp <= quality->temp_max) {
		quality->temp_in_range = true;
	}

	// MLS needs at least 2 points with significant weight for linear fit
	if (points_with_weight < MLS_MIN_POINTS_FOR_FIT) {
		LOG_DBG(
			"T-Cal: Only %d point(s) with significant weight at %.2fC (need %d within %.1fC bandwidth)",
			points_with_weight,
			(double)current_temp,
			MLS_MIN_POINTS_FOR_FIT,
			(double)MLS_BANDWIDTH
		);
		return false;
	}

	// Log quality status (only once to avoid spam)
	static bool logged_quality = false;
	if (!logged_quality) {
		LOG_INF(
			"T-Cal: Quality check passed - %d points with weight at %.2fC (range: [%.2fC, %.2fC])",
			points_with_weight,
			(double)current_temp,
			(double)quality->temp_min,
			(double)quality->temp_max
		);
		logged_quality = true;
	}

	return true;
}

/**
 * Collect bias at current temperature (reuses standard calibration logic)
 * Does NOT save the point to calibration data
 */
static int sensor_boot_bias_collect(float *dest_bias, float *avg_temp)
{
	LOG_INF("Boot Cal: Starting bias collection (4-6 seconds)...");

	// Use the existing sensor_offsetBias function with same parameters
	// This ensures consistent quality between boot cal and normal cal
	float temp_range = NAN;
	float dummy_accel_bias[3] = {0};

	int err = sensor_offsetBias(dummy_accel_bias, dest_bias, avg_temp, &temp_range);

	if (err) {
		if (err == -1) {
			LOG_INF("Boot Cal: Motion detected during collection");
		} else if (err == -2) {
			LOG_ERR("Boot Cal: Timeout during collection");
		} else if (err == -3) {
			LOG_WRN("Boot Cal: Temperature unstable during collection");
		}
		return err;
	}

	LOG_INF(
		"Boot Cal: Collected bias [%.5f, %.5f, %.5f] at temp %.2fC (range: %.2fC)",
		(double)dest_bias[0],
		(double)dest_bias[1],
		(double)dest_bias[2],
		(double)*avg_temp,
		(double)temp_range
	);

	return 0;
}

/**
 * Collect bias for runtime calibration with shorter sampling time
 * Uses RUNTIME_CAL_SAMPLE_TIME_MS instead of BIAS_COLLECT_MAX_SAMPLE_TIME_MS
 * Does NOT save the point to calibration data
 */
static int sensor_runtime_bias_collect(float *dest_bias, float *avg_temp)
{
	LOG_INF("Runtime Cal: Starting bias collection (%d seconds)...", RUNTIME_CAL_SAMPLE_TIME_MS / 1000);

	// Use internal function with shorter sampling time for runtime calibration
	float temp_range = NAN;
	float dummy_accel_bias[3] = {0};

	// Runtime calibration uses shorter max time (3s) and shorter min time (2s)
	int min_sample_time = RUNTIME_CAL_SAMPLE_TIME_MS * 2 / 3; // ~2 seconds minimum
	int err = sensor_offsetBias_internal(
		dummy_accel_bias,
		dest_bias,
		avg_temp,
		&temp_range,
		RUNTIME_CAL_SAMPLE_TIME_MS,
		min_sample_time
	);

	if (err) {
		if (err == -1) {
			LOG_INF("Runtime Cal: Motion detected during collection");
		} else if (err == -2) {
			LOG_ERR("Runtime Cal: Timeout during collection");
		} else if (err == -3) {
			LOG_WRN("Runtime Cal: Temperature unstable during collection");
		}
		return err;
	}

	LOG_INF(
		"Runtime Cal: Collected bias [%.5f, %.5f, %.5f] at temp %.2fC (range: %.2fC)",
		(double)dest_bias[0],
		(double)dest_bias[1],
		(double)dest_bias[2],
		(double)*avg_temp,
		(double)temp_range
	);

	return 0;
}

/**
 * Calculate D_offset and store in runtime state (not persisted)
 * Uses unified strategy: MLS -> Skip if insufficient quality
 *
 * Skip D_offset calculation if:
 * 1. No valid temperature calibration (< 5 points or current temp not covered)
 * 2. Only basic single-point zero bias calibration exists
 *
 * This prevents using unreliable bias estimates from incomplete calibration.
 * Requires more than 4 sampling points to ensure proper temperature coverage.
 */
static int sensor_tcal_calculate_doffset_locked(const float measured_bias[3], float temp, uint16_t operation,
	struct led_token feedback, uint32_t generation)
{
	if (!tcal_compensation_enabled) {
		cal_event_end(operation, CAL_OUTCOME_SKIPPED, CAL_PHASE_VALIDATE, CAL_REASON_DISABLED);
		tcal_result(feedback, generation, LED_PARTIAL);
		tracker_events_notify();
		return 0;
	}
	// Check temperature calibration quality first
	tcal_quality_t quality;
	bool has_valid_tcal = sensor_tcal_assess_quality(temp, &quality);

	// Skip D_offset calculation if:
	// 1. No T-Cal data at all (count == 0)
	// 2. Not enough points (need > 4 points, i.e., at least 5 points)
	// 3. Current temperature is not covered by calibration points
	if (!has_valid_tcal || quality.point_count <= 4 || !quality.temp_in_range) {
		LOG_INF("Boot Cal: Skipping D_offset calculation - insufficient T-Cal quality");
		if (quality.point_count <= 4) {
			LOG_INF(
				"Boot Cal: Only %u calibration point(s), need more than 4 for reliable offset",
				quality.point_count
			);
		}
		if (!quality.temp_in_range && quality.point_count > 0) {
			LOG_INF(
				"Boot Cal: Current temp %.2fC outside calibrated range [%.2fC, %.2fC]",
				(double)temp,
				(double)quality.temp_min,
				(double)quality.temp_max
			);
		}

		cal_event_end(operation, CAL_OUTCOME_SKIPPED, CAL_PHASE_VALIDATE, CAL_REASON_NO_TCAL_COVERAGE);
		tcal_result(feedback, generation, LED_PARTIAL);
		tracker_events_notify();
		return 0; // Not an error, just skipped
	}

	// Calculate curve value at current temperature using MLS
	float curve_bias[3];
	bool offset_calculated = false;
	const char *method_name = "MLS";

	if (sensor_tcal_mls_lookup(temp, curve_bias) == 0) {
		offset_calculated = true;
		LOG_INF("D_offset: Using MLS method");
	}

	// If method failed, this should not happen since we checked quality
	// but handle it gracefully
	if (!offset_calculated) {
		LOG_ERR("D_offset: Failed to calculate curve bias despite passing quality check");
		cal_event_end(operation, CAL_OUTCOME_FAILED, CAL_PHASE_VALIDATE, CAL_REASON_FIT_ERROR);
		tcal_result(feedback, generation, LED_FAILED);
		tracker_events_notify();
		return -1;
	}

	LOG_INF(
		"D_offset: Baseline (%s) [%.5f, %.5f, %.5f] at temp %.2fC",
		method_name,
		(double)curve_bias[0],
		(double)curve_bias[1],
		(double)curve_bias[2],
		(double)temp
	);

// Calculate D_offset = measured - curve
// Apply a minimum threshold to filter out noise - values below threshold are set to 0
#define BOOT_CAL_DOFFSET_MIN_THRESHOLD 0.001f // dps - ignore tiny corrections

	for (int axis = 0; axis < 3; axis++) {
		float doffset = measured_bias[axis] - curve_bias[axis];

		// Apply threshold: if D_offset is too small, it's likely noise - don't correct
		// Reject NaN/±inf (all-ones exponent): NaN < threshold is false, so the
		// plain comparison would persist NaN into retained memory.
		uint32_t doffset_bits;
		memcpy(&doffset_bits, &doffset, sizeof(doffset_bits));
		if ((doffset_bits & 0x7F800000u) == 0x7F800000u
		    || fabsf(doffset) < BOOT_CAL_DOFFSET_MIN_THRESHOLD) {
			retained->bootCalState.doffset[axis] = 0.0f;
		} else {
			retained->bootCalState.doffset[axis] = doffset;
		}
	}

	retained->bootCalState.doffset_valid = true;
	sensor_tcal_mark_measured_bias();
	if (feedback.session) {
		tcal_feedback.token = feedback;
		tcal_feedback.reference_generation = reference_generation;
		tcal_feedback.calibration_generation = generation;
	}
	cal_event_end(operation, CAL_OUTCOME_SUCCESS, CAL_PHASE_APPLIED, CAL_REASON_NONE);
	tracker_events_notify();

	LOG_INF(
		"D_offset: Calculated [%.5f, %.5f, %.5f] (stored in retained memory)",
		(double)retained->bootCalState.doffset[0],
		(double)retained->bootCalState.doffset[1],
		(double)retained->bootCalState.doffset[2]
	);

	return 0;
}

static int sensor_tcal_calculate_doffset(const float measured_bias[3], float temp, uint16_t operation,
	struct led_token feedback, uint32_t generation)
{
	sensor_tcal_lock();
	int result = sensor_tcal_calculate_doffset_locked(measured_bias, temp, operation, feedback, generation);
	sensor_tcal_unlock();
	return result;
}

static void sensor_boot_cal_abandon(uint8_t reason)
{
	retained->bootCalState.completed = true;
	/* An admitted attempt owns its own terminal event, including past 30s. */
	if (sensor_calibration_request(CAL_REQUEST_QUERY, CAL_REQUEST_USER) == CAL_REQUEST_TCAL_BOOT) {
		return;
	}
	uint16_t operation = cal_event_begin(CAL_KIND_TCAL_BOOT | CAL_EVENT_ORIGIN_AUTO, CAL_PHASE_WAIT_STILL, 0);
	cal_event_end(operation, CAL_OUTCOME_SKIPPED, CAL_PHASE_WAIT_STILL, reason);
	tracker_events_notify();
}

/**
 * Main boot calibration check function
 * Called from sensor loop, manages state and timing
 * This function only checks conditions and requests calibration,
 * the actual calibration is performed by the calibration thread
 *
 * Boot calibration now works in three modes:
 * 1. With T-Cal data: Calculate D_offset as difference from T-Cal curve
 * 2. With static gyroBias: Calculate D_offset as difference from static bias
 * 3. No calibration data: Measure and store runtime bias directly
 */
void sensor_tcal_boot_calibration_check(void)
{
	// Check if feature is enabled
	if (!retained->bootCalState.enabled || !sensor_tcal_get_enabled()) {
		return;
	}

	// Check if already completed or requested
	if (retained->bootCalState.completed) {
		return;
	}

	// Check time window using uptime
	int64_t uptime = system_uptime_since_boot_ms();

	// Before window starts
	if (uptime < BOOT_CAL_TIME_WINDOW_START_MS) {
		return;
	}

	// After window ends - give up
	if (uptime >= BOOT_CAL_TIME_WINDOW_END_MS) {
		if (!retained->bootCalState.completed) {
			LOG_INF("Boot Cal: Time window expired (uptime: %lld ms), giving up", uptime);
			sensor_boot_cal_abandon(CAL_REASON_EXPIRED);
		}
		return;
	}

	// Check if another calibration is running
	if (sensor_calibration_request(CAL_REQUEST_QUERY, CAL_REQUEST_USER) != 0) {
		return; // Calibration in progress, wait
	}

	// Get current temperature
	float current_temp = sensor_get_current_imu_temperature();
	if (isnan(current_temp) || current_temp < -20.0f || current_temp > 60.0f) {
		return; // Invalid temperature
	}

	// Check T-Cal quality before proceeding
	// Boot calibration is only useful with sufficient T-Cal data
	// Skip if we have insufficient calibration points (<=4)
	tcal_quality_t quality;
	bool has_tcal = sensor_tcal_assess_quality(current_temp, &quality);

	// Log entry info (only once)
	static bool logged_entry = false;
	if (!logged_entry) {
		if (has_tcal && quality.point_count > BOOT_CAL_MIN_CURVE_POINTS) {
			LOG_INF("Boot Cal: Will use T-Cal data (%u points) for D_offset calculation", quality.point_count);
		} else if (quality.point_count > 0 && quality.point_count <= BOOT_CAL_MIN_CURVE_POINTS) {
			LOG_INF(
				"Boot Cal: Insufficient T-Cal points (%u <= %d), skipping boot calibration",
				quality.point_count,
				BOOT_CAL_MIN_CURVE_POINTS
			);
			sensor_boot_cal_abandon(CAL_REASON_NO_TCAL_COVERAGE);
			return;                                  // Skip boot calibration
		} else {
			LOG_INF("Boot Cal: No T-Cal data, skipping boot calibration");
			sensor_boot_cal_abandon(CAL_REASON_NO_TCAL_COVERAGE);
			return;                                  // Skip boot calibration
		}
		logged_entry = true;
	} else {
		// Check already logged, but still need to verify quality for this iteration
		if (!has_tcal || quality.point_count <= BOOT_CAL_MIN_CURVE_POINTS) {
			// Skip silently - already logged on first check
			if (!retained->bootCalState.completed) {
				sensor_boot_cal_abandon(CAL_REASON_NO_TCAL_COVERAGE);
			}
			return;
		}
	}

	// Log entry into time window (only once)
	static bool logged_window_entry = false;
	if (!logged_window_entry) {
		LOG_INF("Boot Cal: In time window (5-30s), uptime: %lld ms, waiting for stationary condition...", uptime);
		logged_window_entry = true;
	}

	// Request boot calibration through calibration request system
	// This will be executed by the calibration thread, avoiding deadlock
	int request_result = sensor_calibration_request(CAL_REQUEST_TCAL_BOOT, CAL_REQUEST_AUTO);
	if (request_result == 0) {
		LOG_INF("Boot Cal: Requested calibration through calibration thread");
	}
}

/**
 * Perform boot calibration (called by calibration thread)
 * Returns 0 on success, non-zero on failure
 *
 * Automatic requests have no local feedback token. User requests retain their
 * accepted token through collection and the first actual correction use.
 */
int sensor_perform_boot_calibration(void)
{
	const uint16_t operation = sensor_calibration_current_operation();
	const struct led_token feedback = sensor_calibration_current_feedback();
	const uint32_t generation = sensor_calibration_current_generation();
	if (!sensor_tcal_get_enabled()) {
		cal_event_end(operation, CAL_OUTCOME_SKIPPED, CAL_PHASE_WAIT_STILL, CAL_REASON_DISABLED);
		tcal_result(feedback, generation, LED_PARTIAL);
		tracker_events_notify();
		return 0;
	}
	LOG_INF("Boot Cal: Starting boot calibration");
	/* Session D_offset only — never write measured bias into tempCalPoints. */

	// Get current temperature
	float current_temp = sensor_get_current_imu_temperature();
	if (isnan(current_temp) || current_temp < -20.0f || current_temp > 60.0f) {
		LOG_ERR("Boot Cal: Invalid temperature");
		cal_event_end(operation, CAL_OUTCOME_FAILED, CAL_PHASE_WAIT_STILL, CAL_REASON_TEMPERATURE);
		tcal_result(feedback, generation, LED_FAILED);
		tracker_events_notify();
		return -1;
	}

	// Wait for device to be stationary
	if (!wait_for_motion(false, 6)) {
		LOG_WRN("Boot Cal: Device not stationary");
		retained->bootCalState.attempt_count++;

		if (retained->bootCalState.attempt_count >= BOOT_CAL_MAX_ATTEMPTS) {
			LOG_WRN("Boot Cal: Maximum attempts (%d) reached, giving up", BOOT_CAL_MAX_ATTEMPTS);
			retained->bootCalState.completed = true;
		}
		cal_event_end(operation, CAL_OUTCOME_FAILED, CAL_PHASE_WAIT_STILL, CAL_REASON_MOTION);
		tcal_result(feedback, generation, LED_FAILED);
		tracker_events_notify();
		return -1;
	}

	k_msleep(500); // Delay before beginning acquisition

	// Attempt to collect bias
	float measured_bias[3];
	float avg_temp;

	cal_event_step(operation, CAL_PHASE_COLLECT, 0);
	sensor_calibration_stage(feedback, LED_COLLECT_STILL);
	tracker_events_notify();
	int err = sensor_boot_bias_collect(measured_bias, &avg_temp);

	if (err) {
		uint8_t reason = err == -1 ? CAL_REASON_MOTION :
			err == -3 ? CAL_REASON_TEMPERATURE :
			err == BIAS_COLLECT_INSUFFICIENT_SAMPLES ? CAL_REASON_INSUFFICIENT_SAMPLES : CAL_REASON_SAMPLE_TIMEOUT;
		cal_event_end(operation, CAL_OUTCOME_FAILED, CAL_PHASE_COLLECT, reason);
		tcal_result(feedback, generation, LED_FAILED);
		tracker_events_notify();
		// Collection failed - check if we should trigger a full calibration
		retained->bootCalState.attempt_count++;

		if (retained->bootCalState.attempt_count >= BOOT_CAL_MAX_ATTEMPTS) {
			LOG_WRN("Boot Cal: Maximum attempts (%d) reached", BOOT_CAL_MAX_ATTEMPTS);

			// Check if we should auto-trigger single-side calibration to collect data
			tcal_quality_t quality;
			if (!sensor_tcal_assess_quality(current_temp, &quality)) {
				LOG_INF("Boot Cal: T-Cal quality insufficient, requesting single-side calibration");
				retained->bootCalState.completed = true; // Mark boot cal as complete to avoid re-entry

				/* This can remain busy until the worker releases its request slot. */
				int request_result = sensor_calibration_request(CAL_REQUEST_IMU, CAL_REQUEST_AUTO_SILENT);
				return request_result == 0 ? -2 : err;
			}

			retained->bootCalState.completed = true;
		}
		return err;
	}

	// Calculate D_offset
	cal_event_step(operation, CAL_PHASE_VALIDATE, 0);
	if (feedback.session && sensor_calibration_generation_valid(generation)) {
		sensor_calibration_result(feedback, LED_STAGE_ACK);
		sensor_calibration_stage(feedback, LED_PROCESSING);
	}
	tracker_events_notify();
	sys_warm_transaction_begin();
	if (!sensor_calibration_generation_valid(generation)) {
		sys_warm_transaction_end(false);
		cal_event_end(operation, CAL_OUTCOME_CANCELLED, CAL_PHASE_APPLY_PENDING, CAL_REASON_RESET);
		sensor_calibration_result(feedback, LED_CANCELLED);
		tracker_events_notify();
		return -ECANCELED;
	}
	err = sensor_tcal_calculate_doffset(measured_bias, avg_temp, operation, feedback, generation);
	if (err) {
		LOG_ERR("Boot Cal: Failed to calculate D_offset");
		retained->bootCalState.completed = true;
		sys_warm_transaction_end(false);
		return err;
	}

	// Success! Update fusion bias while preserving orientation
	retained->bootCalState.completed = true;
	sys_warm_transaction_end(false);

	// Record temperature and time for runtime calibration comparison
	runtime_cal_last_temp = avg_temp;
	runtime_cal_last_time = k_uptime_get();

	LOG_INF("Boot Cal: Completed successfully at %.2fC (uptime: %lld ms)", (double)avg_temp, runtime_cal_last_time);

	/* Local success waits for the sensor to consume this session correction. */
	return 0;
}

// Enable/disable boot calibration
int sensor_boot_cal_set_enabled(bool enabled)
{
	retained->bootCalState.enabled = enabled;
	LOG_INF("Boot Cal: %s", enabled ? "Enabled" : "Disabled");
	return sensor_operation_result(LED_OWNER_TCAL, 0, true);
}

// Enable/disable T-Cal compensation (persisted via NVS)
int sensor_tcal_set_enabled(bool enabled)
{
#if CONFIG_SENSOR_TCAL_HEATED
	if (sensor_calibration_maintenance_begin() != 0) {
		printk("T-Cal compensation unchanged: calibration busy.\n");
		return sensor_operation_result(LED_OWNER_TCAL, -EBUSY, false);
	}
#endif
	sys_warm_transaction_begin();
	sensor_tcal_lock();
	if (tcal_compensation_enabled == enabled) {
		int storage_error = tcal_compensation_storage_error;
		sensor_tcal_unlock();
		sys_warm_transaction_end(false);
#if CONFIG_SENSOR_TCAL_HEATED
		sensor_calibration_maintenance_end();
#endif
		LOG_INF("T-Cal compensation already %s", enabled ? "enabled" : "disabled");
		return sensor_operation_result(LED_OWNER_TCAL, storage_error, true);
	}
	tcal_compensation_enabled = enabled;
	retained->tcal_enabled = enabled;
	reference_generation++;
	retained->fusion_id = 0;
	if (!enabled && measured_bias_reset_pending) {
		/* A later enable must not reinterpret an unconsumed measurement. */
		sensor_tcal_clear_doffset();
	}
	sensor_tcal_refresh_apply_cache();
	sensor_tcal_unlock();
	int err = sys_write(TCAL_ENABLED_ID, &retained->tcal_enabled, &retained->tcal_enabled,
	          sizeof(retained->tcal_enabled));
	sensor_tcal_lock();
	tcal_compensation_storage_error = err;
	sensor_tcal_unlock();
	sys_warm_transaction_end(false);
#if CONFIG_SENSOR_TCAL_HEATED
	sensor_calibration_maintenance_end();
#endif
	LOG_INF(
		"T-Cal compensation %s (persisted) | apply=%s",
		enabled ? "enabled" : "disabled",
		sensor_tcal_get_apply_mode_name()
	);
	return sensor_operation_result(LED_OWNER_TCAL, err, true);
}

bool sensor_tcal_get_enabled(void)
{
	sensor_tcal_lock();
	bool enabled = tcal_compensation_enabled;
	sensor_tcal_unlock();
	return enabled;
}

// Get boot calibration status
bool sensor_boot_cal_is_completed(void)
{
	return retained->bootCalState.completed;
}

// Get boot calibration D_offset
void sensor_boot_cal_get_doffset(float offset[3])
{
	sensor_tcal_lock();
	if (retained->bootCalState.doffset_valid) {
		memcpy(offset, retained->bootCalState.doffset, sizeof(retained->bootCalState.doffset));
	} else {
		memset(offset, 0, sizeof(retained->bootCalState.doffset));
	}
	sensor_tcal_unlock();
}

// Reset boot calibration state (call before reboot/shutdown, not before WoM)
void sensor_boot_cal_reset(void)
{
	retained->bootCalState.completed = false;
	retained->bootCalState.attempt_count = 0;
	sensor_tcal_clear_doffset();
	LOG_INF("Boot Cal: State reset (will recalibrate on next boot)");
}

// =============================================================================
// Runtime Periodic Zero Bias Calibration Implementation
// =============================================================================

/**
 * Perform runtime zero bias calibration
 * Called by calibration thread when device has been at rest for extended period
 * This updates D_offset to track bias drift during long usage sessions
 *
 * Uses shorter sampling time (3 seconds) compared to normal calibration (4-6 seconds)
 * for quicker response while maintaining reasonable accuracy
 *
 * Automatic requests are locally silent. A user correction completes only
 * after collection and actual gyro application, without adding NVS writes.
 */
int sensor_perform_runtime_calibration(void)
{
	const uint16_t operation = sensor_calibration_current_operation();
	const struct led_token feedback = sensor_calibration_current_feedback();
	const uint32_t generation = sensor_calibration_current_generation();
	LOG_INF("Runtime Cal: Starting quick zero bias calibration (~3 seconds)");
	/* Updates D_offset only — does not append/overwrite tempCalPoints. */

	// Get current temperature
	float current_temp = sensor_get_current_imu_temperature();
	if (isnan(current_temp) || current_temp < -20.0f || current_temp > 60.0f) {
		LOG_ERR("Runtime Cal: Invalid temperature");
		// Apply failure cooldown to prevent immediate retry
		runtime_cal_last_time = k_uptime_get() - RUNTIME_CAL_COOLDOWN_MS + RUNTIME_CAL_FAILURE_COOLDOWN_MS;
		cal_event_end(operation, CAL_OUTCOME_FAILED, CAL_PHASE_WAIT_STILL, CAL_REASON_TEMPERATURE);
		tcal_result(feedback, generation, LED_FAILED);
		tracker_events_notify();
		return -1;
	}

	// Collect bias using short sampling period
	// Uses sensor_runtime_bias_collect with RUNTIME_CAL_SAMPLE_TIME_MS
	float measured_bias[3];
	float avg_temp;

	cal_event_step(operation, CAL_PHASE_COLLECT, 0);
	sensor_calibration_stage(feedback, LED_COLLECT_STILL);
	tracker_events_notify();
	int err = sensor_runtime_bias_collect(measured_bias, &avg_temp);

	if (err) {
		LOG_WRN("Runtime Cal: Bias collection failed (err: %d)", err);
		// Apply failure cooldown to prevent immediate retry
		runtime_cal_last_time = k_uptime_get() - RUNTIME_CAL_COOLDOWN_MS + RUNTIME_CAL_FAILURE_COOLDOWN_MS;
		uint8_t reason = err == -1 ? CAL_REASON_MOTION :
			err == -3 ? CAL_REASON_TEMPERATURE :
			err == BIAS_COLLECT_INSUFFICIENT_SAMPLES ? CAL_REASON_INSUFFICIENT_SAMPLES : CAL_REASON_SAMPLE_TIMEOUT;
		cal_event_end(operation, CAL_OUTCOME_FAILED, CAL_PHASE_COLLECT, reason);
		tcal_result(feedback, generation, LED_FAILED);
		tracker_events_notify();
		return err;
	}

	// Calculate D_offset using the unified function
	// This works regardless of whether T-Cal data exists
	cal_event_step(operation, CAL_PHASE_VALIDATE, 0);
	if (feedback.session && sensor_calibration_generation_valid(generation)) {
		sensor_calibration_result(feedback, LED_STAGE_ACK);
		sensor_calibration_stage(feedback, LED_PROCESSING);
	}
	tracker_events_notify();
	sys_warm_transaction_begin();
	if (!sensor_calibration_generation_valid(generation)) {
		sys_warm_transaction_end(false);
		cal_event_end(operation, CAL_OUTCOME_CANCELLED, CAL_PHASE_APPLY_PENDING, CAL_REASON_RESET);
		sensor_calibration_result(feedback, LED_CANCELLED);
		tracker_events_notify();
		return -ECANCELED;
	}
	err = sensor_tcal_calculate_doffset(measured_bias, avg_temp, operation, feedback, generation);
	sys_warm_transaction_end(false);
	if (err) {
		LOG_ERR("Runtime Cal: Failed to calculate D_offset");
		return err;
	}

	// Update runtime calibration timestamp and temperature
	runtime_cal_last_time = k_uptime_get();
	runtime_cal_last_temp = avg_temp;

	// Update fusion bias while preserving orientation
	LOG_INF("Runtime Cal: Completed at %.2fC, D_offset updated", (double)avg_temp);

	return 0;
}

/**
 * Check if runtime calibration should be triggered
 * Called from sensor loop when device is at rest
 *
 * @param is_resting true if device is currently at rest
 */
void sensor_runtime_calibration_check(bool is_resting)
{
	// Skip if runtime calibration is disabled
	if (!runtime_cal_enabled) {
		return;
	}

	// Skip if boot calibration hasn't completed yet
	if (!retained->bootCalState.completed) {
		return;
	}

	int64_t now = k_uptime_get();

	// Enforce minimum uptime before runtime calibration
	if (now < RUNTIME_CAL_MIN_UPTIME_MS) {
		return;
	}

	// Check cooldown period
	if (runtime_cal_last_time != 0 && (now - runtime_cal_last_time) < RUNTIME_CAL_COOLDOWN_MS) {
		return;
	}

	// Check if another calibration is running
	if (sensor_calibration_request(CAL_REQUEST_QUERY, CAL_REQUEST_USER) != 0) {
		runtime_cal_rest_tracking = false;
		runtime_cal_rest_start = 0;
		return;
	}

	// Get current temperature for comparison
	float current_temp = sensor_get_current_imu_temperature();

	if (is_resting) {
		// Start or continue tracking rest period
		if (!runtime_cal_rest_tracking) {
			runtime_cal_rest_tracking = true;
			runtime_cal_rest_start = now;
			LOG_DBG("Runtime Cal: Started tracking rest period");
		} else {
			// Check if we've been resting long enough
			int64_t rest_duration = now - runtime_cal_rest_start;
			if (rest_duration >= RUNTIME_CAL_REST_TIME_MS) {
				// Check temperature change since last calibration
				// Skip if temperature hasn't changed enough
				if (!isnan(runtime_cal_last_temp) && !isnan(current_temp)) {
					float temp_change = fabsf(current_temp - runtime_cal_last_temp);
					if (temp_change < RUNTIME_CAL_TEMP_CHANGE_MIN) {
						LOG_DBG(
							"Runtime Cal: Skipping - temp change %.2fC < %.2fC threshold",
							(double)temp_change,
							(double)RUNTIME_CAL_TEMP_CHANGE_MIN
						);
						// Reset tracking but don't request calibration
						runtime_cal_rest_tracking = false;
						runtime_cal_rest_start = 0;
						return;
					}
				}

				LOG_INF(
					"Runtime Cal: Device at rest for %lld ms, temp %.2fC (last: %.2fC), requesting calibration",
					rest_duration,
					(double)current_temp,
					isnan(runtime_cal_last_temp) ? 0.0 : (double)runtime_cal_last_temp
				);

				int request_result = sensor_calibration_request(CAL_REQUEST_TCAL_RUNTIME, CAL_REQUEST_AUTO);
				if (request_result == 0) {
					LOG_INF("Runtime Cal: Calibration requested");
					runtime_cal_rest_tracking = false;
					runtime_cal_rest_start = 0;
				}
			}
		}
	} else {
		// Device moved, reset rest tracking
		if (runtime_cal_rest_tracking) {
			LOG_DBG("Runtime Cal: Rest tracking reset due to motion");
			runtime_cal_rest_tracking = false;
			runtime_cal_rest_start = 0;
		}
	}
}

/**
 * Get runtime calibration status information
 */
void sensor_runtime_cal_get_status(int64_t *last_cal_time, int64_t *rest_duration)
{
	if (last_cal_time) {
		*last_cal_time = runtime_cal_last_time;
	}
	if (rest_duration) {
		if (runtime_cal_rest_tracking) {
			*rest_duration = k_uptime_get() - runtime_cal_rest_start;
		} else {
			*rest_duration = 0;
		}
	}
}

// =============================================================================
// T-Cal Test/Debug Functions
// =============================================================================

/**
 * Test and compare different calibration methods at a given temperature
 * Useful for debugging and understanding method differences
 */
void sensor_tcal_test_methods(float temp)
{
	/* Prediction below is deliberately independent of the enable switch.
	 * Only the sensor owner knows the offset actually used for the last sample. */
	float applied_bias[3];
	sensor_calibration_get_last_gyro_offset(applied_bias);
	printk("Selected apply path: %s\n", sensor_tcal_get_apply_mode_name());
	printk("Last applied gyro offset (not a prediction at test temperature):\n");
	printk("  [%.5f, %.5f, %.5f] dps\n",
	       (double)applied_bias[0], (double)applied_bias[1], (double)applied_bias[2]);
	if (retained->tempCalState.count < 1) {
		printk("No calibration data available.\n");
		return;
	}

	printk("\n=== T-Cal Method Comparison at %.2fC ===\n", (double)temp);
	printk("Total calibration points: %u\n\n", retained->tempCalState.count);

	// Show available calibration points
	float min_temp = INFINITY, max_temp = -INFINITY;
	int point_count = 0;
	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (retained->tempCalPoints[i].temp != 0.0f) {
			float t = retained->tempCalPoints[i].temp;
			if (t < min_temp) {
				min_temp = t;
			}
			if (t > max_temp) {
				max_temp = t;
			}
			point_count++;
		}
	}
	printk("Calibrated range: %.2fC to %.2fC\n", (double)min_temp, (double)max_temp);

	// Show temperature position
	if (temp < min_temp) {
		printk("Test temp is %.2fC BELOW calibrated range\n", (double)(min_temp - temp));
	} else if (temp > max_temp) {
		printk("Test temp is %.2fC ABOVE calibrated range\n", (double)(temp - max_temp));
	} else {
		printk("Test temp is WITHIN calibrated range\n");
	}
	printk("\n");

	// Method 1: MLS (Moving Least Squares - primary method)
	if (retained->tempCalState.count >= MLS_MIN_POINTS_FOR_FIT) {
		float mls_bias[3];
		int result = sensor_tcal_mls_lookup(temp, mls_bias);

		if (result == 0) {
			printk("Theoretical MLS prediction at test temperature (bandwidth=%.1fC):\n", (double)MLS_BANDWIDTH);
			printk("  Bias: [%.5f, %.5f, %.5f] dps\n", (double)mls_bias[0], (double)mls_bias[1], (double)mls_bias[2]);
		} else {
			printk("MLS Method: FAILED\n");
		}
		printk("\n");
	} else {
		printk("MLS Method: Not enough points (need >= %d)\n\n", MLS_MIN_POINTS_FOR_FIT);
	}

	float doffset[3];
	sensor_boot_cal_get_doffset(doffset);
	printk("Curve-relative session offset (used only on the curve path):\n");
	printk("  [%.5f, %.5f, %.5f] dps\n",
	       (double)doffset[0], (double)doffset[1], (double)doffset[2]);


	printk("\n=== End of T-Cal Method Comparison ===\n");
}

#endif /* CONFIG_SENSOR_USE_TCAL */
