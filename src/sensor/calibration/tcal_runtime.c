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
#include "sensor/sensor.h"
#include "system/system.h"
#include "system/uptime.h"
#include "system/watchdog.h"
#include "util.h"

#include <math.h>
#include <string.h>

#include "bias_collect.h"
#include "calibration.h"
#include "tcal_mls_lut.h"
#include "tcal_runtime.h"

#if CONFIG_SENSOR_USE_TCAL

LOG_MODULE_REGISTER(cal_tcal_runtime, LOG_LEVEL_INF);

#define TCAL_ACCUM_FLUSH_INTERVAL_MS 25000
#define TCAL_ACCUM_MIN_SAMPLES 2000
#define TCAL_ACCUM_TEMP_DRIFT_MAX 0.53f
#define TCAL_ACCUM_GYRO_RANGE_THRESHOLD 5.0f /* dps, windowed-mean range method */
#define TCAL_ACCUM_GYRO_MOTION_WINDOW_MS 250 /* ms - smooth raw gyro noise before range check */

#define TCAL_SAVE_SIGNIFICANCE_THRESHOLD 0.002f

static bool runtime_cal_enabled = false;
static int64_t runtime_cal_last_time = 0;
static int64_t runtime_cal_rest_start = 0;
static bool runtime_cal_rest_tracking = false;
float runtime_cal_last_temp = NAN;

static bool tcal_auto_calibration_enabled = false;
static bool tcal_compensation_enabled = true; /* until init_from_retained */
/* Hot-path cache: avoid count/enable re-check every gyro sample. */
static bool tcal_curve_apply_ready;

tcal_temp_direction_t tcal_current_direction = TCAL_DIR_UNKNOWN;
float tcal_direction_ref_temp = NAN;

static struct {
	double gyro_sum[3];
	double temp_sum;
	int sample_count;
	int temp_count;
	/* Gyro motion gate uses the range of short-window MEANS, not raw samples:
	 * a large but stable zero-rate offset plus high-frequency noise (e.g.
	 * >10 dps spread while stationary) must not abort accumulation. */
	double gyro_win_sum[3];
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
} tcal_accum;

static int64_t tcal_accum_last_commit_time = 0;

static void tcal_accum_flush(void);
static void tcal_save_point(int idx, const float bias[3], float measured_temp);
static int sensor_boot_bias_collect(float *dest_bias, float *avg_temp);
static int sensor_runtime_bias_collect(float *dest_bias, float *avg_temp);
static int sensor_tcal_calculate_doffset(const float measured_bias[3], float temp);

void sensor_tcal_refresh_apply_cache(void)
{
	tcal_curve_apply_ready =
		tcal_compensation_enabled && (retained->tempCalState.count >= MLS_MIN_POINTS_FOR_FIT);
}

bool sensor_tcal_curve_apply_ready(void)
{
	return tcal_curve_apply_ready;
}

sensor_tcal_apply_mode_t sensor_tcal_get_apply_mode(void)
{
	if (!tcal_compensation_enabled) {
		return SENSOR_TCAL_APPLY_DISABLED;
	}
	if (tcal_curve_apply_ready) {
		return SENSOR_TCAL_APPLY_CURVE;
	}
	return SENSOR_TCAL_APPLY_ZRO_FALLBACK;
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
	if (healed_points > 0) {
		retained->tempCalState.count = valid_points;
		retained->tempCalState.valid = false;
		retained_update();
	}

	tcal_compensation_enabled = retained->tcal_enabled;
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
	// Invalidate lookup cache since calibration data changed
	sensor_tcal_cache_invalidate();
	sensor_tcal_refresh_apply_cache();

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

	// Update fusion bias while preserving orientation
	sensor_fusion_update_bias(NULL);
}

void sensor_tcal_set_auto_calibration(bool enabled)
{
	tcal_auto_calibration_enabled = enabled;
	/* Do not set SYS_STATUS_CALIBRATION_RUNNING — that flag means blocking cal
	 * and would stall WOM/status_ready for continuous bucket sampling. */
	if (!enabled) {
		tcal_accum_reset();
	}
	LOG_INF("T-Cal Auto-calibration %s", enabled ? "enabled" : "disabled");
}

// Get auto-calibration enabled status
bool sensor_tcal_get_auto_calibration(void)
{
	return tcal_auto_calibration_enabled;
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

/**
 * Save a calibration point with hysteresis-aware blending.
 *
 * Slot = TEMP_TO_IDX(measured_temp). Store the measured average temperature
 * (not bucket center): bucket only addresses collisions; we do not assume
 * samples are uniform inside the bin.
 */
static void tcal_save_point(int idx, const float bias[3], float measured_temp)
{
	if (idx < 0 || idx >= TCAL_BUFFER_SIZE) {
		LOG_WRN("T-Cal: Index %d out of range, skipping", idx);
		return;
	}
	if (isnan(measured_temp)) {
		LOG_WRN("T-Cal: Invalid measured temperature, skipping save");
		return;
	}
	/* One NaN point breaks every MLS/LUT lookup (NaN weights pass the
	 * min-weight gate), so never commit a non-finite bias. */
	if (!v_finite(bias, 3)) {
		LOG_WRN("T-Cal: Non-finite bias, skipping save");
		return;
	}

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

	retained->tempCalPoints[idx].temp = measured_temp;
	memcpy(retained->tempCalPoints[idx].bias, final_bias, sizeof(float) * 3);
	retained->tempCalState.valid = false;

	LOG_INF(
		"T-Cal: Committed point at idx %d (%.2fC): [%.5f, %.5f, %.5f] (delta: %.4f dps)",
		idx,
		(double)measured_temp,
		(double)final_bias[0],
		(double)final_bias[1],
		(double)final_bias[2],
		(double)max_delta
	);

	if (is_new_point || max_delta >= TCAL_SAVE_SIGNIFICANCE_THRESHOLD) {
		update_tcal_state();
	} else {
		LOG_DBG(
			"T-Cal: Change below threshold (%.4f < %.4f), skipping warm NVS mark",
			(double)max_delta,
			(double)TCAL_SAVE_SIGNIFICANCE_THRESHOLD
		);
		/* Keep CRC valid for soft-reset; do not dirty NVS for insignificant churn. */
		retained_update();
		sensor_tcal_cache_invalidate();
	}
}

/**
 * Flush the accumulator: compute average bias/temperature, save to the
 * appropriate temperature bucket.
 */
static void tcal_accum_flush(void)
{
	if (!tcal_accum.active || tcal_accum.sample_count < TCAL_ACCUM_MIN_SAMPLES) {
		return;
	}

	float avg_bias[3];
	for (int axis = 0; axis < 3; axis++) {
		avg_bias[axis] = (float)(tcal_accum.gyro_sum[axis] / tcal_accum.sample_count);
	}

	float avg_temp = (tcal_accum.temp_count > 0) ? (float)(tcal_accum.temp_sum / tcal_accum.temp_count) : NAN;

	if (isnan(avg_temp)) {
		LOG_WRN("T-Cal: No valid temperature samples in accumulator, discarding");
		tcal_accum_reset();
		return;
	}

	/* Quasi-steady gate: reject fast thermal ramps so buckets stay near-static. */
	float elapsed_s = (float)(k_uptime_get() - tcal_accum.start_time) / 1000.0f;
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

	/* Last measured temp lives in retained; NVS only if point commit dirties warm set. */
	retained->gyroTemp = avg_temp;
	retained_update();

	tcal_save_point(idx, avg_bias, avg_temp);
	tcal_accum_last_commit_time = k_uptime_get();

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
void sensor_tcal_feed_continuous_sample(const float g[3], float temp)
{
	if (!tcal_auto_calibration_enabled) {
		return;
	}

	// Validate temperature
	if (isnan(temp) || temp < (float)CONFIG_SENSOR_POLY_TEMP_MIN || temp > (float)CONFIG_SENSOR_POLY_TEMP_MAX) {
		return;
	}

	// A NaN/±inf sample would poison gyro_sum and be committed as a NaN
	// calibration point (tcal_save_point validates temperature only).
	if (!v_finite(g, 3)) {
		return;
	}

	// Initialize accumulator on first sample
	if (!tcal_accum.active) {
		tcal_accum_reset();
		tcal_accum.active = true;
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
		float a0[3];
		if (sensor_peek_accel(a0)) {
			memcpy(tcal_accum.min_a, a0, sizeof(tcal_accum.min_a));
			memcpy(tcal_accum.max_a, a0, sizeof(tcal_accum.max_a));
			tcal_accum.accel_tracking = true;
		}
	}

	// Motion detection: gyro windowed-mean range
	for (int j = 0; j < 3; j++) {
		tcal_accum.gyro_win_sum[j] += (double)g[j];
	}
	tcal_accum.gyro_win_count++;
	if (tcal_accum.gyro_win_count >= tcal_accum.gyro_win_samples) {
		for (int j = 0; j < 3; j++) {
			float win_mean = (float)(tcal_accum.gyro_win_sum[j] / tcal_accum.gyro_win_count);
			tcal_accum.gyro_win_sum[j] = 0.0;
			if (!tcal_accum.gyro_win_tracked) {
				tcal_accum.min_g[j] = win_mean;
				tcal_accum.max_g[j] = win_mean;
			} else {
				if (win_mean < tcal_accum.min_g[j]) {
					tcal_accum.min_g[j] = win_mean;
				}
				if (win_mean > tcal_accum.max_g[j]) {
					tcal_accum.max_g[j] = win_mean;
				}
			}
		}
		tcal_accum.gyro_win_count = 0;
		tcal_accum.gyro_win_tracked = true;
		for (int j = 0; j < 3; j++) {
			if (tcal_accum.max_g[j] - tcal_accum.min_g[j] > TCAL_ACCUM_GYRO_RANGE_THRESHOLD) {
				LOG_DBG(
					"T-Cal: Gyro motion in accumulator, axis %d (windowed-mean range: %.3f dps), resetting",
					j,
					(double)(tcal_accum.max_g[j] - tcal_accum.min_g[j])
				);
				tcal_accum_reset();
				return;
			}
		}
	}

	/*
	 * Accel peak–peak: peek every TCAL_ACCUM_ACCEL_PEEK_DIV samples only.
	 * Accel updates ~100Hz; checking every raw gyro sample was wasted work.
	 */
	if (++tcal_accum.accel_peek_div >= TCAL_ACCUM_ACCEL_PEEK_DIV) {
		tcal_accum.accel_peek_div = 0;
		float a[3];
		if (sensor_peek_accel(a)) {
			if (!tcal_accum.accel_tracking) {
				memcpy(tcal_accum.min_a, a, sizeof(tcal_accum.min_a));
				memcpy(tcal_accum.max_a, a, sizeof(tcal_accum.max_a));
				tcal_accum.accel_tracking = true;
			} else {
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
						return;
					}
				}
			}
		}
	}

	// Accumulate gyro
	tcal_accum.gyro_sum[0] += (double)g[0];
	tcal_accum.gyro_sum[1] += (double)g[1];
	tcal_accum.gyro_sum[2] += (double)g[2];
	tcal_accum.sample_count++;

	// Accumulate temperature
	if (temp < tcal_accum.temp_min) {
		tcal_accum.temp_min = temp;
	}
	if (temp > tcal_accum.temp_max) {
		tcal_accum.temp_max = temp;
	}
	tcal_accum.temp_sum += (double)temp;
	tcal_accum.temp_count++;

	// Check flush conditions
	int64_t elapsed = k_uptime_get() - tcal_accum.start_time;
	float temp_drift = tcal_accum.temp_max - tcal_accum.temp_min;

	// Condition 1: Temperature drifted beyond one bucket width — flush early
	// to avoid cross-bucket contamination, then start a new accumulation window
	if (temp_drift > TCAL_ACCUM_TEMP_DRIFT_MAX && tcal_accum.sample_count >= TCAL_ACCUM_MIN_SAMPLES) {
		LOG_INF("T-Cal: Temperature drift %.2fC exceeded threshold, early flush", (double)temp_drift);
		tcal_accum_flush();
		return;
	}

	// Condition 2: Flush interval reached
	if (elapsed >= TCAL_ACCUM_FLUSH_INTERVAL_MS && tcal_accum.sample_count >= TCAL_ACCUM_MIN_SAMPLES) {
		tcal_accum_flush();
		return;
	}
}

/**
 * Called when motion is detected — flush if enough data, then reset.
 */
void sensor_tcal_continuous_motion_detected(void)
{
	if (tcal_accum.active) {
		if (tcal_accum.sample_count >= TCAL_ACCUM_MIN_SAMPLES) {
			tcal_accum_flush();
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

	if (!tcal_auto_calibration_enabled) {
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

	int request_result = sensor_calibration_request(1);
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
static int sensor_tcal_calculate_doffset(const float measured_bias[3], float temp)
{
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

		// Mark D_offset as invalid - use existing ZRO calibration only
		retained->bootCalState.doffset_valid = false;
		retained->bootCalState.doffset[0] = 0.0f;
		retained->bootCalState.doffset[1] = 0.0f;
		retained->bootCalState.doffset[2] = 0.0f;
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
		retained->bootCalState.doffset_valid = false;
		retained->bootCalState.doffset[0] = 0.0f;
		retained->bootCalState.doffset[1] = 0.0f;
		retained->bootCalState.doffset[2] = 0.0f;
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

	LOG_INF(
		"D_offset: Calculated [%.5f, %.5f, %.5f] (stored in retained memory)",
		(double)retained->bootCalState.doffset[0],
		(double)retained->bootCalState.doffset[1],
		(double)retained->bootCalState.doffset[2]
	);

	return 0;
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
	if (!retained->bootCalState.enabled) {
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
			retained->bootCalState.completed = true;
		}
		return;
	}

	// Check if another calibration is running
	if (sensor_calibration_request(0) != 0) {
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
			retained->bootCalState.completed = true; // Mark as completed to avoid repeated checks
			return;                                  // Skip boot calibration
		} else {
			LOG_INF("Boot Cal: No T-Cal data, skipping boot calibration");
			retained->bootCalState.completed = true; // Mark as completed to avoid repeated checks
			return;                                  // Skip boot calibration
		}
		logged_entry = true;
	} else {
		// Check already logged, but still need to verify quality for this iteration
		if (!has_tcal || quality.point_count <= BOOT_CAL_MIN_CURVE_POINTS) {
			// Skip silently - already logged on first check
			if (!retained->bootCalState.completed) {
				retained->bootCalState.completed = true;
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
	int request_result = sensor_calibration_request(3); // Use ID 3 for boot calibration
	if (request_result == 0) {
		LOG_INF("Boot Cal: Requested calibration through calibration thread");
	}
}

/**
 * Perform boot calibration (called by calibration thread)
 * Returns 0 on success, non-zero on failure
 *
 * Note: This is an automatic calibration - no LED changes to keep it
 * transparent to the user. LED state is preserved throughout.
 */
int sensor_perform_boot_calibration(void)
{
	LOG_INF("Boot Cal: Starting boot calibration");
	/* Session D_offset only — never write measured bias into tempCalPoints. */
	// Note: No LED changes for automatic boot calibration - keep it transparent

	// Get current temperature
	float current_temp = sensor_get_current_imu_temperature();
	if (isnan(current_temp) || current_temp < -20.0f || current_temp > 60.0f) {
		LOG_ERR("Boot Cal: Invalid temperature");
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
		return -1;
	}

	k_msleep(500); // Delay before beginning acquisition

	// Attempt to collect bias
	float measured_bias[3];
	float avg_temp;

	int err = sensor_boot_bias_collect(measured_bias, &avg_temp);

	if (err) {
		// Collection failed - check if we should trigger a full calibration
		retained->bootCalState.attempt_count++;

		if (retained->bootCalState.attempt_count >= BOOT_CAL_MAX_ATTEMPTS) {
			LOG_WRN("Boot Cal: Maximum attempts (%d) reached", BOOT_CAL_MAX_ATTEMPTS);

			// Check if we should auto-trigger single-side calibration to collect data
			tcal_quality_t quality;
			if (!sensor_tcal_assess_quality(current_temp, &quality)) {
				LOG_INF("Boot Cal: T-Cal quality insufficient, triggering single-side calibration");
				retained->bootCalState.completed = true; // Mark boot cal as complete to avoid re-entry

				// Request standard calibration to collect tcal data
				sensor_request_calibration();
				return -2; // Special error code indicating auto-calibration triggered
			}

			retained->bootCalState.completed = true;
		}
		return err;
	}

	// Calculate D_offset
	err = sensor_tcal_calculate_doffset(measured_bias, avg_temp);
	if (err) {
		LOG_ERR("Boot Cal: Failed to calculate D_offset");
		retained->bootCalState.completed = true;
		return err;
	}

	// Success! Update fusion bias while preserving orientation
	retained->bootCalState.completed = true;

	// Record temperature and time for runtime calibration comparison
	runtime_cal_last_temp = avg_temp;
	runtime_cal_last_time = k_uptime_get();

	LOG_INF("Boot Cal: Completed successfully at %.2fC (uptime: %lld ms)", (double)avg_temp, runtime_cal_last_time);
	sensor_fusion_update_bias(NULL);

	// Note: No LED flash for automatic boot calibration - keep it transparent
	return 0;
}

// Enable/disable boot calibration
void sensor_boot_cal_set_enabled(bool enabled)
{
	retained->bootCalState.enabled = enabled;
	LOG_INF("Boot Cal: %s", enabled ? "Enabled" : "Disabled");
}

// Enable/disable T-Cal compensation (persisted via NVS)
void sensor_tcal_set_enabled(bool enabled)
{
	if (tcal_compensation_enabled == enabled) {
		LOG_INF("T-Cal compensation already %s", enabled ? "enabled" : "disabled");
		return;
	}
	tcal_compensation_enabled = enabled;
	bool val = enabled;
	sys_write(TCAL_ENABLED_ID, &retained->tcal_enabled, &val, sizeof(val));
	sensor_tcal_refresh_apply_cache();
	LOG_INF(
		"T-Cal compensation %s (persisted) | apply=%s",
		enabled ? "enabled" : "disabled",
		sensor_tcal_get_apply_mode_name()
	);
}

bool sensor_tcal_get_enabled(void)
{
	return tcal_compensation_enabled;
}

// Get boot calibration status
bool sensor_boot_cal_is_completed(void)
{
	return retained->bootCalState.completed;
}

// Get boot calibration D_offset
void sensor_boot_cal_get_doffset(float offset[3])
{
	if (retained->bootCalState.doffset_valid) {
		memcpy(offset, retained->bootCalState.doffset, sizeof(retained->bootCalState.doffset));
	} else {
		memset(offset, 0, sizeof(retained->bootCalState.doffset));
	}
}

// Reset boot calibration state (call before reboot/shutdown, not before WoM)
void sensor_boot_cal_reset(void)
{
	retained->bootCalState.completed = false;
	retained->bootCalState.attempt_count = 0;
	retained->bootCalState.doffset_valid = false;
	retained->bootCalState.doffset[0] = 0.0f;
	retained->bootCalState.doffset[1] = 0.0f;
	retained->bootCalState.doffset[2] = 0.0f;
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
 * Note: This is an automatic calibration - no LED changes to keep it
 * transparent to the user. LED state is preserved throughout.
 */
int sensor_perform_runtime_calibration(void)
{
	LOG_INF("Runtime Cal: Starting quick zero bias calibration (~3 seconds)");
	/* Updates D_offset only — does not append/overwrite tempCalPoints. */
	// Note: No LED changes for automatic runtime calibration - keep it transparent

	// Get current temperature
	float current_temp = sensor_get_current_imu_temperature();
	if (isnan(current_temp) || current_temp < -20.0f || current_temp > 60.0f) {
		LOG_ERR("Runtime Cal: Invalid temperature");
		// Apply failure cooldown to prevent immediate retry
		runtime_cal_last_time = k_uptime_get() - RUNTIME_CAL_COOLDOWN_MS + RUNTIME_CAL_FAILURE_COOLDOWN_MS;
		return -1;
	}

	// Collect bias using short sampling period
	// Uses sensor_runtime_bias_collect with RUNTIME_CAL_SAMPLE_TIME_MS
	float measured_bias[3];
	float avg_temp;

	int err = sensor_runtime_bias_collect(measured_bias, &avg_temp);

	if (err) {
		LOG_WRN("Runtime Cal: Bias collection failed (err: %d)", err);
		// Apply failure cooldown to prevent immediate retry
		runtime_cal_last_time = k_uptime_get() - RUNTIME_CAL_COOLDOWN_MS + RUNTIME_CAL_FAILURE_COOLDOWN_MS;
		return err;
	}

	// Calculate D_offset using the unified function
	// This works regardless of whether T-Cal data exists
	err = sensor_tcal_calculate_doffset(measured_bias, avg_temp);
	if (err) {
		LOG_ERR("Runtime Cal: Failed to calculate D_offset");
		return err;
	}

	// Update runtime calibration timestamp and temperature
	runtime_cal_last_time = k_uptime_get();
	runtime_cal_last_temp = avg_temp;

	// Update fusion bias while preserving orientation
	LOG_INF("Runtime Cal: Completed at %.2fC, D_offset updated", (double)avg_temp);
	sensor_fusion_update_bias(NULL);

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
	if (sensor_calibration_request(0) != 0) {
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

				// Request runtime calibration (uses calibration request ID 4)
				int request_result = sensor_calibration_request(4);
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
			printk("MLS Method (bandwidth=%.1fC):\n", (double)MLS_BANDWIDTH);
			printk("  Bias: [%.5f, %.5f, %.5f] dps\n", (double)mls_bias[0], (double)mls_bias[1], (double)mls_bias[2]);
		} else {
			printk("MLS Method: FAILED\n");
		}
		printk("\n");
	} else {
		printk("MLS Method: Not enough points (need >= %d)\n\n", MLS_MIN_POINTS_FOR_FIT);
	}

	// Show boot cal D_offset if active
	if (retained->bootCalState.doffset_valid) {
		printk("Additional Offsets:\n");
		printk(
			"  Boot cal D_offset: [%.5f, %.5f, %.5f] dps\n",
			(double)retained->bootCalState.doffset[0],
			(double)retained->bootCalState.doffset[1],
			(double)retained->bootCalState.doffset[2]
		);
		printk("\n");
	}

	// Show final effective bias that would be applied
	printk("Final Effective Bias (as applied to gyro data):\n");

	// Calculate what would actually be used using unified strategy: MLS -> Static
	float final_bias[3] = {0.0f, 0.0f, 0.0f};
	bool calculated = false;
	const char *method_used = "static";

	if (sensor_tcal_mls_lookup(temp, final_bias) == 0) {
		calculated = true;
		method_used = "MLS";
	}

	if (calculated) {
		// tempCalCorrectionOffset is retained for compatibility only; no longer used.

		// Add boot cal D_offset if valid
		if (retained->bootCalState.doffset_valid) {
			for (int i = 0; i < 3; i++) {
				final_bias[i] += retained->bootCalState.doffset[i];
			}
		}

		printk(
			"  Total: [%.5f, %.5f, %.5f] dps\n",
			(double)final_bias[0],
			(double)final_bias[1],
			(double)final_bias[2]
		);
		printk("  Method: %s (Unified Strategy: MLS -> Static)\n", method_used);
	} else {
		printk(
			"  Fallback to static bias: [%.5f, %.5f, %.5f] dps\n",
			(double)retained->gyroBias[0],
			(double)retained->gyroBias[1],
			(double)retained->gyroBias[2]
		);
		printk("  (No valid T-Cal method available)\n");
	}

	printk("\n=== End of T-Cal Method Comparison ===\n");
}

#endif /* CONFIG_SENSOR_USE_TCAL */
