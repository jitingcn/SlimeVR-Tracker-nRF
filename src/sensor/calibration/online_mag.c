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
#include "util.h"

#include <math.h>
#include <string.h>

#include "sensor/magneto/magneto1_4.h"

#include "calibration.h"
#include "mag_common.h"
#include "online_mag.h"

LOG_MODULE_REGISTER(cal_online_mag, LOG_LEVEL_INF);

static mag_center_estimator_t online_center_estimator;

// Per-quadrant ring buffer for online magnetometer calibration.
// Combines the directional coverage guarantee of quadrant-based sampling
// (8 octants based on sign of x, y, z) with the natural aging of FIFO
// per-quadrant sliding windows. Each octant independently wraps after
// QUADRANT_BUF_SIZE samples — staying in one orientation only updates
// that octant, leaving the other 7 with diverse data.
#define QUADRANT_BUF_SIZE 32
#define ONLINE_QUADRANT_COUNT 8
#define ONLINE_BUFFER_SAMPLE_CAPACITY (ONLINE_QUADRANT_COUNT * QUADRANT_BUF_SIZE)

typedef struct {
	float x, y, z;
} quadrant_sample_t;

typedef struct {
	quadrant_sample_t samples[QUADRANT_BUF_SIZE];
	uint8_t head;   // next write position
	uint8_t count;  // valid samples (0..QUADRANT_BUF_SIZE)
	int64_t last_seq; // global accepted-sample sequence of the newest sample in this octant
} quadrant_buf_t;

// Incremental calibration blending (EMA on BAinv elements)
// Base alpha: weight given to new trial calibration. A value of 0.35 means
// 35% new + 65% existing → gradual convergence over ~3 updates.
// Higher when trial diverges significantly from existing (environment change).
#define ONLINE_BLEND_BASE_ALPHA 0.35f
#define ONLINE_BLEND_MIN_ALPHA 0.12f   // floor: very similar calibrations
#define ONLINE_BLEND_MAX_ALPHA 0.70f   // ceiling: significant divergence detected
#define ONLINE_BLEND_SIMILARITY_LOW 0.85f   // below this similarity, increase alpha
#define ONLINE_BLEND_SIMILARITY_HIGH 0.97f  // above this similarity, use min alpha

static quadrant_buf_t quad_buf[ONLINE_QUADRANT_COUNT];
static int64_t online_total_sample_count;
static int64_t online_last_checked_sample_count;
static int64_t online_last_check_time;
static int64_t online_last_sample_time; // rate limiting
// Drop octants that have not been refreshed for too long.
// This is kept separate from the check cadence: stale-history rejection should
// not depend on how often the background thread decides to run Magneto.
#define ONLINE_STALE_QUADRANT_MAX_AGE (ONLINE_BUFFER_SAMPLE_CAPACITY * 5 / 2)
// Minimum direction change to accept an online sample. The configured value is
// expressed in degrees and converted to the equivalent 1 - cos(theta) threshold.
static float online_last_dir[3];
static float online_last_accel_dir[3]; // accel direction for cross-validation

#define ONLINE_MIN_DIR_CHANGE_DEG 10.0f
#define ONLINE_MIN_INTERVAL_MS 30  // minimum 30ms between online samples
// Background checks should not run on every calibration-thread pass.
// Tie the minimum check spacing to roughly one fresh fit's worth of accepted
// samples at the maximum online sampling rate.
#define ONLINE_MIN_CHECK_INTERVAL_MS (ONLINE_BUFFER_SAMPLE_CAPACITY * ONLINE_MIN_INTERVAL_MS)

// Runtime calibrated norm tracking (exponential moving average)
// Used to assess current calibration quality and decide if online update is needed
static float cal_norm_ema;        // EMA of calibrated mag norm
static float cal_norm_var_ema;    // EMA of squared deviation from mean
static uint32_t cal_norm_count;   // number of norm samples processed
#define CAL_NORM_EMA_ALPHA 0.01f  // smoothing factor (~100 sample window)
// Don't update calibration if current norm CV is below this threshold
#define CAL_NORM_GOOD_CV 0.05f    // 5% = good enough calibration

// Minimum time between online calibration updates (prevents frequent VQF mag ref resets)
#define ONLINE_MIN_UPDATE_INTERVAL_S 6  // 6 seconds cooldown
static int64_t online_last_update_time;
static bool sensor_calibration_online_mag_enabled_from_retained(void);

// Suppress online sample collection for N ms after buffer resets (wake-up,
// reboot, environment change, calibration update).  This lets sensor data
// stabilise before collecting, avoiding transient/mixed-environment samples
// that produce poor calibration fits.
#define ONLINE_COLLECTION_SUPPRESS_MS 1500
static int64_t online_collection_suppress_until;

// Minimum sustained VQF magnetic disturbance duration before allowing an online
// calibration update.  Short disturbance bursts are usually transient interference;
// updating calibration during those resets VQF's heading reference for no benefit.
#define ONLINE_VQF_DIST_MIN_DURATION_MS 3000
static int64_t online_mag_dist_start_time;

// Require at least N successful calibration updates before trusting the
// norm CV gate. Prevents a single early fit from being declared "good enough"
// when the buffer is still filling and directional coverage is incomplete.
#define ONLINE_MIN_UPDATES 3
static int online_update_count;

// Norm-change detection: when the average raw field strength in the buffer
// changes by >40% between updates, the magnetic environment has changed.
// We clear buffers to avoid mixed-data fits and let the next cycle fit
// on consistent data from the new environment.
static float online_last_buf_avg_norm = 0.0f;


static void magneto_online_clear_history(void);
static bool magneto_online_quadrant_is_recent(const quadrant_buf_t *qbuf);
static double magneto_online_recent_center(mag_center_estimator_t *center);
static double magneto_online_collect_recent(double ata_out[100], double *norm_sum_out,
					   float dir_sum_out[3], float *raw_range_out);
static int magneto_online_recent_sample_count(void);
static float magneto_online_recent_dir_bias(void);
static float magneto_directional_bias(const float ds[3], double count);
static void magneto_accumulate_direction(float ds[3], const float v[3]);
static float magneto_BAinv_similarity(float existing[4][3], float candidate[4][3]);
static bool magneto_blend_BAinv(float out[4][3], float existing[4][3], float candidate[4][3]);

int cal_online_mag_update_count(void)
{
	return online_update_count;
}

uint32_t cal_online_mag_norm_count(void)
{
	return cal_norm_count;
}

void magneto_online_runtime_load_retained(void)
{
	magneto_online_reset();
	online_update_count = retained->onlineMagState.update_count;
	online_last_buf_avg_norm = retained->onlineMagState.last_buf_avg_norm;
}

static bool sensor_calibration_online_mag_enabled_from_retained(void)
{
	return retained->mag_online_calibration_mode != MAG_ONLINE_CALIBRATION_DISABLED;
}

bool sensor_calibration_get_online_mag_enabled(void)
{
	return sensor_calibration_online_mag_enabled_from_retained();
}

void sensor_calibration_set_online_mag_enabled(bool enabled)
{
	uint8_t mode = enabled ? MAG_ONLINE_CALIBRATION_ENABLED : MAG_ONLINE_CALIBRATION_DISABLED;

	if (sensor_calibration_get_online_mag_enabled() == enabled &&
	    retained->mag_online_calibration_mode == mode) {
		LOG_INF("Online mag calibration already %s", enabled ? "enabled" : "disabled");
		return;
	}

	magneto_online_runtime_reset();
	if (!enabled) {
		sensor_calibration_online_mag_retained_clear();
	}
	sys_write(
		MAG_ONLINE_CALIBRATION_ID,
		&retained->mag_online_calibration_mode,
		&mode,
		sizeof(mode)
	);
	LOG_INF("Online mag calibration %s (persisted)", enabled ? "enabled" : "disabled");
}

void sensor_calibration_online_mag_retained_save(void)
{
	if (!sensor_calibration_get_online_mag_enabled()) {
		sensor_calibration_online_mag_retained_clear();
		return;
	}
	retained->onlineMagState.update_count = (uint8_t)CLAMP(online_update_count, 0, 255);
	retained->onlineMagState.last_buf_avg_norm = online_last_buf_avg_norm;
}

void sensor_calibration_online_mag_retained_clear(void)
{
	memset(&retained->onlineMagState, 0, sizeof(retained->onlineMagState));
}

void sensor_calibration_online_mag_cold_start(void)
{
	magneto_online_runtime_reset();
	sensor_calibration_online_mag_retained_clear();
}


void magneto_online_reset(void)
{
	magneto_online_clear_history();
	online_last_sample_time = 0;
	online_last_update_time = 0;
}

void magneto_online_runtime_reset(void)
{
	magneto_online_reset();
	online_update_count = 0;
	online_last_buf_avg_norm = 0.0f;
	cal_norm_count = 0;
	cal_norm_ema = 0.0f;
	cal_norm_var_ema = 0.0f;
}

static void magneto_online_clear_history(void)
{
	memset(quad_buf, 0, sizeof(quad_buf));
	online_total_sample_count = 0;
	online_last_checked_sample_count = 0;
	online_last_check_time = 0;
	memset(online_last_dir, 0, sizeof(online_last_dir));
	memset(online_last_accel_dir, 0, sizeof(online_last_accel_dir));
	magneto_center_reset(&online_center_estimator);
	// Suppress collection for a few seconds so transient/stale samples from
	// wake-up, reboot, or environment transitions are not mixed into the
	// fresh buffer.
	online_collection_suppress_until = k_uptime_get() + ONLINE_COLLECTION_SUPPRESS_MS;
}

static bool magneto_online_quadrant_is_recent(const quadrant_buf_t *qbuf)
{
	if (qbuf->count == 0) {
		return false;
	}
	return (online_total_sample_count - qbuf->last_seq) <= ONLINE_STALE_QUADRANT_MAX_AGE;
}

static double magneto_online_recent_center(mag_center_estimator_t *center)
{
	magneto_center_reset(center);

	double count = 0;
	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent(&quad_buf[q])) {
			continue;
		}
		for (int i = 0; i < quad_buf[q].count; i++) {
			quadrant_sample_t *s = &quad_buf[q].samples[i];
			float m[3] = {s->x, s->y, s->z};
			magneto_center_update(center, m);
			count++;
		}
	}

	return count;
}

// Collect all valid samples from all 8 quadrant ring buffers.
// Recomputes ATA, norm_sum, centered dir_sum, and raw min/max range from raw samples.
static double magneto_online_collect_recent(double ata_out[100], double *norm_sum_out,
                                            float dir_sum_out[3], float *raw_range_out)
{
	memset(ata_out, 0, sizeof(double) * 100);
	*norm_sum_out = 0;
	memset(dir_sum_out, 0, sizeof(float) * 3);

	mag_center_estimator_t recent_center;
	double recent_sample_count = magneto_online_recent_center(&recent_center);
	if (raw_range_out) {
		*raw_range_out = magneto_center_min_range(&recent_center);
	}

	double fit_sample_count = 0;
	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent(&quad_buf[q])) {
			continue;
		}
		for (int i = 0; i < quad_buf[q].count; i++) {
			quadrant_sample_t *s = &quad_buf[q].samples[i];
			magneto_sample((double)s->x, (double)s->y, (double)s->z, ata_out, norm_sum_out, &fit_sample_count);
			float raw[3] = {s->x, s->y, s->z};
			float coverage_sample[3];
			magneto_coverage_sample(&recent_center, raw, coverage_sample);
			magneto_accumulate_direction(dir_sum_out, coverage_sample);
		}
	}
	return recent_sample_count;
}

static int magneto_online_recent_sample_count(void)
{
	int count = 0;
	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent(&quad_buf[q])) {
			continue;
		}
		count += quad_buf[q].count;
	}
	return count;
}

static float magneto_online_recent_dir_bias(void)
{
	float dir_sum_recent[3] = {0};
	mag_center_estimator_t recent_center;
	double recent_sample_count = magneto_online_recent_center(&recent_center);

	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent(&quad_buf[q])) {
			continue;
		}
		for (int i = 0; i < quad_buf[q].count; i++) {
			quadrant_sample_t *s = &quad_buf[q].samples[i];
			float raw[3] = {s->x, s->y, s->z};
			float coverage_sample[3];
			magneto_coverage_sample(&recent_center, raw, coverage_sample);
			magneto_accumulate_direction(dir_sum_recent, coverage_sample);
		}
	}

	return magneto_directional_bias(dir_sum_recent, recent_sample_count);
}

float magneto_online_min_dir_change_threshold(void)
{
	static bool initialized = false;
	static float threshold = 0.0f;

	if (!initialized) {
		const float deg_to_rad = 0.01745329251994329577f;
		threshold = 1.0f - cosf(ONLINE_MIN_DIR_CHANGE_DEG * deg_to_rad);
		initialized = true;
	}

	return threshold;
}


/**
 * Compute directional bias of accumulated mag samples.
 * Returns |sum(m/|m|)| / N, where 0=perfect sphere coverage, 1=all same direction.
 */
static float magneto_directional_bias(const float ds[3], double count)
{
	if (count < 2) {
		return 1.0f;
	}
	float inv_n = 1.0f / (float)count;
	float cx = ds[0] * inv_n;
	float cy = ds[1] * inv_n;
	float cz = ds[2] * inv_n;
	return sqrtf(cx * cx + cy * cy + cz * cz);
}


/**
 * Accumulate a normalized direction for diversity tracking.
 */
static void magneto_accumulate_direction(float ds[3], const float v[3])
{
	float dir[3];
	if (!magneto_normalize_direction(v, dir)) {
		return;
	}
	ds[0] += dir[0];
	ds[1] += dir[1];
	ds[2] += dir[2];
}


/**
 * Compute similarity between two BAinv calibration matrices.
 * Uses normalized Frobenius norm: similarity = 1.0 - ||candidate - existing|| / ||existing||.
 * Returns 1.0 for identical calibrations, approaching 0 for very different ones.
 * The offset row (row 0) and soft-iron rows (1-3) contribute equally to the norm.
 */
static float magneto_BAinv_similarity(float existing[4][3], float candidate[4][3])
{
	float diff_norm_sq = 0;
	float existing_norm_sq = 0;

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 3; c++) {
			float d = candidate[r][c] - existing[r][c];
			diff_norm_sq += d * d;
			existing_norm_sq += existing[r][c] * existing[r][c];
		}
	}

	if (existing_norm_sq < 1e-12f) {
		// Existing calibration is near-zero (identity): treat as low similarity
		return 0.0f;
	}

	float similarity = 1.0f - sqrtf(diff_norm_sq / existing_norm_sq);
	if (similarity < 0.0f) { similarity = 0.0f; }
	if (similarity > 1.0f) { similarity = 1.0f; }
	return similarity;
}

/**
 * Blend two BAinv calibrations using exponential moving average (EMA).
 * blended = (1 - alpha) * existing + alpha * candidate
 * Alpha is computed from similarity: more similar → lower alpha (conservative),
 * more different → higher alpha (adaptive to environmental change).
 *
 * The blended result is validated with the same structural checks as
 * magneto_quality_check. If blending produces an invalid result, the
 * candidate is used directly (fallback to full replacement).
 *
 * Returns true if the blend (or fallback) is valid, false if both are invalid.
 */
static bool magneto_blend_BAinv(float out[4][3], float existing[4][3],
                                float candidate[4][3])
{
	float similarity = magneto_BAinv_similarity(existing, candidate);

	// Compute adaptive blending weight
	float alpha;
	if (similarity >= ONLINE_BLEND_SIMILARITY_HIGH) {
		// Very similar: candidate is just a minor refinement → low alpha
		alpha = ONLINE_BLEND_MIN_ALPHA;
	} else if (similarity <= ONLINE_BLEND_SIMILARITY_LOW) {
		// Significant divergence (possible environment change) → high alpha
		alpha = ONLINE_BLEND_MAX_ALPHA;
	} else {
		// Linear interpolation between low and high thresholds
		float t = (similarity - ONLINE_BLEND_SIMILARITY_LOW)
		        / (ONLINE_BLEND_SIMILARITY_HIGH - ONLINE_BLEND_SIMILARITY_LOW);
		alpha = ONLINE_BLEND_MAX_ALPHA
		      + t * (ONLINE_BLEND_MIN_ALPHA - ONLINE_BLEND_MAX_ALPHA);
	}

	// Blend
	float blended[4][3];
	float one_minus_alpha = 1.0f - alpha;
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 3; c++) {
			blended[r][c] = one_minus_alpha * existing[r][c]
			              + alpha * candidate[r][c];
		}
	}

	// Validate blended result
	float zero[3] = {0};
	float diagonal[3];
	for (int i = 0; i < 3; i++) {
		diagonal[i] = blended[i + 1][i];
	}
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	float max_gain = MAX(MAX(fabsf(diagonal[0]), fabsf(diagonal[1])), fabsf(diagonal[2]));
	if (v_epsilon(blended[0], zero, 1)
	    && v_epsilon(diagonal, average, MAX(magnitude * 0.2f, 0.1f))
	    && max_gain <= MAG_CAL_MAX_AXIS_GAIN) {
		// Blended result is valid
		memcpy(out, blended, sizeof(blended));
		return true;
	}

	// Fallback: use candidate directly if blend is invalid.
	// Candidate was already validated by magneto_quality_check before this call,
	// so validate with its own diagonal here for consistency.
	{
		float c_diag[3];
		for (int i = 0; i < 3; i++) {
			c_diag[i] = candidate[i + 1][i];
		}
		float c_avg = v_avg(c_diag);
		float c_avg_arr[3] = {c_avg, c_avg, c_avg};
		float c_max_gain = MAX(MAX(fabsf(c_diag[0]), fabsf(c_diag[1])), fabsf(c_diag[2]));
		if (v_epsilon(candidate[0], zero, 1)
		    && v_epsilon(c_diag, c_avg_arr, MAX(c_avg * 0.2f, 0.1f))
		    && c_max_gain <= MAG_CAL_MAX_AXIS_GAIN) {
			memcpy(out, candidate, sizeof(float) * 4 * 3);
			return true;
		}
	}

	// Both blend and fallback invalid — should not happen since candidate was
	// already validated by magneto_quality_check before calling this function
	return false;
}


// Phase 2: Background online magnetometer calibration
// Called from sensor loop for each new raw mag sample during normal operation.
// Gated by: VQF disturbance detection, accel magnitude, time interval, and direction change.
void sensor_calibration_online_mag_sample(const float m[3])
{
	if (!sensor_calibration_get_online_mag_enabled()) {
		return;
	}

	// Don't accumulate during manual calibration
	if (magneto_progress & 0x80) {
		return;
	}

	int64_t now = k_uptime_get();

	// Track fusion mag-disturbance duration (before any sample gates) so the
	// background check function knows how long disturbance has persisted.
	if (sensor_fusion_get_mag_dist_detected()) {
		if (online_mag_dist_start_time == 0) {
			online_mag_dist_start_time = now;
		}
	} else {
		online_mag_dist_start_time = 0;
	}

	// Suppress collection after buffer resets (wake-up, reboot, environment
	// change, calibration update) to let sensor data stabilise.
	if (now < online_collection_suppress_until) {
		return;
	}

	// Reject if fusion detects magnetic disturbance (only when we have an existing
	// calibration — fusion only receives mag data when calibrated, so mag_dist_detected
	// is meaningless without calibration).
	// Exception 1: if current calibration quality is bad (norm CV > 6%), the disturbance
	// detection itself may be unreliable due to the bad calibration, so skip the gate.
	// Exception 2: if fusion has been reporting disturbance continuously for a long time,
	// the "disturbance" is likely a calibration drift or environment change rather than
	// transient interference.  Allow samples through to enable recalibration.
	// Without this, a deadlock occurs: disturbance → gate blocks samples →
	// cal_norm_count stops updating (guarded by !magDistDetected in sensor.c) → CV
	// stays frozen at a low value → gate never opens → no recalibration possible.
	{
		float zero[3] = {0};
		bool has_cal = (v_diff_mag(magBAinv[0], zero) != 0);
		float current_cv = sensor_calibration_get_mag_quality();
		if (has_cal && current_cv < 0.06f && sensor_fusion_get_mag_dist_detected()) {
			// Sustained disturbance override: if disturbance has persisted for
			// more than 5 seconds, allow samples through.
			bool sustained = (online_mag_dist_start_time > 0 &&
			                  (now - online_mag_dist_start_time) > 5000);
			if (!sustained) {
				return;
			}
		}
	}

	// Rate limit: minimum interval between samples
	if (now - online_last_sample_time < ONLINE_MIN_INTERVAL_MS) {
		return;
	}

	// Gate by accel magnitude: reject samples under strong linear acceleration
	float accel_mag_sq = aBuf[0] * aBuf[0] + aBuf[1] * aBuf[1] + aBuf[2] * aBuf[2];
	if (accel_mag_sq < MAG_CAL_ACCEL_MAG_MIN_SQ || accel_mag_sq > MAG_CAL_ACCEL_MAG_MAX_SQ) {
		return;
	}

	// Direction diversity gate: accept sample if either mag direction OR
	// accelerometer (gravity) direction has changed since last accepted sample.
	// Pure magnetometer-based direction check can suffer from "direction lock-in"
	// during strong magnetic interference: the mag reading points to a distorted
	// but stable direction while the tracker physically rotates.  The accelerometer
	// cross-check breaks this deadlock — if the device has physically moved
	// (accel direction changed), accept the sample regardless of mag direction.
	float raw_mag[3] = {m[0], m[1], m[2]};
	float cur_dir[3];
	if (magneto_norm_sq(raw_mag) < 1e-8f) {
		return;
	}
	magneto_center_update(&online_center_estimator, raw_mag);
	if (!magneto_centered_direction(&online_center_estimator, raw_mag, cur_dir)) {
		return;
	}

	// Normalize accelerometer to get gravity direction
	// aBuf magnitude already validated (~1g) by the accel gate above
	float accel_norm = sqrtf(accel_mag_sq);
	float accel_inv = 1.0f / accel_norm;
	float cur_accel_dir[3] = {aBuf[0] * accel_inv, aBuf[1] * accel_inv, aBuf[2] * accel_inv};

	if (online_total_sample_count > 0) {
		float mag_dot = cur_dir[0] * online_last_dir[0]
		              + cur_dir[1] * online_last_dir[1]
		              + cur_dir[2] * online_last_dir[2];
		float accel_dot = cur_accel_dir[0] * online_last_accel_dir[0]
		                + cur_accel_dir[1] * online_last_accel_dir[1]
		                + cur_accel_dir[2] * online_last_accel_dir[2];

		float min_change = magneto_online_min_dir_change_threshold();
		bool mag_changed = (1.0f - mag_dot >= min_change);
		bool accel_changed = (1.0f - accel_dot >= min_change);

		if (!mag_changed && !accel_changed) {
			return; // neither mag nor accel direction changed enough
		}
	}

	online_last_sample_time = now;
	online_last_dir[0] = cur_dir[0];
	online_last_dir[1] = cur_dir[1];
	online_last_dir[2] = cur_dir[2];
	online_last_accel_dir[0] = cur_accel_dir[0];
	online_last_accel_dir[1] = cur_accel_dir[1];
	online_last_accel_dir[2] = cur_accel_dir[2];

	float route_mag[3];
	magneto_coverage_sample(&online_center_estimator, raw_mag, route_mag);
	if (magneto_norm_sq(route_mag) < 1e-8f) {
		memcpy(route_mag, raw_mag, sizeof(route_mag));
	}

	// Route sample to its octant based on sign relative to the min/max center.
	// This guarantees each octant independently rolls its ring buffer,
	// preventing a single orientation from evicting diverse data in other octants.
	int octant = 0;
	if (route_mag[0] < 0) octant |= 1;
	if (route_mag[1] < 0) octant |= 2;
	if (route_mag[2] < 0) octant |= 4;

	quadrant_buf_t *qbuf = &quad_buf[octant];
	online_total_sample_count++;
	qbuf->last_seq = online_total_sample_count;
	qbuf->samples[qbuf->head].x = m[0];
	qbuf->samples[qbuf->head].y = m[1];
	qbuf->samples[qbuf->head].z = m[2];
	qbuf->head = (qbuf->head + 1) % QUADRANT_BUF_SIZE;
	if (qbuf->count < QUADRANT_BUF_SIZE) {
		qbuf->count++;
	}
}

bool sensor_calibration_online_mag_check(void)
{
	if (!sensor_calibration_get_online_mag_enabled()) {
		return false;
	}

	int recent_sample_count_now = magneto_online_recent_sample_count();
	int64_t now = k_uptime_get();

	if (recent_sample_count_now < MAG_CAL_MIN_SAMPLES) {
		return false;
	}
	if (online_total_sample_count == online_last_checked_sample_count) {
		return false;
	}
	if (online_last_check_time > 0 &&
	    (now - online_last_check_time) < ONLINE_MIN_CHECK_INTERVAL_MS) {
		return false;
	}

	online_last_checked_sample_count = online_total_sample_count;
	online_last_check_time = now;

	float zero[3] = {0};
	bool has_existing = (v_diff_mag(magBAinv[0], zero) != 0);
	float current_cv = has_existing ? sensor_calibration_get_mag_quality() : 1.0f;

	// When fusion has a reliable calibration (CV < 4%) and is NOT experiencing
	// sustained magnetic disturbance, skip the calibration check entirely.
	// Updating calibration resets the fusion mag reference, causing ~6s of
	// heading instability.  Only attempt a recalibration when fusion has been
	// detecting disturbance for >3 seconds, indicating the current calibration
	// is genuinely insufficient.
	if (has_existing && current_cv < 0.04f) {
		if (online_mag_dist_start_time == 0) {
			return false; // fusion not disturbed — current cal is fine
		}
		int64_t dist_duration_ms = now - online_mag_dist_start_time;
		if (dist_duration_ms < ONLINE_VQF_DIST_MIN_DURATION_MS) {
			return false; // transient disturbance — wait
		}
	}

	// If the current calibration is already good enough AND we've had enough
	// updates to trust that assessment, skip the heavy Magneto fit.
	//
	// Two-tier convergence:
	//   Tier 1 (strict): CV is good AND directional coverage is adequate
	//     → require low dir_bias (sphere sampled evenly)
	//   Tier 2 (relaxed): CV is excellent AND we've done many updates
	//     → trust the fit regardless of dir_bias (directional fluctuations
	//       during normal rotation are just sampling noise, not real problems)
	//
	// Exception: skip convergence checks when fusion is experiencing sustained
	// magnetic disturbance.  The CV value is frozen during disturbance (norm
	// tracking gated by !magDistDetected in sensor.c), so a low frozen CV
	// does NOT mean the calibration is still good — the environment may have
	// changed.  If the code reached here past the CV < 4% disturbance gate
	// above, the disturbance is sustained and recalibration should proceed.
	bool mag_sustained_dist = (online_mag_dist_start_time > 0 &&
	                           (now - online_mag_dist_start_time) > ONLINE_VQF_DIST_MIN_DURATION_MS);
	if (has_existing && current_cv < CAL_NORM_GOOD_CV && online_update_count >= ONLINE_MIN_UPDATES
	    && !mag_sustained_dist) {
		// Tier 2: Excellent fit + sufficient history — lock it in.
		// CV < 0.035 and 3+ updates mean the calibration has reliably
		// converged.  Further updates would only add noise.
		if (current_cv < 0.035f && online_update_count >= 3) {
			LOG_INF("Online mag cal: converged (cv=%.3f, %d updates)",
			        (double)current_cv, online_update_count);
			return false;
		}

		float dir_bias_check = magneto_online_recent_dir_bias();

		// Tier 1: Good fit with adequate directional coverage
		if (dir_bias_check < 0.10f) {
			LOG_INF("Online mag cal: skipping (cv=%.3f < %.3f, dir_bias=%.3f, %d updates)",
			        (double)current_cv, (double)CAL_NORM_GOOD_CV,
			        (double)dir_bias_check, online_update_count);
			return false;
		}
		// Directional bias still too high: buffer samples are clustered.
		// Fall through to run calibration even though CV looks good.
		LOG_INF("Online mag cal: CV ok but dir_bias=%.3f >= 0.10, continuing",
		        (double)dir_bias_check);
	}

	double ata_recent[100];
	double recent_norm_sum;
	float recent_dir_sum[3];
	float recent_raw_range;
	double recent_sample_count = magneto_online_collect_recent(ata_recent, &recent_norm_sum,
	                                                           recent_dir_sum, &recent_raw_range);
	if (recent_sample_count < MAG_CAL_MIN_SAMPLES) {
		return false;
	}
	if (recent_raw_range < MAG_CAL_MIN_RAW_AXIS_RANGE) {
		LOG_INF("Online mag cal: need more rotation (raw_range=%.3f < %.3f, %d recent samples)",
		        (double)recent_raw_range, (double)MAG_CAL_MIN_RAW_AXIS_RANGE,
		        (int)recent_sample_count);
		return false;
	}

	// Detect magnetic environment changes by comparing the buffer's
	// average raw field strength against the last update's reference.
	// When the norm changes by >25% (e.g., moving between a desk and
	// a high-interference area), clear buffers to prevent mixed-data
	// fits.  Direction is preserved — only scale changes.
	float buf_avg_norm = (float)(recent_norm_sum / recent_sample_count);

	if (has_existing && online_update_count > 0 && online_last_buf_avg_norm > 0.0f) {
		float norm_ratio = buf_avg_norm / online_last_buf_avg_norm;
		if (norm_ratio > 1.25f || norm_ratio < 0.80f) {
			LOG_WRN("Online mag cal: env change detected (buf norm %.3f -> %.3f, ratio %.2f), "
			        "resetting buffers",
			        (double)online_last_buf_avg_norm, (double)buf_avg_norm, (double)norm_ratio);
			magneto_online_clear_history();
			online_update_count = 0;
			online_last_buf_avg_norm = 0.0f;
			return false;
		}
	}

	float dbias = magneto_directional_bias(recent_dir_sum, recent_sample_count);
	LOG_INF("Online mag cal: coverage raw_range=%.3f, dir_bias=%.3f, n=%d",
	        (double)recent_raw_range, (double)dbias, (int)recent_sample_count);

	// Quality check: directional diversity + validation + compute calibration
	float m_inv[4][3];
	if (!magneto_quality_check(ata_recent, recent_norm_sum, recent_sample_count, m_inv)) {
		LOG_INF("Online mag cal: check failed (%d recent samples, dir_bias=%.3f)",
		        (int)recent_sample_count, (double)dbias);
		return false;
	}
	if (!sensor_calibration_get_online_mag_enabled()) {
		LOG_INF("Online mag cal: disabled before apply, skipping update");
		return false;
	}

	if (has_existing) {
		// Enforce minimum cooldown between updates to avoid frequent VQF mag ref resets.
		// Each update resets VQF's heading reference, causing ~6s of re-establishment.
		if (online_last_update_time > 0 &&
		    (now - online_last_update_time) < (ONLINE_MIN_UPDATE_INTERVAL_S * 1000LL)) {
			return false;
		}

		// Blend trial calibration with existing using EMA.
		// Blending weight is similarity-adaptive: more similar → conservative,
		// more divergent → faster adaptation (possible environment change).
		float blended[4][3];
		if (!magneto_blend_BAinv(blended, magBAinv, m_inv)) {
			LOG_WRN("Online mag cal: blend validation failed, skipping update");
			return false;
		}

		float similarity = magneto_BAinv_similarity(magBAinv, m_inv);

		// Reject candidate if similarity is below threshold — the data
		// is too inconsistent for a meaningful fit.  This guards against
		// mixed-data fits (e.g. when the tracker moves between magnetic
		// environments and the quadrant buffer holds samples from both old
		// and new locations).
		// Rather than blindly trusting a poor fit, let the buffer age out
		// stale samples; the next cycle will fit a consistent dataset with
		// much higher similarity.
		//
		// Exception: during the first ONLINE_MIN_UPDATES cycles we accept
		// even low-sim fits to establish an initial baseline (especially
		// important when booting with a stale NVS calibration).
		if (similarity < 0.85f) {
			if (online_update_count < ONLINE_MIN_UPDATES) {
				LOG_INF("Online mag cal: low sim=%.3f accepted (early bootstrap #%d)",
				        (double)similarity, online_update_count + 1);
			} else if (current_cv < CAL_NORM_GOOD_CV) {
				LOG_WRN("Online mag cal: rejecting candidate (sim=%.3f < 0.85, "
				        "current cv=%.3f is good — possible mixed data)",
				        (double)similarity, (double)current_cv);
				return false;
			} else {
				LOG_WRN("Online mag cal: rejecting candidate (sim=%.3f < 0.85, "
				        "current cv=%.3f — incomplete/dirty buffer?)",
				        (double)similarity, (double)current_cv);
				return false;
			}
		}

		LOG_INF("Online mag cal: blended (#%d, %d samples, dir_bias=%.3f, cur_cv=%.3f, sim=%.3f)",
		        online_update_count + 1,
		        (int)recent_sample_count, (double)dbias, (double)current_cv,
		        (double)similarity);
		memcpy(magBAinv, blended, sizeof(magBAinv));
		memcpy(m_inv, blended, sizeof(m_inv)); // for logging below

		// Start the next cycle from a clean buffer. Keeping pre-update samples
		// around can immediately re-mix old field conditions into the next fit.
		magneto_online_clear_history();
		online_last_update_time = now;
		online_update_count++;
		online_last_buf_avg_norm = buf_avg_norm;

		// Reset fusion mag reference so it re-establishes with the refined calibration
		sensor_fusion_reset_mag_ref();
		sensor_mag_ref_reset();
	} else {
		LOG_INF("Online mag cal: first calibration (%d recent samples, dir_bias=%.3f)",
		        (int)recent_sample_count, (double)dbias);

		// First calibration: use candidate directly
		memcpy(magBAinv, m_inv, sizeof(magBAinv));
		magneto_online_clear_history();
		online_last_update_time = now;
		online_update_count = 1;
		online_last_buf_avg_norm = buf_avg_norm;

		// Reset fusion mag reference so it re-establishes with the new calibration
		sensor_fusion_reset_mag_ref();
		sensor_mag_ref_reset();
	}

	// Reset norm tracking after calibration change
	cal_norm_count = 0;
	cal_norm_ema = 0;
	cal_norm_var_ema = 0;

	sys_write(MAIN_MAG_BIAS_ID, &retained->magBAinv, magBAinv, sizeof(magBAinv));
	sensor_refresh_sensor_ids();

	LOG_INF("Online mag cal applied:");
	for (int i = 0; i < 3; i++) {
		LOG_INF("%.5f %.5f %.5f %.5f",
			(double)m_inv[0][i], (double)m_inv[1][i],
			(double)m_inv[2][i], (double)m_inv[3][i]);
	}

	return true;
}

int sensor_calibration_online_mag_status(float *dir_bias)
{
	if (!sensor_calibration_get_online_mag_enabled()) {
		if (dir_bias) {
			*dir_bias = 1.0f;
		}
		return 0;
	}
	if (dir_bias) {
		*dir_bias = magneto_online_recent_dir_bias();
	}
	return magneto_online_recent_sample_count();
}

// Feed calibrated mag norm for runtime quality tracking.
// Called from sensor.c after applying BAinv calibration.
void sensor_calibration_track_mag_norm(float cal_norm)
{
	if (!sensor_calibration_get_online_mag_enabled()) {
		return;
	}
	if (cal_norm < 1e-6f) {
		return;
	}
	if (cal_norm_count == 0) {
		cal_norm_ema = cal_norm;
		cal_norm_var_ema = 0;
	} else {
		float diff = cal_norm - cal_norm_ema;
		cal_norm_ema += CAL_NORM_EMA_ALPHA * diff;
		cal_norm_var_ema += CAL_NORM_EMA_ALPHA * (diff * diff - cal_norm_var_ema);
	}
	cal_norm_count++;
}

// Get current calibration quality: returns norm CV (std/mean).
// Lower is better. Returns 1.0 if insufficient data.
float sensor_calibration_get_mag_quality(void)
{
	if (cal_norm_count < 100 || cal_norm_ema < 1e-6f) {
		return 1.0f;
	}
	float std = sqrtf(cal_norm_var_ema);
	return std / cal_norm_ema;
}

