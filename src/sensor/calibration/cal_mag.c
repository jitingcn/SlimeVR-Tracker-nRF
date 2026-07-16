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
#include "system/watchdog.h"
#include "util.h"

#include <math.h>
#include <string.h>

#if CONFIG_CMSIS_DSP
#include <arm_math.h>
#endif

#include "sensor/magneto/magneto1_4.h"

#include "bias_collect.h"
#include "cal_mag.h"
#include "cal_sample.h"
#include "calibration.h"
#include "mag_common.h"
#include "online_mag.h"

LOG_MODULE_REGISTER(cal_mag, LOG_LEVEL_INF);

/* Owned here; declared in mag_common.h for online_mag / calibration glue. */
uint8_t magneto_progress;
static uint8_t last_magneto_progress;
static int64_t magneto_progress_time;

static int64_t mag_cal_last_status_log;

double ata[100]; /* manual / 6-side Magneto accumulator */
double norm_sum;
double sample_count;

/* Direction range tracking for manual calibration: per-axis min/max of normalized direction */
static float dir_min[3];
static float dir_max[3];

static mag_center_estimator_t manual_center_estimator;

/* Manual calibration direction tracking (same cross-validation logic as online path) */
static float manual_last_dir[3];
static float manual_last_accel_dir[3];

/* Biases / matrices owned by calibration.c */
extern float magBAinv[4][3];

static void magneto_update_dir_range(const float v[3]);
static float magneto_min_dir_range(void);
static void sensor_sample_mag_magneto_sample(const float m[3]);

int sensor_calibrate_mag(void)
{
	float zero[3] = {0};
	if (v_diff_mag(magBAinv[0], zero) != 0) {
		magneto_reset();
		if (get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
			set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
		}
		return -1; // magnetometer calibration already exists
	}

	if (!get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
		set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
	}

	float m[3];
	if (sensor_wait_mag(m, K_MSEC(1000))) {
		return -1; // Timeout
	}
	sensor_sample_mag_magneto_sample(m); // 400us

	// Periodic status log every 1 second
	int64_t now = k_uptime_get();
	if (now - mag_cal_last_status_log >= 1000) {
		mag_cal_last_status_log = now;
		LOG_INF("Mag cal: %d samples collected", (int)sample_count);
	}

	if (magneto_progress != 0b11111111) {
		return 1; // Still collecting - signal caller to use short sleep
	}

	float m_inv[4][3];
	LOG_INF("Calibrating magnetometer hard/soft iron offset");

	// max allocated 1072 bytes
#if DEBUG
	printk("ata:\n");
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			printk("%7.2f, ", (double)ata[i * 10 + j]);
		}
		printk("\n");
		k_msleep(3);
	}
	printk("norm_sum: %.2f, sample_count: %.0f\n", norm_sum, sample_count);
#endif
	wait_for_threads();
	magneto_current_calibration(m_inv, ata, norm_sum, sample_count); // 25ms
	magneto_reset();

	LOG_INF("Magnetometer matrix:");
	for (int i = 0; i < 3; i++) {
		LOG_INF(
			"%.5f %.5f %.5f %.5f",
			(double)m_inv[0][i],
			(double)m_inv[1][i],
			(double)m_inv[2][i],
			(double)m_inv[3][i]
		);
	}
	if (sensor_calibration_validate_mag(m_inv, false)) {
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		LOG_INF("Restoring previous calibration");
		LOG_INF("Magnetometer matrix:");
		for (int i = 0; i < 3; i++) {
			LOG_INF(
				"%.5f %.5f %.5f %.5f",
				(double)magBAinv[0][i],
				(double)magBAinv[1][i],
				(double)magBAinv[2][i],
				(double)magBAinv[3][i]
			);
		}
		sensor_calibration_validate_mag(NULL, true); // additionally verify old calibration
		if (get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
			set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
		}
		return -1;
	} else {
		LOG_INF("Applying calibration");
		memcpy(magBAinv, m_inv, sizeof(magBAinv));
		magneto_online_runtime_reset(); // Restart online calibration from a clean baseline
		sensor_fusion_reset_mag_ref();
		sensor_mag_ref_reset(); // Recompute magRef from new calibration
								// fusion invalidation not necessary
	}
	sys_write(MAIN_MAG_BIAS_ID, &retained->magBAinv, magBAinv, sizeof(magBAinv));

	LOG_INF("Finished calibration");
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
	sensor_refresh_sensor_ids(); // Refresh reported mag status after calibration
	if (get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
		set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
	}
	return 0;
}

void magneto_reset(void)
{
	magneto_progress = 0;
	last_magneto_progress = 0;
	magneto_progress_time = 0;
	mag_cal_last_status_log = 0;
	memset(ata, 0, sizeof(ata));
	norm_sum = 0;
	sample_count = 0;
	for (int i = 0; i < 3; i++) {
		dir_min[i] = 2.0f;  // start high
		dir_max[i] = -2.0f; // start low
	}
	magneto_center_reset(&manual_center_estimator);
	memset(manual_last_dir, 0, sizeof(manual_last_dir));
	memset(manual_last_accel_dir, 0, sizeof(manual_last_accel_dir));
}

/**
 * Update direction range tracking (min/max per axis of normalized direction).
 * Used only by manual calibration to ensure sufficient multi-axis rotation.
 */
static void magneto_update_dir_range(const float v[3])
{
	float d[3];
	if (!magneto_normalize_direction(v, d)) {
		return;
	}
	for (int i = 0; i < 3; i++) {
		if (d[i] < dir_min[i]) {
			dir_min[i] = d[i];
		}
		if (d[i] > dir_max[i]) {
			dir_max[i] = d[i];
		}
	}
}

/**
 * Get minimum direction range across all three axes.
 * Range = max - min of normalized direction per axis.
 * Higher = more diverse coverage.
 */
static float magneto_min_dir_range(void)
{
	float min_range = 2.0f;
	for (int i = 0; i < 3; i++) {
		float range = dir_max[i] - dir_min[i];
		if (range < min_range) {
			min_range = range;
		}
	}
	return min_range;
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
// Target number of samples, 6 faces + 12 edges
#define CALIB_TARGET_SAMPLES 18
// Orientation difference threshold (cosine value).
// cos(25 degrees) ≈ 0.90. If dot product > 0.90, the angle between two directions is less than 25 degrees, considered
// duplicate.
#define MIN_ORIENTATION_DIFF_COS 0.90f
// Acceleration threshold for determining stationary state
#define THRESHOLD_ACC 0.02f
// Number of samples to collect for each orientation
#define SAMPLES_PER_ORIENTATION 500
// Timeout for waiting for new pose (30 seconds in milliseconds)
#define CALIB_POSE_TIMEOUT_MS 30000
typedef struct {
	float x, y, z;
} Vector3;

int sensor_6_sideBias(float a_inv[][3], int *captured_count_out)
{
	float rawData[3];
	float pre_acc[3] = {0};
	int resttime = 0;

	Vector3 captured_dirs[CALIB_TARGET_SAMPLES];
	int captured_count = 0;
	int64_t last_new_pose_time = k_uptime_get(); // Track time of last new pose

	// Initialize output parameter
	if (captured_count_out) {
		*captured_count_out = 0;
	}

	magneto_reset();

	LOG_INF("Starting Multi-Position Calibration (Target: %d poses)", CALIB_TARGET_SAMPLES);
	LOG_INF("Please rotate device to random orientations and hold still.");

	// Main loop: until target number of samples collected
	while (captured_count < CALIB_TARGET_SAMPLES) {

		// 1. Wait for device to be stationary
		set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_SENSOR); // Indicate searching for stationary state
		bool pose_timeout = false;
		while (1) {
			/* Feed watchdog during user interaction wait */
			watchdog_feed(WDT_CHANNEL_CALIBRATION);

			/* Check for timeout - no new pose in CALIB_POSE_TIMEOUT_MS */
			if ((k_uptime_get() - last_new_pose_time) > CALIB_POSE_TIMEOUT_MS) {
				LOG_WRN("Timeout: No new pose detected for %d seconds", CALIB_POSE_TIMEOUT_MS / 1000);
				pose_timeout = true;
				break;
			}

			if (sensor_wait_accel(rawData, K_MSEC(1000))) {
				return -2; // Timeout, magneto state not handled here
			}

			int rest = isAccRest(rawData, pre_acc, THRESHOLD_ACC, &resttime, 100);
			memcpy(pre_acc, rawData, sizeof(rawData)); // Update previous data

			if (rest == 1) {
				// Device is stationary, now check if this pose is new
				// Calculate current vector magnitude
#if CONFIG_CMSIS_DSP
				float norm_sq;
				arm_dot_prod_f32(rawData, rawData, 3, &norm_sq);
				float norm;
				arm_sqrt_f32(norm_sq, &norm);
#else
				float norm = sqrtf(rawData[0] * rawData[0] + rawData[1] * rawData[1] + rawData[2] * rawData[2]);
#endif
				if (norm < 0.1f) {
					continue; // Prevent division by zero (unlikely under gravity)
				}

				// Normalize current vector
				float curr_dir_x = rawData[0] / norm;
				float curr_dir_y = rawData[1] / norm;
				float curr_dir_z = rawData[2] / norm;

				bool is_duplicate = false;
				// Compare with historical data
				for (int i = 0; i < captured_count; i++) {
					// Calculate cosine of angle using dot product
					float dot = curr_dir_x * captured_dirs[i].x + curr_dir_y * captured_dirs[i].y
							  + curr_dir_z * captured_dirs[i].z;

					// If dot product is close to 1, the directions are almost the same,
					// or a slight wobble of a previous direction
					if (dot > MIN_ORIENTATION_DIFF_COS) {
						is_duplicate = true;
						break;
					}
				}

				if (!is_duplicate) {
					// This is a new valid pose! Break the wait loop and start capturing data
					// Save this direction vector
					captured_dirs[captured_count].x = curr_dir_x;
					captured_dirs[captured_count].y = curr_dir_y;
					captured_dirs[captured_count].z = curr_dir_z;
					break;
				} else {
					// Duplicate pose detected, briefly flash LED to prompt user to change orientation, but do not error
					set_led(SYS_LED_PATTERN_FLASH, SYS_LED_PRIORITY_SENSOR);
					k_msleep(100); // Debounce slightly
				}
			}
			k_msleep(20);
		}

		// Check if we timed out waiting for a new pose
		if (pose_timeout) {
			if (captured_count_out) {
				*captured_count_out = captured_count;
			}
			// Return -3 for timeout with captured_count available for partial save decision
			return -3;
		}

		// Reset timeout counter when a new valid pose is found
		last_new_pose_time = k_uptime_get();

		LOG_INF("Capturing pose %d/%d...", captured_count + 1, CALIB_TARGET_SAMPLES);
		set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_SENSOR);

		int sample_idx = 0;
		while (sample_idx < SAMPLES_PER_ORIENTATION) {
			if (sensor_wait_accel(rawData, K_MSEC(1000))) {
				return -2;
			}

			if (!v_epsilon(rawData, pre_acc, 0.03f)) {
				LOG_INF("Motion detected during capture, retrying...");
				sample_idx = -1;
				break;
			}
			memcpy(pre_acc, rawData, sizeof(rawData));

			magneto_sample(rawData[0], rawData[1], rawData[2], ata, &norm_sum, &sample_count);

			sample_idx++;

			if (sample_idx % 50 == 0) {
				printk(".");
				/* Feed watchdog periodically during sampling */
				watchdog_feed(WDT_CHANNEL_CALIBRATION);
			}
		}

		if (sample_idx == -1) {
			continue;
		}

		captured_count++;
		set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
		LOG_INF("Pose %d saved!", captured_count);

		k_msleep(500);
	}

	if (captured_count_out) {
		*captured_count_out = captured_count;
	}

	LOG_INF("Calculating calibration matrix...");

	wait_for_threads();
	magneto_current_calibration(a_inv, ata, norm_sum, sample_count);

	magneto_reset();

	LOG_INF("Calibration calculation complete.");
	return 0;
}
#endif

// Collect magnetometer sample for manual calibration.
// Accumulates into ATA and periodically runs trial calibration for quality check.
static void sensor_sample_mag_magneto_sample(const float m[3])
{
	// Direction diversity gate: accept sample if either mag direction OR
	// accelerometer (gravity) direction has changed since last accepted.
	// Identical logic to the online path — breaks direction lock-in during
	// magnetic interference by cross-validating with physical orientation.
	//
	// Gate accelerometer by magnitude as well: skip the direction check
	// entirely when the device is under strong linear acceleration, falling
	// back to mag-only for that sample.
	float accel_mag_sq = aBuf[0] * aBuf[0] + aBuf[1] * aBuf[1] + aBuf[2] * aBuf[2];
	bool accel_trustworthy = (accel_mag_sq >= MAG_CAL_ACCEL_MAG_MIN_SQ && accel_mag_sq <= MAG_CAL_ACCEL_MAG_MAX_SQ);

	float raw_mag[3] = {m[0], m[1], m[2]};
	float cur_mag_dir[3];
	if (magneto_norm_sq(raw_mag) < 1e-8f) {
		return;
	}
	magneto_center_update(&manual_center_estimator, raw_mag);
	if (!magneto_centered_direction(&manual_center_estimator, raw_mag, cur_mag_dir)) {
		return;
	}

	// Normalize accelerometer (if trustworthy)
	float cur_accel_dir[3] = {0};
	if (accel_trustworthy) {
		float accel_inv = 1.0f / sqrtf(accel_mag_sq);
		cur_accel_dir[0] = aBuf[0] * accel_inv;
		cur_accel_dir[1] = aBuf[1] * accel_inv;
		cur_accel_dir[2] = aBuf[2] * accel_inv;
	}

	if (sample_count > 0) {
		float min_change = magneto_online_min_dir_change_threshold();

		// Mag direction change
		float mag_dot = cur_mag_dir[0] * manual_last_dir[0] + cur_mag_dir[1] * manual_last_dir[1]
					  + cur_mag_dir[2] * manual_last_dir[2];
		bool mag_changed = (1.0f - mag_dot >= min_change);

		// Accel direction change (only if accel is trustworthy)
		bool accel_changed = false;
		if (accel_trustworthy) {
			float accel_dot = cur_accel_dir[0] * manual_last_accel_dir[0] + cur_accel_dir[1] * manual_last_accel_dir[1]
							+ cur_accel_dir[2] * manual_last_accel_dir[2];
			accel_changed = (1.0f - accel_dot >= min_change);
		}

		// Accept only if at least one direction source shows movement
		if (!mag_changed && !accel_changed) {
			return; // redundant sample
		}
	}

	// Update last accepted directions
	manual_last_dir[0] = cur_mag_dir[0];
	manual_last_dir[1] = cur_mag_dir[1];
	manual_last_dir[2] = cur_mag_dir[2];
	if (accel_trustworthy) {
		manual_last_accel_dir[0] = cur_accel_dir[0];
		manual_last_accel_dir[1] = cur_accel_dir[1];
		manual_last_accel_dir[2] = cur_accel_dir[2];
	}

	// Accept sample - add to Magneto accumulator
	magneto_sample(m[0], m[1], m[2], ata, &norm_sum, &sample_count); // 400us
	float coverage_mag[3];
	magneto_coverage_sample(&manual_center_estimator, raw_mag, coverage_mag);
	magneto_update_dir_range(coverage_mag);

	// Attempt trial calibration every MAG_CAL_TRIAL_INTERVAL samples
	if (sample_count >= MAG_CAL_MIN_SAMPLES && (int)sample_count % MAG_CAL_TRIAL_INTERVAL < 1) {
		float min_range = magneto_min_dir_range();
		float raw_range = magneto_center_min_range(&manual_center_estimator);
		LOG_INF(
			"Mag cal check: %d samples, min_range=%.2f (need %.2f), raw_range=%.3f (need %.3f)",
			(int)sample_count,
			(double)min_range,
			(double)MAG_CAL_MIN_DIR_RANGE,
			(double)raw_range,
			(double)MAG_CAL_MIN_RAW_AXIS_RANGE
		);

		// Require minimum directional coverage before attempting calibration
		if (min_range < MAG_CAL_MIN_DIR_RANGE || raw_range < MAG_CAL_MIN_RAW_AXIS_RANGE) {
			LOG_INF("Mag cal: need more rotation, keep turning");
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
			return;
		}

		if (magneto_quality_check(ata, norm_sum, sample_count, NULL)) {
			magneto_progress |= 0b01111111;
			LOG_INF("Mag cal ready: %d samples, min_range=%.2f", (int)sample_count, (double)min_range);
			set_led(SYS_LED_PATTERN_FLASH, SYS_LED_PRIORITY_SENSOR);
		} else {
			LOG_INF("Mag cal: not ready yet, keep rotating (%d samples)", (int)sample_count);
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
		}
	}
}
