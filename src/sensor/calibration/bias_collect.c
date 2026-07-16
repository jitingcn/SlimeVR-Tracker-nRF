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
#include "system/watchdog.h"

#include <math.h>
#include <string.h>

#if CONFIG_CMSIS_DSP
#include <arm_math.h>
#endif

#include "bias_collect.h"
#include "cal_sample.h"

LOG_MODULE_REGISTER(cal_bias_collect, LOG_LEVEL_INF);

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
int isAccRest(float *acc, float *pre_acc, float threshold, int *t, int restdelta)
{
	float delta[3];
	delta[0] = acc[0] - pre_acc[0];
	delta[1] = acc[1] - pre_acc[1];
	delta[2] = acc[2] - pre_acc[2];

#if CONFIG_CMSIS_DSP
	float norm_sq;
	arm_dot_prod_f32(delta, delta, 3, &norm_sq);
	float norm_diff;
	arm_sqrt_f32(norm_sq, &norm_diff);
#else
	float norm_diff = sqrtf(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
#endif

	if (norm_diff <= threshold) {
		*t += restdelta;
	} else {
		*t = 0;
	}

	if (*t > 2000) {
		return 1;
	}
	return 0;
}
#endif

int sensor_offsetBias_internal(
	float *dest1,
	float *dest2,
	float *avg_temp,
	float *temp_range,
	int max_sample_time_ms,
	int min_sample_time_ms
)
{
	float rawData[3];
	float min_a[3], max_a[3];
	float min_g[3], max_g[3];

	// Initialize min/max with initial samples
	if (sensor_wait_accel(min_a, K_MSEC(1000))) {
		return -2; // Timeout
	}
	memcpy(max_a, min_a, sizeof(max_a));

	if (sensor_wait_gyro(min_g, K_MSEC(1000))) {
		return -2; // Timeout
	}
	memcpy(max_g, min_g, sizeof(max_g));

	double gyro_sum[3] = {0};

#if CONFIG_SENSOR_USE_TCAL
	double temp_sum = 0;
	float temp_min = INFINITY, temp_max = -INFINITY;
	float current_temp;
	int valid_temp_count = 0;

	// Record start temperature
	current_temp = sensor_get_current_imu_temperature();
	if (!isnan(current_temp) && current_temp > -20.0f && current_temp < 60.0f) {
		temp_min = current_temp;
		temp_max = current_temp;
	}
#endif

	int64_t sampling_start_time = k_uptime_get();
	int i = 0;
#if CONFIG_SENSOR_USE_TCAL
	bool temp_threshold_reached = false;
#endif

	// Accel motion check counter - check every N gyro samples to avoid blocking
	float actual_gyro_odr = sensor_get_gyro_odr();
	float actual_accel_odr = sensor_get_accel_odr();

	// Calculate effective ODRs after oversampling
	// sensor_wait_gyro receives data at effective_gyro_odr (after gyro oversampling)
	// sensor_wait_accel receives data at effective_accel_odr (after accel oversampling)
#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
	float effective_gyro_odr = actual_gyro_odr / CONFIG_SENSOR_GYRO_OVERSAMPLING;
#else
	float effective_gyro_odr = actual_gyro_odr;
#endif

#if CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1
	float effective_accel_odr = actual_accel_odr / CONFIG_SENSOR_ACCEL_OVERSAMPLING;
#else
	float effective_accel_odr = actual_accel_odr;
#endif

	int accel_check_interval = (int)(effective_gyro_odr / effective_accel_odr + 0.5f); // Round to nearest
	if (accel_check_interval < 1) {
		accel_check_interval = 1; // Ensure at least 1
	}
	int accel_check_counter = 0;

#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1 || CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1
	LOG_INF(
		"Calibration: ODR - Gyro: %.2fHz (eff: %.2fHz, %dx OS), Accel: %.2fHz (eff: %.2fHz, %dx OS), Check interval: "
		"%d",
		(double)actual_gyro_odr,
		(double)effective_gyro_odr,
		CONFIG_SENSOR_GYRO_OVERSAMPLING,
		(double)actual_accel_odr,
		(double)effective_accel_odr,
		CONFIG_SENSOR_ACCEL_OVERSAMPLING,
		accel_check_interval
	);
#else
	LOG_INF(
		"Calibration: Using actual ODR - Gyro: %.2fHz, Accel: %.2fHz, Check interval: %d",
		(double)actual_gyro_odr,
		(double)actual_accel_odr,
		accel_check_interval
	);
#endif

	// Collect samples with smart stop conditions
	// Main loop runs at gyro ODR, accel checked periodically
	int wdt_feed_counter = 0;
	while (true) {
		int64_t elapsed = k_uptime_get() - sampling_start_time;

		// Check stop conditions
		if (elapsed >= max_sample_time_ms) {
			LOG_INF("Max sampling time reached (%lld ms)", elapsed);
			break;
		}

		// Feed watchdog periodically during long sampling (~every 1 second based on gyro ODR)
		wdt_feed_counter++;
		if (wdt_feed_counter >= (int)effective_gyro_odr) {
			watchdog_feed(WDT_CHANNEL_CALIBRATION);
			wdt_feed_counter = 0;
		}

#if CONFIG_SENSOR_USE_TCAL
		// Check temperature threshold only after min_sample_time_ms
		if (elapsed >= min_sample_time_ms && temp_threshold_reached) {
			LOG_INF("Temperature threshold reached after %lld ms with %d samples", elapsed, i);
			break;
		}
#endif

		// Check accelerometer motion periodically (not every loop iteration)
		// This prevents the loop from being blocked by slower accel ODR
		if (accel_check_counter >= accel_check_interval) {
			if (sensor_wait_accel(rawData, K_MSEC(100))) {
				return -2; // Timeout
			}

			// Check Accel Motion (Min/Max method)
			for (int j = 0; j < 3; j++) {
				if (rawData[j] < min_a[j]) {
					min_a[j] = rawData[j];
				}
				if (rawData[j] > max_a[j]) {
					max_a[j] = rawData[j];
				}
				if (max_a[j] - min_a[j] > BIAS_COLLECT_ACCEL_MOTION_THRESHOLD) {
					LOG_INF("Accel motion detected: axis %d range %.4f", j, (double)(max_a[j] - min_a[j]));
					return -1;
				}
			}
			accel_check_counter = 0;
		}
		accel_check_counter++;

		// Accumulate Gyroscope
		if (sensor_wait_gyro(rawData, K_MSEC(100))) {
			return -2; // Timeout
		}

		// Check Gyro Motion
		for (int j = 0; j < 3; j++) {
			if (rawData[j] < min_g[j]) {
				min_g[j] = rawData[j];
			}
			if (rawData[j] > max_g[j]) {
				max_g[j] = rawData[j];
			}
			if (max_g[j] - min_g[j] > BIAS_COLLECT_GYRO_MOTION_THRESHOLD) {
				LOG_INF("Gyro motion detected: axis %d range %.4f", j, (double)(max_g[j] - min_g[j]));
				return -1;
			}
		}

		// Accumulate gyro data using online algorithm
		gyro_sum[0] += (double)rawData[0];
		gyro_sum[1] += (double)rawData[1];
		gyro_sum[2] += (double)rawData[2];

#if CONFIG_SENSOR_USE_TCAL
		// Sample and accumulate temperature
		current_temp = sensor_get_current_imu_temperature();
		if (!isnan(current_temp) && current_temp > -20.0f && current_temp < 60.0f) {
			temp_sum += (double)current_temp;
			valid_temp_count++;

			if (current_temp < temp_min) {
				temp_min = current_temp;
			}
			if (current_temp > temp_max) {
				temp_max = current_temp;
			}

			// Check if temperature range threshold exceeded
			if ((temp_max - temp_min) >= BIAS_COLLECT_TEMP_RANGE_THRESHOLD) {
				temp_threshold_reached = true;
			}
		}
#endif
		i++;
	}

	LOG_INF("Samples collected: %d", i);

	// Calculate minimum samples based on actual gyro ODR and min_sample_time_ms
	// Convert min_sample_time_ms to seconds for calculation
	float min_sample_time_sec = (float)min_sample_time_ms / 1000.0f;
#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
	// With oversampling enabled, the effective sample rate sent to calibration is reduced
	// by the oversampling factor (e.g., 1600Hz / 4 = 400Hz effective)
	int min_samples_required = (int)(effective_gyro_odr * min_sample_time_sec);
	LOG_INF(
		"Calibration: Gyro oversampling %dx, effective ODR: %.2fHz",
		CONFIG_SENSOR_GYRO_OVERSAMPLING,
		(double)effective_gyro_odr
	);
#else
	int min_samples_required = (int)(actual_gyro_odr * min_sample_time_sec);
#endif

	if (i < min_samples_required) {
		LOG_WRN(
			"Not enough samples: %d < %d (based on actual gyro ODR: %.2fHz, min time: %dms)",
			i,
			min_samples_required,
			(double)actual_gyro_odr,
			min_sample_time_ms
		);
		return -2;
	}

#if CONFIG_SENSOR_USE_TCAL
	if (avg_temp != NULL && valid_temp_count > 0) {
		*avg_temp = (float)(temp_sum / valid_temp_count);
		LOG_INF("T-Cal: Average temperature: %.2fC (%d samples)", (double)*avg_temp, valid_temp_count);
	}

	if (temp_range != NULL) {
		*temp_range = temp_max - temp_min;
		LOG_INF(
			"T-Cal: Temperature range: %.2fC (%.2fC to %.2fC)",
			(double)*temp_range,
			(double)temp_min,
			(double)temp_max
		);
	}
#endif

	// Calculate averages
	dest2[0] = (float)(gyro_sum[0] / i);
	dest2[1] = (float)(gyro_sum[1] / i);
	dest2[2] = (float)(gyro_sum[2] / i);

#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	// In single-side calibration mode, do NOT calculate accelerometer bias.
	dest1[0] = 0.0f;
	dest1[1] = 0.0f;
	dest1[2] = 0.0f;
#endif

	return 0;
}

/**
 * Standard sensor offset bias collection function
 * Uses default timing: BIAS_COLLECT_MAX_SAMPLE_TIME_MS max, BIAS_COLLECT_MIN_SAMPLE_TIME_MS min
 */
int sensor_offsetBias(float *dest1, float *dest2, float *avg_temp, float *temp_range)
{
	return sensor_offsetBias_internal(
		dest1,
		dest2,
		avg_temp,
		temp_range,
		BIAS_COLLECT_MAX_SAMPLE_TIME_MS,
		BIAS_COLLECT_MIN_SAMPLE_TIME_MS
	);
}
