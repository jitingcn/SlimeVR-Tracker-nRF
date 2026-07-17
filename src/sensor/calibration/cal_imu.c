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

#include "sensor/sensors_enum.h"
#include "sensor/magneto/magneto1_4.h"
#if IS_ENABLED(CONFIG_SENSOR_DRV_BMI270)
#include "sensor/imu/BMI270.h"
#endif

#include "bias_collect.h"
#include "cal_imu.h"
#include "cal_mag.h"
#include "cal_sample.h"
#include "calibration.h"
#if CONFIG_SENSOR_USE_TCAL
#include "tcal_mls_lut.h"
#include "tcal_runtime.h"
#endif

LOG_MODULE_REGISTER(cal_imu, LOG_LEVEL_INF);

/* Owned by calibration.c */
extern float accelBias[3];
extern float gyroBias[3];
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
extern float accBAinv[4][3];
#endif

void sensor_calibrate_imu(void)
{
	float a_bias[3], g_bias[3];
	LOG_INF("Calibrating main accelerometer and gyroscope zero rate offset");
	LOG_INF("Rest the device on a stable surface");

	set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_SENSOR);
	if (!wait_for_motion(false, 6)) // Wait for accelerometer to settle, timeout 3s
	{
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return; // Timeout, calibration failed
	}

	set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_SENSOR);
	k_msleep(500); // Delay before beginning acquisition

#if CONFIG_SENSOR_USE_TCAL
	// Variables to store average temperature and temperature range from calibration
	float avg_temp = NAN;
	float temp_range = NAN;
#endif

#if IS_ENABLED(CONFIG_SENSOR_DRV_BMI270)
	if (sensor_calibration_get_imu_id() == IMU_BMI270) // bmi270 specific
	{
		uint8_t *sensor_data = sensor_calibration_get_sensor_data();
		LOG_INF("Suspending sensor thread");
		main_imu_suspend();
		LOG_INF("Running BMI270 component retrimming");
		int err = bmi_crt(sensor_data); // will automatically reinitialize // TODO: this blocks sensor!
		LOG_INF("Resuming sensor thread");
		main_imu_resume();
		if (err) {
			LOG_WRN("IMU specific calibration was not completed properly");
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
			return; // Calibration failed
		}
		LOG_INF("Finished IMU specific calibration");
		sys_write(MAIN_SENSOR_DATA_ID, &retained->sensor_data, sensor_data, sizeof(retained->sensor_data));
		sensor_fusion_invalidate(); // only invalidate fusion if calibration was successful
		k_msleep(500);              // Delay before beginning acquisition
	}
#endif

	LOG_INF("Reading data");
	sensor_calibration_clear(a_bias, g_bias, false);
#if CONFIG_SENSOR_USE_TCAL
	int err = sensor_offsetBias(a_bias, g_bias, &avg_temp, &temp_range);
#else
	int err = sensor_offsetBias(a_bias, g_bias, NULL, NULL);
#endif
	if (err) // This takes about 3s
	{
		if (err == -1) {
			LOG_INF("Motion detected");
		} else if (err == -3) {
			LOG_INF("Temperature instability detected");
		}
		a_bias[0] = NAN; // invalidate calibration
	} else {
		LOG_INF("Gyroscope bias: %.5f %.5f %.5f", (double)g_bias[0], (double)g_bias[1], (double)g_bias[2]);
	}
	if (sensor_calibration_validate(a_bias, g_bias, false)) {
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		LOG_INF("Restoring previous calibration");
		LOG_INF("Gyroscope bias: %.5f %.5f %.5f", (double)gyroBias[0], (double)gyroBias[1], (double)gyroBias[2]);
		sensor_calibration_validate(NULL, NULL, true); // additionally verify old calibration
		return;
	} else {
		LOG_INF("Applying calibration");
		memcpy(accelBias, a_bias, sizeof(accelBias));
		memcpy(gyroBias, g_bias, sizeof(gyroBias));
		sensor_fusion_update_bias(NULL); // Only bias changed, preserve orientation
	}
#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	// In 6-side calibration mode, save accelerometer bias (full calibration matrix used elsewhere)
	sys_write(MAIN_ACCEL_BIAS_ID, &retained->accelBias, accelBias, sizeof(accelBias));
#endif

#if CONFIG_SENSOR_USE_TCAL
	if (sensor_tcal_get_auto_calibration() && !isnan(avg_temp)) {
		// Auto temperature calibration enabled: save to tcal data points only, don't change gyro bias
		sys_write(MAIN_GYRO_TEMP_ID, &retained->gyroTemp, &avg_temp, sizeof(avg_temp));
		LOG_INF("T-Cal auto-calibration enabled: saving to tcal data only, not updating gyro bias");

		// Update temperature direction tracking for hysteresis-aware blending
		if (!isnan(tcal_direction_ref_temp)) {
			float delta = avg_temp - tcal_direction_ref_temp;
			if (delta > 0.2f) {
				tcal_current_direction = TCAL_DIR_RISING;
			} else if (delta < -0.2f) {
				tcal_current_direction = TCAL_DIR_FALLING;
			}
			// If delta is within ±0.2°C, keep previous direction (noise filter)
		}
		tcal_direction_ref_temp = avg_temp;

		// Check if T-Cal coverage is good - if so, skip saving a redundant point
		tcal_quality_t quality;
		bool has_good_coverage = false;

		if (sensor_tcal_assess_quality(avg_temp, &quality) && quality.temp_in_range) {
			// Find closest point and check for upper/lower bounds
			float closest_distance = INFINITY;
			bool has_lower_bound = false; // Point below current temp
			bool has_upper_bound = false; // Point above current temp
			float lower_distance = INFINITY;
			float upper_distance = INFINITY;
			float sampling_interval = 1.0f / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE;

			for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
				if (retained->tempCalPoints[i].temp != 0.0f) {
					float point_temp = retained->tempCalPoints[i].temp;
					float distance = fabsf(point_temp - avg_temp);

					if (distance < closest_distance) {
						closest_distance = distance;
					}

					// Check if this point is below or above current temp
					if (point_temp < avg_temp) {
						has_lower_bound = true;
						if (distance < lower_distance) {
							lower_distance = distance;
						}
					} else if (point_temp > avg_temp) {
						has_upper_bound = true;
						if (distance < upper_distance) {
							upper_distance = distance;
						}
					}
				}
			}

			// Coverage is good if:
			// 1. Closest point is within sampling interval (very close match)
			// OR
			// 2. Has both upper and lower bounds AND closest is within 1x sampling interval
			if (closest_distance <= sampling_interval) {
				// Very close to existing point - definitely good coverage
				has_good_coverage = true;
				LOG_INF(
					"T-Cal: Excellent coverage at %.2fC (closest: %.2fC away, within sampling interval)",
					(double)avg_temp,
					(double)closest_distance
				);
			} else if (has_lower_bound && has_upper_bound && closest_distance <= sampling_interval * 1.0f) {
				// Bounded interpolation with reasonable distance
				has_good_coverage = true;
				LOG_INF(
					"T-Cal: Good coverage at %.2fC (bounded: lower %.2fC, upper %.2fC)",
					(double)avg_temp,
					(double)lower_distance,
					(double)upper_distance
				);
			} else {
				// Log why coverage is insufficient
				if (!has_lower_bound || !has_upper_bound) {
					LOG_INF(
						"T-Cal: Coverage insufficient at %.2fC (missing %s bound, closest: %.2fC)",
						(double)avg_temp,
						!has_lower_bound ? "lower" : "upper",
						(double)closest_distance
					);
				} else {
					LOG_INF(
						"T-Cal: Coverage insufficient at %.2fC (closest: %.2fC > threshold: %.2fC)",
						(double)avg_temp,
						(double)closest_distance,
						(double)(sampling_interval * 1.0f)
					);
				}
			}
		}

		if (has_good_coverage) {
			LOG_INF("T-Cal: Coverage sufficient at %.2fC, skipping point save", (double)avg_temp);
		}

		if (!has_good_coverage) {
			// Save as new calibration point
			LOG_INF(
				"T-Cal: Saving calibration point at average temp %.2fC (range: %.2fC)",
				(double)avg_temp,
				(double)temp_range
			);
			int idx = TEMP_TO_IDX(avg_temp);
			if (idx >= 0 && idx < TCAL_BUFFER_SIZE) {

				// Hysteresis-aware blending: prefer rising-phase data.
				// Use tcal_current_direction directly — do not infer from temp comparison.
				if (retained->tempCalPoints[idx].temp != 0.0f) {
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
						"T-Cal: Blending with existing point (dir: %s, alpha: %.2f)",
						tcal_current_direction == TCAL_DIR_RISING    ? "rising"
						: tcal_current_direction == TCAL_DIR_FALLING ? "falling"
																	 : "unknown",
						(double)ema_alpha
					);
					for (int axis = 0; axis < 3; axis++) {
						g_bias[axis]
							= ema_alpha * g_bias[axis] + (1.0f - ema_alpha) * retained->tempCalPoints[idx].bias[axis];
					}
					LOG_INF(
						"T-Cal: Blended bias: %.5f %.5f %.5f",
						(double)g_bias[0],
						(double)g_bias[1],
						(double)g_bias[2]
					);
				} else {
					retained->tempCalState.count++; // New slot
				}
				retained->tempCalPoints[idx].temp = avg_temp;
				memcpy(retained->tempCalPoints[idx].bias, g_bias, sizeof(g_bias));
				retained->tempCalState.valid = false; // Invalidate old curve
				// Manual calibration always writes NVS (user-initiated, not periodic)
				update_tcal_state();

			} else {
				LOG_WRN(
					"T-Cal: Temperature %.2fC is outside the configured calibration range. Point not saved.",
					(double)avg_temp
				);
			}
		}
	} else {
		// Auto tcal not enabled or no valid temperature: save gyro bias to NVS only
		sys_write(MAIN_GYRO_BIAS_ID, &retained->gyroBias, gyroBias, sizeof(gyroBias));
		LOG_INF("Saving gyro bias to NVS (auto tcal not enabled)");
	}
#else
	// No tcal support: always save gyro bias to NVS
	sys_write(MAIN_GYRO_BIAS_ID, &retained->gyroBias, gyroBias, sizeof(gyroBias));
#endif

	LOG_INF("Finished calibration");
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
// Minimum poses required for partial calibration save (must be before sensor_calibrate_6_side)
#define CALIB_MIN_POSES_FOR_PARTIAL 6

void sensor_calibrate_6_side(void)
{
	float a_inv[4][3];
	int captured_count = 0;
	LOG_INF("Calibrating main accelerometer 6-side offset");
	LOG_INF("Rest the device on a stable surface");

	sensor_calibration_clear_6_side(a_inv, false);
	int err = sensor_6_sideBias(a_inv, &captured_count);
	if (err) {
		if (err == -3) {
			// Timeout occurred - check if we have enough samples for partial calibration
			LOG_WRN("Calibration timeout after %d poses (minimum: %d)", captured_count, CALIB_MIN_POSES_FOR_PARTIAL);
			if (captured_count >= CALIB_MIN_POSES_FOR_PARTIAL) {
				// We have enough samples, try to calculate calibration from partial data
				LOG_INF("Attempting partial calibration with %d poses...", captured_count);
				wait_for_threads();
				magneto_current_calibration(a_inv, ata, norm_sum, sample_count);
				magneto_reset();
				// Continue to validation below - err will be handled by validate function
				err = 0; // Clear error to allow validation
			} else {
				// Not enough samples - discard and restore previous calibration
				LOG_ERR("Insufficient poses for calibration, discarding data");
				magneto_reset();
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
				return; // Existing calibration is preserved in accBAinv
			}
		} else {
			magneto_reset();
			if (err == -1) {
				LOG_INF("Motion detected");
			}
			a_inv[0][0] = NAN; // invalidate calibration
		}
	}

	if (!err) {
		LOG_INF("Accelerometer matrix:");
		for (int i = 0; i < 3; i++) {
			LOG_INF(
				"%.5f %.5f %.5f %.5f",
				(double)a_inv[0][i],
				(double)a_inv[1][i],
				(double)a_inv[2][i],
				(double)a_inv[3][i]
			);
		}
	}
	if (sensor_calibration_validate_6_side(a_inv, false)) {
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		LOG_INF("Restoring previous calibration");
		LOG_INF("Accelerometer matrix:");
		for (int i = 0; i < 3; i++) {
			LOG_INF(
				"%.5f %.5f %.5f %.5f",
				(double)accBAinv[0][i],
				(double)accBAinv[1][i],
				(double)accBAinv[2][i],
				(double)accBAinv[3][i]
			);
		}
		sensor_calibration_validate_6_side(NULL, true); // additionally verify old calibration
		return;
	} else {
		LOG_INF("Applying calibration");
		memcpy(accBAinv, a_inv, sizeof(accBAinv));
		sensor_fusion_invalidate(); // only invalidate fusion if calibration was successful
	}
	sys_write(MAIN_ACC_6_BIAS_ID, &retained->accBAinv, accBAinv, sizeof(accBAinv));

	LOG_INF("Finished calibration");
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
}
#endif
