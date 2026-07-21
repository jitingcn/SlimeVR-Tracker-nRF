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
#include <zephyr/irq.h>

#if CONFIG_CMSIS_DSP
#include <arm_math.h>
#endif

#include "bias_collect.h"
#include "cal_mag.h"
#include "cal_sample.h"
#include "calibration.h"
#include "mag_common.h"
#include "magneto.h"
#include "online_mag.h"
#include "cal_imu.h"
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
#include "cal_sens.h"
#endif
#if CONFIG_SENSOR_USE_TCAL
#include "tcal_mls_lut.h"
#include "tcal_runtime.h"
#endif

static uint8_t imu_id;
static uint8_t sensor_data[128]; // any use sensor data

float accelBias[3] = {0}, gyroBias[3] = {0}, magBias[3] = {0}; // offset biases

float accBAinv[4][3];
float magBAinv[4][3];

K_MUTEX_DEFINE(calibration_request_lock);
static int requested_calibration;
static K_SEM_DEFINE(calibration_wake_sem, 0, 1);

/* Also occupies request slot so IMU/TCal/sens cannot overlap magneto_progress collection. */
#define CAL_REQUEST_MAG 6

/* Identify LED on cal thread — never k_msleep on ESB/connection. */
static bool mag_cal_led_pending;

static void calibration_signal_wake(void)
{
	k_sem_give(&calibration_wake_sem);
}

#if CONFIG_SENSOR_USE_SENS_CALIBRATION
// Parameters for the requested gyro sensitivity calibration, latched by
// sensor_request_calibration_sens() before the calibration thread runs.
uint8_t sens_cal_axis;
uint16_t sens_cal_revolutions;
#endif

#define CALIBRATION_SENSOR_INIT_WAIT_MS 10000
#define CALIBRATION_SENSOR_INIT_POLL_MS 10

// #define DEBUG true

#if DEBUG
LOG_MODULE_REGISTER(calibration, LOG_LEVEL_DBG);
#else
LOG_MODULE_REGISTER(calibration, LOG_LEVEL_INF);
#endif

#if CONFIG_SENSOR_USE_TCAL
static float last_gyro_tcal_offset[3] = {0.0f, 0.0f, 0.0f};
#endif

int sensor_calibration_request(int id);

static void calibration_thread(void);
// Keep background calibration below the sensor loop so trial Magneto solves do
// not preempt FIFO servicing. This makes online/manual calibration a little less
// eager, but avoids sensor-loop timing regressions from background work.
K_THREAD_DEFINE(calibration_thread_id, 4096, calibration_thread, NULL, NULL, NULL, CALIBRATION_THREAD_PRIORITY, K_FP_REGS, 0);

void sensor_calibration_process_accel(float a[3])
{
	sensor_sample_accel(a);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	apply_BAinv(a, accBAinv);
#else
	// In single-side calibration mode, accelBias should be zero.
	// Single-side bias is orientation-dependent and should not be applied.
	// for (int i = 0; i < 3; i++) {
	// 	a[i] -= accelBias[i];
	// }
#endif
}

void sensor_calibration_process_gyro(float g[3])
{
	sensor_sample_gyro(g);
#if CONFIG_SENSOR_USE_TCAL
	float calculated_offset[3];
	bool offset_calculated = false;

	const bool auto_cal = sensor_tcal_get_auto_calibration();
	const bool curve_ready = sensor_tcal_curve_apply_ready();
	float temp = NAN;
	if (auto_cal || curve_ready) {
		temp = sensor_get_current_imu_temperature();
	}

	/* Continuous collect only when auto-cal on. */
	if (auto_cal) {
		sensor_tcal_feed_continuous_sample(g, temp);
	}

	/* Cached enable∧points≥min — LUT/MLS; else ZRO below. */
	if (curve_ready) {
		if (sensor_tcal_lut_lookup(temp, calculated_offset) == 0) {
			offset_calculated = true;
		} else if (sensor_tcal_mls_lookup(temp, calculated_offset) == 0) {
			offset_calculated = true;
		}
	}

	if (!offset_calculated) {
		calculated_offset[0] = gyroBias[0];
		calculated_offset[1] = gyroBias[1];
		calculated_offset[2] = gyroBias[2];
	}

	if (retained->bootCalState.doffset_valid) {
#if CONFIG_CMSIS_DSP
		arm_add_f32(calculated_offset, retained->bootCalState.doffset, calculated_offset, 3);
#else
		for (int axis = 0; axis < 3; axis++) {
			calculated_offset[axis] += retained->bootCalState.doffset[axis];
		}
#endif
	}

#if CONFIG_CMSIS_DSP
	arm_sub_f32(g, calculated_offset, g, 3);
#else
	for (int i = 0; i < 3; i++) {
		g[i] -= calculated_offset[i];
	}
#endif

	memcpy(last_gyro_tcal_offset, calculated_offset, sizeof(last_gyro_tcal_offset));
#else
#if CONFIG_CMSIS_DSP
	arm_sub_f32(g, gyroBias, g, 3);
#else
	for (int i = 0; i < 3; i++) {
		g[i] -= gyroBias[i];
	}
#endif
#endif
}

void sensor_calibration_process_mag(float m[3])
{
	//	for (int i = 0; i < 3; i++)
	//		m[i] -= magBias[i];
	sensor_sample_mag(m);
	/* Snap under irq_lock so calibration_thread publish cannot tear the live matrix. */
	float snap[4][3];
	unsigned key = irq_lock();
	memcpy(snap, magBAinv, sizeof(snap));
	irq_unlock(key);
	apply_BAinv(m, snap);
}

void sensor_calibration_update_sensor_ids(int imu)
{
	imu_id = imu;
}

uint8_t sensor_calibration_get_imu_id(void)
{
	return imu_id;
}

uint8_t *sensor_calibration_get_sensor_data()
{
	return sensor_data;
}

void sensor_calibration_read(void)
{
	memcpy(sensor_data, retained->sensor_data, sizeof(sensor_data));
	memcpy(accelBias, retained->accelBias, sizeof(accelBias));
	memcpy(gyroBias, retained->gyroBias, sizeof(gyroBias));
	memcpy(magBias, retained->magBias, sizeof(magBias));
	{
		unsigned key = irq_lock();
		memcpy(magBAinv, retained->magBAinv, sizeof(magBAinv));
		irq_unlock(key);
	}
	memcpy(accBAinv, retained->accBAinv, sizeof(accBAinv));
	if (retained->mag_online_calibration_mode > MAG_ONLINE_CALIBRATION_DISABLED) {
		retained->mag_online_calibration_mode = MAG_ONLINE_CALIBRATION_DEFAULT;
	}
	LOG_INF("Online mag calibration: %s", sensor_calibration_get_online_mag_enabled() ? "enabled" : "disabled");
	if (sensor_calibration_get_online_mag_enabled()) {
		float zero[3] = {0};
		if (v_diff_mag(magBAinv[0], zero) != 0) {
			magneto_online_runtime_load_retained();
			if (cal_online_mag_update_count() > 0 || cal_online_mag_norm_count() > 0) {
				LOG_INF(
					"Online mag runtime restored (%d updates, %u norm samples)",
					cal_online_mag_update_count(),
					cal_online_mag_norm_count()
				);
			}
		} else {
			magneto_online_runtime_reset();
		}
	} else {
		magneto_online_runtime_reset();
	}
#if CONFIG_SENSOR_USE_TCAL
	sensor_tcal_runtime_init_from_retained();
#endif
}

int sensor_calibration_validate(float *a_bias, float *g_bias, bool write)
{
	if (a_bias == NULL) {
		a_bias = accelBias;
	}
	if (g_bias == NULL) {
		g_bias = gyroBias;
	}
	float zero[3] = {0};
	if (!v_epsilon(a_bias, zero, 0.5) || !v_epsilon(g_bias, zero, 50.0)) // check accel is <0.5G and gyro <50dps
	{
		sensor_calibration_clear(a_bias, g_bias, write);
		// Validation failure: do NOT call any fusion function
		// Let fusion keep its current bias estimate to avoid residual drift
		LOG_WRN("Invalidated calibration");
		LOG_WRN("The IMU may be damaged or calibration was not completed properly");
		return -1;
	}
	return 0;
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
int sensor_calibration_validate_6_side(float a_inv[][3], bool write)
{
	if (a_inv == NULL) {
		a_inv = accBAinv;
	}
	float zero[3] = {0};
	float diagonal[3];
	for (int i = 0; i < 3; i++) {
		diagonal[i] = a_inv[i + 1][i];
	}
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	if (!v_epsilon(a_inv[0], zero, 0.5)
		|| !v_epsilon(diagonal, average, magnitude * 0.1f)) // check accel is <0.5G and diagonals are within 10%
	{
		sensor_calibration_clear_6_side(a_inv, write);
		LOG_WRN("Invalidated calibration");
		LOG_WRN("The IMU may be damaged or calibration was not completed properly");
		return -1;
	}
	return 0;
}
#endif

int sensor_calibration_validate_mag(float m_inv[][3], bool write)
{
	if (m_inv == NULL) {
		m_inv = magBAinv;
	}
	if (!mag_bainv_structurally_ok(m_inv, 0.0f)) {
		sensor_calibration_clear_mag(m_inv, write);
		LOG_WRN("Invalidated calibration");
		LOG_WRN("The magnetometer may be damaged or calibration was not completed properly");
		return -1;
	}
	return 0;
}

void sensor_calibration_clear(float *a_bias, float *g_bias, bool write)
{
	if (a_bias == NULL) {
		a_bias = accelBias;
	}
	if (g_bias == NULL) {
		g_bias = gyroBias;
	}
	memset(a_bias, 0, sizeof(accelBias));
	memset(g_bias, 0, sizeof(gyroBias));
	if (write) {
		LOG_INF("Clearing stored calibration data");
		sys_write(MAIN_ACCEL_BIAS_ID, &retained->accelBias, a_bias, sizeof(accelBias));
		sys_write(MAIN_GYRO_BIAS_ID, &retained->gyroBias, g_bias, sizeof(gyroBias));
#if CONFIG_SENSOR_USE_TCAL
		// Also clear boot/runtime calibration D_offset since ZRO is being reset
		retained->bootCalState.doffset_valid = false;
		retained->bootCalState.doffset[0] = 0.0f;
		retained->bootCalState.doffset[1] = 0.0f;
		retained->bootCalState.doffset[2] = 0.0f;
		LOG_INF("Clearing D_offset along with ZRO calibration");
#endif
		// Note: Caller is responsible for calling sensor_fusion_update_bias() or
		// sensor_fusion_invalidate() as appropriate:
		// - sensor_fusion_update_bias(): for internal/automatic calibration (preserves quaternion)
		// - sensor_fusion_invalidate(): for manual reset commands (resets quaternion)
	}
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
void sensor_calibration_clear_6_side(float a_inv[][3], bool write)
{
	if (a_inv == NULL) {
		a_inv = accBAinv;
	}
	memset(a_inv, 0, sizeof(accBAinv));
	for (int i = 0; i < 3; i++) { // set identity matrix
		a_inv[i + 1][i] = 1;
	}
	if (write) {
		LOG_INF("Clearing stored calibration data");
		sys_write(MAIN_ACC_6_BIAS_ID, &retained->accBAinv, a_inv, sizeof(accBAinv));
	}
}
#endif

void sensor_calibration_clear_mag(float m_inv[][3], bool write)
{
	bool clearing_live_state = (m_inv == NULL || m_inv == magBAinv);
	if (m_inv == NULL) {
		m_inv = magBAinv;
	}
	if (clearing_live_state) {
		unsigned key = irq_lock();
		memset(m_inv, 0, sizeof(magBAinv)); // zeroed matrix will disable magnetometer in fusion
		irq_unlock(key);
		magneto_online_runtime_reset();
	} else {
		memset(m_inv, 0, sizeof(magBAinv));
	}
	if (write) {
		LOG_INF("Clearing stored calibration data");
		sensor_calibration_online_mag_retained_clear();
		sys_write(MAIN_MAG_BIAS_ID, &retained->magBAinv, m_inv, sizeof(magBAinv));
		sensor_refresh_sensor_ids(); // Refresh reported mag status after clear
	}
}

void sensor_request_calibration(void)
{
	sensor_calibration_request(1);
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
void sensor_request_calibration_6_side(void)
{
	sensor_calibration_request(2);
}
#endif

#if CONFIG_SENSOR_USE_SENS_CALIBRATION
int sensor_request_calibration_sens(uint8_t axis, uint16_t revolutions)
{
	if (axis > 2 || revolutions == 0) {
		return -1;
	}

	k_mutex_lock(&calibration_request_lock, K_FOREVER);
	if (requested_calibration != 0 || (magneto_progress & 0x80)) {
		k_mutex_unlock(&calibration_request_lock);
		LOG_ERR("Sensor calibration is already running");
		return -1;
	}

	sens_cal_axis = axis;
	sens_cal_revolutions = revolutions;
	requested_calibration = 5;
	k_mutex_unlock(&calibration_request_lock);
	calibration_signal_wake();
	return 0;
}
#endif

void sensor_request_calibration_mag(void)
{
	k_mutex_lock(&calibration_request_lock, K_FOREVER);
	if (magneto_progress & 0x80 || mag_cal_led_pending) {
		k_mutex_unlock(&calibration_request_lock);
		if (!get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
			set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
		}
		return;
	}
	if (requested_calibration != 0) {
		k_mutex_unlock(&calibration_request_lock);
		LOG_ERR("Sensor calibration is already running");
		return;
	}
	/* Claim slot immediately; LED + magneto_progress arm on cal thread. */
	requested_calibration = CAL_REQUEST_MAG;
	mag_cal_led_pending = true;
	k_mutex_unlock(&calibration_request_lock);

	if (!get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
		set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
	}
	calibration_signal_wake();
	LOG_INF("Magnetometer calibration requested");
}


int sensor_calibration_request(int id)
{
	int result;

	k_mutex_lock(&calibration_request_lock, K_FOREVER);
	switch (id) {
	case -1:
		requested_calibration = 0;
		mag_cal_led_pending = false;
		result = 0;
		break;
	case 0:
		result = requested_calibration;
		break;
	default:
		if (requested_calibration != 0 || (magneto_progress & 0x80)) {
			LOG_ERR("Sensor calibration is already running");
			result = -1;
		} else {
			requested_calibration = id;
			result = 0;
		}
		break;
	}
	k_mutex_unlock(&calibration_request_lock);
	if (result == 0 && id > 0) {
		calibration_signal_wake();
	}
	return result;
}

static void calibration_thread(void)
{
	/* Register calibration thread with watchdog - use long timeout for lengthy operations */
	watchdog_register_thread(WDT_CHANNEL_CALIBRATION, 0);

	sensor_calibration_read();

	// Startup validation and T-Cal LUT build require live sensor state.
	// If no IMU was detected, keep the retained values as-is and avoid
	// reporting missing hardware as corrupted calibration.
	bool sensor_ready = sensor_is_initialized();
	int64_t init_wait_start = k_uptime_get();
	while (!sensor_ready) {
		watchdog_feed(WDT_CHANNEL_CALIBRATION);
		k_msleep(CALIBRATION_SENSOR_INIT_POLL_MS);
		sensor_ready = sensor_is_initialized();
		if (!sensor_ready && k_uptime_get() - init_wait_start >= CALIBRATION_SENSOR_INIT_WAIT_MS) {
			LOG_INF("Sensor not initialized; skipping startup calibration validation");
			break;
		}
	}

#if CONFIG_SENSOR_USE_TCAL
	// Build LUT at startup if T-Cal data is available and sensor is initialized
	// LUT is only in RAM and needs to be rebuilt after every boot
	// Use incremental build: priority zone first, then background completion
	if (sensor_ready && retained->tempCalState.count >= MLS_MIN_POINTS_FOR_FIT) {
		// Validate tempCalState.count to prevent issues with corrupted retained data
		if (retained->tempCalState.count > TCAL_BUFFER_SIZE) {
			LOG_ERR(
				"T-Cal: Invalid point count %u (max %d), resetting",
				retained->tempCalState.count,
				TCAL_BUFFER_SIZE
			);
			retained->tempCalState.count = 0;
			retained->tempCalState.valid = false;
		} else {
			float current_temp = sensor_get_current_imu_temperature();
			if (!isnan(current_temp)) {
				LOG_INF(
					"T-Cal: Starting incremental LUT build at startup (current temp: %.1f°C)",
					(double)current_temp
				);
				sensor_tcal_build_lut_priority(current_temp);
			} else {
				LOG_WRN("T-Cal: Cannot build LUT - temperature not available");
			}
		}
	}
#endif

	// TODO: be able to block the sensor while doing certain operations
	// TODO: reset fusion on calibration finished
	// TODO: start and run thread from request?
	// TODO: replace wait_for_motion with isAccRest

	// Verify calibrations only after the sensor stack is initialized.
	if (sensor_ready) {
		sensor_calibration_validate(NULL, NULL, true);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		sensor_calibration_validate_6_side(NULL, true);
#endif
		if (sensor_get_mag_available()) {
			sensor_calibration_validate_mag(NULL, true);
		}
	}

	// requested calibrations run here
	while (1) {
		int requested = sensor_calibration_request(0);
		switch (requested) {
		case 1:
			set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
			sensor_calibrate_imu();
			sensor_calibration_request(-1); // clear request
			set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
			break;
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		case 2:
			set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
			sensor_calibrate_6_side();
			sensor_calibration_request(-1); // clear request
			set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
			break;
#endif
#if CONFIG_SENSOR_USE_TCAL
		case 3: // Boot calibration
			set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
			sensor_perform_boot_calibration();
			sensor_calibration_request(-1); // clear request
			set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
			break;
		case 4: // Runtime periodic calibration
			set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
			sensor_perform_runtime_calibration();
			sensor_calibration_request(-1); // clear request
			set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
			break;
#endif
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
		case 5: // Gyro sensitivity calibration
			set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
			sensor_calibrate_sens();
			sensor_calibration_request(-1); // clear request
			set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
			break;
#endif
		case CAL_REQUEST_MAG:
			if (mag_cal_led_pending) {
				mag_cal_led_pending = false;
				LOG_INF("Magnetometer calibration: identify tracker");
				set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_SENSOR);
				watchdog_feed(WDT_CHANNEL_CALIBRATION);
				k_msleep(2000);
				watchdog_feed(WDT_CHANNEL_CALIBRATION);
				set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
				k_msleep(800);
				watchdog_feed(WDT_CHANNEL_CALIBRATION);
				magneto_reset();
				magneto_online_reset();
				magneto_progress |= 1 << 7;
				LOG_INF("Magnetometer calibration started (rotate tracker in all orientations)");
				break;
			}
			if (magneto_progress & 0b10000000) {
				requested = sensor_calibrate_mag();
				/* 1 = still collecting; 0/-1 = finished or aborted */
				if (requested != 1) {
					sensor_calibration_request(-1);
				}
			} else {
				sensor_calibration_request(-1);
			}
			break;
		default:
			break;
		}

#if CONFIG_SENSOR_USE_TCAL
		// Continue LUT background build if in progress
		if (sensor_tcal_lut_get_build_state() == MLS_LUT_BUILD_BACKGROUND) {
			if (sensor_tcal_build_lut_continue()) {
				LOG_INF(
					"T-Cal LUT: Background build complete (%d/%d entries)",
					sensor_tcal_lut_get_computed_count(),
					MLS_LUT_SIZE
				);
			}
		}
#endif

		// Phase 2: Background online magnetometer calibration check
		if (requested == 0) {
			sensor_calibration_online_mag_check();
		}

		/* Feed watchdog at end of each loop iteration */
		watchdog_feed(WDT_CHANNEL_CALIBRATION);

		if (requested < 0) {
			(void)k_sem_take(&calibration_wake_sem, K_MSEC(5));
		} else if (requested > 0) {
			(void)k_sem_take(&calibration_wake_sem, K_MSEC(20));
		} else {
			(void)k_sem_take(&calibration_wake_sem, K_MSEC(100));
		}
	}
}

#if CONFIG_SENSOR_USE_TCAL

void sensor_tcal_status(void)
{
	printk("Temperature Calibration Status (MLS):\n");
	printk(
		"  - Compensation flag: %s\n",
		sensor_tcal_get_enabled() ? "enabled (default on)" : "disabled"
	);
	printk(
		"  - Active apply: %s",
		sensor_tcal_get_apply_mode_name()
	);
	if (sensor_tcal_get_apply_mode() == SENSOR_TCAL_APPLY_ZRO_FALLBACK) {
		printk(" (need >= %d points for curve)\n", MLS_MIN_POINTS_FOR_FIT);
	} else {
		printk("\n");
	}
	printk("  - MLS available: %s\n", retained->tempCalState.valid ? "Yes" : "No");
	printk("  - Points collected: %u / %d\n", retained->tempCalState.count, TCAL_BUFFER_SIZE);

	// Display LUT status
	const char *lut_state_str;
	switch (sensor_tcal_lut_get_build_state()) {
	case MLS_LUT_BUILD_IDLE:
		lut_state_str = "Idle";
		break;
	case MLS_LUT_BUILD_PRIORITY:
		lut_state_str = "Building priority zone";
		break;
	case MLS_LUT_BUILD_BACKGROUND:
		lut_state_str = "Background build";
		break;
	case MLS_LUT_BUILD_COMPLETE:
		lut_state_str = "Complete";
		break;
	default:
		lut_state_str = "Unknown";
		break;
	}
	printk(
		"  - LUT valid: %s, state: %s, entries: %d/%d\n",
		sensor_tcal_lut_is_valid() ? "Yes" : "No",
		lut_state_str,
		sensor_tcal_lut_get_computed_count(),
		MLS_LUT_SIZE
	);

	// Use quality assessment to get calibrated temperature range and error
	float current_temp = sensor_get_current_imu_temperature();
	tcal_quality_t quality;
	bool quality_ok = sensor_tcal_assess_quality(current_temp, &quality);

	if (quality.point_count > 0 && quality.temp_min < quality.temp_max) {
		printk("  - Calibrated temp range: %.2fC to %.2fC\n", (double)quality.temp_min, (double)quality.temp_max);
	}

	// Display quality assessment
	if (quality.point_count > 0) {
		// Display quality status
		const char *quality_str;
		if (quality_ok) {
			quality_str = "GOOD - Suitable for boot calibration (MLS)";
		} else if (quality.point_count < BOOT_CAL_MIN_CURVE_POINTS) {
			quality_str = "INSUFFICIENT - Need more calibration points";
		} else {
			quality_str = "UNKNOWN";
		}
		printk("  - Calibration quality: %s\n", quality_str);
	}

	// Display Boot Calibration D_offset
	printk("\nBoot/Runtime Calibration Status:\n");
	if (retained->bootCalState.doffset_valid) {
		printk(
			"  - D_offset: [%.5f, %.5f, %.5f] dps\n",
			(double)retained->bootCalState.doffset[0],
			(double)retained->bootCalState.doffset[1],
			(double)retained->bootCalState.doffset[2]
		);
		printk("  - Boot Cal completed: Yes\n");
	} else {
		printk("  - D_offset: Not available\n");
		printk("  - Boot Cal completed: %s\n", retained->bootCalState.completed ? "Failed/Skipped" : "Not yet");
	}

	// Display runtime calibration status
	int64_t last_runtime_cal_time = 0;
	int64_t current_rest_duration = 0;
	sensor_runtime_cal_get_status(&last_runtime_cal_time, &current_rest_duration);

	printk("  - Last calibration temp: ");
	if (!isnan(runtime_cal_last_temp)) {
		printk("%.2fC\n", (double)runtime_cal_last_temp);
	} else {
		printk("N/A\n");
	}

	printk("  - Current temp: %.2fC", (double)current_temp);
	if (!isnan(runtime_cal_last_temp)) {
		float temp_diff = fabsf(current_temp - runtime_cal_last_temp);
		printk(" (diff: %.2fC, threshold: %.1fC)", (double)temp_diff, (double)RUNTIME_CAL_TEMP_CHANGE_MIN);
	}
	printk("\n");

	printk("  - Runtime Cal last: ");
	if (last_runtime_cal_time > 0) {
		int64_t seconds_ago = (k_uptime_get() - last_runtime_cal_time) / 1000;
		printk("%lld seconds ago\n", seconds_ago);
	} else {
		printk("Not yet performed\n");
	}

	if (current_rest_duration > 0) {
		printk(
			"  - Current rest duration: %lld ms (trigger at %d ms)\n",
			current_rest_duration,
			RUNTIME_CAL_REST_TIME_MS
		);
	}

	// Display mode info
	if (quality.point_count > 0) {
		printk("  - Bias tracking mode: T-Cal + D_offset\n");
	} else {
		printk("  - Bias tracking mode: Static gyroBias + D_offset\n");
	}
}

// Public function for 'tcal clear' and 'reset tcal'
void sensor_tcal_clear(void)
{
	if (sensor_calibration_request(0) != 0) {
		LOG_ERR("Another calibration is running. Cannot clear T-Cal data.");
		printk("Error: Another calibration is running.\n");
		return;
	}

	// Invalidate lookup cache since calibration data will be cleared
	sensor_tcal_cache_invalidate();

	// Reset temperature direction tracking
	tcal_current_direction = TCAL_DIR_UNKNOWN;
	tcal_direction_ref_temp = NAN;

	LOG_INF("Clearing all manual T-Cal data.");
	memset(retained->tempCalPoints, 0, sizeof(retained->tempCalPoints));
	memset(retained->tempCalCoeffs, 0, sizeof(retained->tempCalCoeffs));
	memset(&retained->tempCalState, 0, sizeof(retained->tempCalState)); // Clear the whole state struct

	// Save cleared state to NVS directly (don't call update_tcal_state which refreshes runtime state)
	sys_write(
		MAIN_GYRO_TCAL_STATE_ID,
		&retained->tempCalState,
		&retained->tempCalState,
		sizeof(retained->tempCalState)
	);
	sys_write(
		MAIN_GYRO_TCAL_POINTS_ID,
		retained->tempCalPoints,
		retained->tempCalPoints,
		sizeof(retained->tempCalPoints)
	);
	sys_write(
		MAIN_GYRO_TCAL_COEFFS_ID,
		retained->tempCalCoeffs,
		retained->tempCalCoeffs,
		sizeof(retained->tempCalCoeffs)
	);

	// Also clear boot/runtime calibration D_offset since T-Cal is being reset
	retained->bootCalState.doffset_valid = false;
	retained->bootCalState.doffset[0] = 0.0f;
	retained->bootCalState.doffset[1] = 0.0f;
	retained->bootCalState.doffset[2] = 0.0f;
	LOG_INF("Clearing D_offset along with T-Cal data");

	// Reset continuous accumulator sampling state
	tcal_accum_reset();
	sensor_tcal_refresh_apply_cache();

	// Manual command: invalidate fusion to force quaternion recalculation
	sensor_fusion_invalidate();

	printk("All temperature calibration data and D_offset have been cleared.\n");
}

// Public function for 'tcal remove <index>'
void sensor_tcal_remove_point(int index_to_remove)
{
	if (sensor_calibration_request(0) != 0) {
		LOG_ERR("Another calibration is running. Cannot remove T-Cal point.");
		printk("Error: Another calibration is running.\n");
		return;
	}

	if (index_to_remove < 0 || index_to_remove >= TCAL_BUFFER_SIZE) {
		printk("Error: Index %d is out of valid range (0 to %d).\n", index_to_remove, TCAL_BUFFER_SIZE - 1);
		return;
	}

	// Check if there was actually data in that slot
	if (retained->tempCalPoints[index_to_remove].temp != 0.0f) {
		// Invalidate lookup cache since a point is being removed
		sensor_tcal_cache_invalidate();

		LOG_INF("Removing T-Cal point at index %d.", index_to_remove);

		// Zero out the slot
		retained->tempCalPoints[index_to_remove].temp = 0.0f;
		memset(retained->tempCalPoints[index_to_remove].bias, 0, sizeof(retained->tempCalPoints[index_to_remove].bias));

		// Recalculate the count by scanning all points
		uint16_t new_count = 0;
		for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
			if (retained->tempCalPoints[i].temp != 0.0f) {
				new_count++;
			}
		}
		retained->tempCalState.count = new_count;
		retained->tempCalState.valid = false;

		printk("Point at index %d removed. Recalculating MLS state...\n", index_to_remove);
		update_tcal_state();
		sys_flush_warm(); /* console/user action: durable immediately */
	} else {
		printk("No data found at index %d. Nothing to remove.\n", index_to_remove);
	}
}

// Check if current temperature needs calibration (missing nearby calibration point)
bool sensor_tcal_is_temp_outside_range(float temp, float *min_temp, float *max_temp)
{
	if (retained->tempCalState.count < 1) {
		if (min_temp) {
			*min_temp = NAN;
		}
		if (max_temp) {
			*max_temp = NAN;
		}
		return true; // No calibration data, need calibration
	}

	// Calculate the sampling interval from Kconfig
	// CONFIG_SENSOR_POLY_STEPS_PER_DEGREE: e.g., 2 means 0.5°C steps, 1 means 1.0°C steps
	float sampling_interval = 1.0f / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE;

	// Find the closest calibration point
	float closest_distance = INFINITY;
	float closest_temp = NAN;

	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (retained->tempCalPoints[i].temp != 0.0f) {
			float distance = fabsf(retained->tempCalPoints[i].temp - temp);
			if (distance < closest_distance) {
				closest_distance = distance;
				closest_temp = retained->tempCalPoints[i].temp;
			}
		}
	}

	// Return the closest point info if requested
	if (min_temp) {
		*min_temp = closest_temp;
	}
	if (max_temp) {
		*max_temp = closest_distance;
	}

	// Need calibration if closest point is farther than sampling interval
	// Add small tolerance (5%) to avoid triggering at boundary
	return (closest_distance > sampling_interval * 1.05f);
}

void sensor_calibration_get_last_gyro_offset(float offset[3])
{
	memcpy(offset, last_gyro_tcal_offset, sizeof(last_gyro_tcal_offset));
}

#endif
