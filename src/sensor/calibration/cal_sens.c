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
#include "system/system.h"
#include "system/watchdog.h"

#include <math.h>
#include <zephyr/kernel.h>

#include "cal_sample.h"
#include "cal_sens.h"
#include "util.h"

#if CONFIG_SENSOR_USE_SENS_CALIBRATION

LOG_MODULE_REGISTER(cal_sens, LOG_LEVEL_INF);

/* Latched by sensor_request_calibration_sens() in calibration.c */
extern uint8_t sens_cal_axis;
extern uint16_t sens_cal_revolutions;

// =============================================================================
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
// The user spins the tracker a known number of full revolutions about a single
// axis. We integrate the measured gyro rate over that motion and compare the
// measured angle against the true angle to derive a per-axis scale factor that
// corrects cumulative over- or under-rotation.
#define SENS_CAL_BIAS_SAMPLE_MS 1000    // In-situ bias averaging window
#define SENS_CAL_START_RATE_DPS 30.0f   // Rate that counts as "spin started"
#define SENS_CAL_STOP_RATE_DPS 10.0f    // Rate that counts as "spin stopped"
#define SENS_CAL_STOP_DWELL_MS 1000     // Rate must stay low this long to stop
#define SENS_CAL_START_TIMEOUT_MS 30000 // Give up waiting for the spin to start
#define SENS_CAL_SPIN_TIMEOUT_MS 60000  // Give up waiting for the spin to finish
#define SENS_CAL_MIN_FRACTION 0.85f     // Require near-complete expected angle before stopping
#define SENS_CAL_MIN_SCALE 0.9f         // Reject implausible results (likely wrong turn count)
#define SENS_CAL_MAX_SCALE 1.1f
#define SENS_CAL_WARN_OFF_AXIS_RATIO 0.10f
#define SENS_CAL_MAX_OFF_AXIS_RATIO 0.25f

void sensor_calibrate_sens(void)
{
	uint8_t axis = sens_cal_axis;
	uint16_t revolutions = sens_cal_revolutions;

	if (axis > 2 || revolutions == 0) {
		LOG_ERR("Sensitivity calibration: invalid parameters");
		printk("Gyro sensitivity auto-calibration failed: invalid parameters.\n");
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}
	char axis_char = "XYZ"[axis];
	float expected_deg = 360.0f * revolutions;

	LOG_INF(
		"Sensitivity calibration: axis %c, %u rev (%.1f deg expected)",
		axis_char,
		revolutions,
		(double)expected_deg
	);

	float g[3];

	// 1. Wait for the tracker to be held still before measuring bias.
	set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_SENSOR);
	LOG_INF("Sensitivity calibration: hold still");
	if (!wait_for_motion(false, 6)) {
		LOG_WRN("Sensitivity calibration: tracker not still, aborting");
		printk("Gyro sensitivity auto-calibration failed: tracker was not still.\n");
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}

	// 2. Measure the in-situ gyro bias. sensor_wait_gyro returns
	//    raw samples (before bias and sensitivity are applied), so we average a
	//    short window here rather than relying on the stored gyro bias.
	double bias_sum[3] = {0.0, 0.0, 0.0};
	int bias_count = 0;
	int64_t bias_start = k_uptime_get();
	while (k_uptime_get() - bias_start < SENS_CAL_BIAS_SAMPLE_MS) {
		if (sensor_wait_gyro(g, K_MSEC(1000))) {
			LOG_WRN("Sensitivity calibration: gyro timeout during bias, aborting");
			printk("Gyro sensitivity auto-calibration failed: gyro timeout while measuring bias.\n");
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
			return;
		}
		for (int i = 0; i < 3; i++) {
			bias_sum[i] += (double)g[i];
		}
		bias_count++;
		watchdog_feed(WDT_CHANNEL_CALIBRATION);
	}
	if (bias_count == 0) {
		LOG_WRN("Sensitivity calibration: no bias samples, aborting");
		printk("Gyro sensitivity auto-calibration failed: no bias samples.\n");
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}
	float gyro_bias[3];
	for (int i = 0; i < 3; i++) {
		gyro_bias[i] = (float)(bias_sum[i] / bias_count);
	}
	LOG_INF(
		"Sensitivity calibration: bias %.4f %.4f %.4f dps",
		(double)gyro_bias[0],
		(double)gyro_bias[1],
		(double)gyro_bias[2]
	);

	// 3. Arm and wait for the user to start spinning. FLASH means "ready, spin now".
	set_led(SYS_LED_PATTERN_FLASH, SYS_LED_PRIORITY_SENSOR);
	LOG_INF("Sensitivity calibration: spin the tracker about the %c axis now", axis_char);
	int64_t arm_start = k_uptime_get();
	int64_t last_wdt = arm_start;
	float rate = 0.0f;
	while (true) {
		if (k_uptime_get() - arm_start >= SENS_CAL_START_TIMEOUT_MS) {
			LOG_WRN("Sensitivity calibration: no spin detected, aborting");
			printk("Gyro sensitivity auto-calibration failed: no spin detected.\n");
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
			return;
		}
		if (sensor_wait_gyro(g, K_MSEC(1000))) {
			continue; // Tolerate occasional waits while watching for the start
		}
		rate = g[axis] - gyro_bias[axis];
		if (k_uptime_get() - last_wdt >= 1000) {
			watchdog_feed(WDT_CHANNEL_CALIBRATION);
			last_wdt = k_uptime_get();
		}
		if (fabsf(rate) > SENS_CAL_START_RATE_DPS) {
			break;
		}
	}

	// 4. Integrate the gyro rate over the spin. ON indicates recording.
	//    sensor_wait_gyro returns only the most recent sample, so crediting each
	//    observed sample a fixed 1/ODR step would silently drop the rotation from
	//    any samples produced while this loop was busy and undercount the spin.
	//    Integrate against the real elapsed time between samples instead (as the
	//    fusion path does with its measured time step); each observed sample then
	//    covers the true interval since the previous one, which also tolerates the
	//    sensor's actual sample rate differing from its nominal ODR.
	set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_SENSOR);
	LOG_INF("Sensitivity calibration: recording");
	double measured = 0.0;
	double axis_motion = 0.0;
	double off_axis_motion = 0.0;
	int64_t spin_start = k_uptime_get();
	int64_t last_ticks = k_uptime_ticks();
	int64_t below_since = -1; // When the rate first dropped below the stop threshold
	bool finished = false;
	last_wdt = spin_start;
	while (k_uptime_get() - spin_start < SENS_CAL_SPIN_TIMEOUT_MS) {
		if (sensor_wait_gyro(g, K_MSEC(1000))) {
			LOG_WRN("Sensitivity calibration: gyro timeout during spin, aborting");
			printk("Gyro sensitivity auto-calibration failed: gyro timeout during spin.\n");
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
			return;
		}
		int64_t now_ticks = k_uptime_ticks();
		double dt = (double)k_ticks_to_us_near64(now_ticks - last_ticks) * 1e-6;
		last_ticks = now_ticks;

		rate = g[axis] - gyro_bias[axis];
		measured += (double)rate * dt;
		axis_motion += fabs((double)rate) * dt;
		double off_axis_rate_sq = 0.0;
		for (int i = 0; i < 3; i++) {
			if (i != axis) {
				double off_rate = (double)(g[i] - gyro_bias[i]);
				off_axis_rate_sq += off_rate * off_rate;
			}
		}
		off_axis_motion += sqrt(off_axis_rate_sq) * dt;

		if (k_uptime_get() - last_wdt >= 1000) {
			watchdog_feed(WDT_CHANNEL_CALIBRATION);
			last_wdt = k_uptime_get();
		}

		// The spin is complete once the rate stays low for the dwell time, but only
		// after at least a minimum fraction of the expected angle has been covered.
		// This keeps a brief pause mid-spin from ending the measurement early.
		if (fabsf(rate) < SENS_CAL_STOP_RATE_DPS && fabs(measured) >= (double)(expected_deg * SENS_CAL_MIN_FRACTION)) {
			if (below_since < 0) {
				below_since = k_uptime_get();
			} else if (k_uptime_get() - below_since >= SENS_CAL_STOP_DWELL_MS) {
				finished = true;
				break;
			}
		} else {
			below_since = -1;
		}
	}

	if (!finished) {
		LOG_WRN("Sensitivity calibration: spin did not complete in time, aborting");
		printk("Gyro sensitivity auto-calibration failed: spin did not complete in time.\n");
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}

	float measured_deg = (float)fabs(measured);
	if (measured_deg < 1e-3f) {
		LOG_WRN("Sensitivity calibration: measured angle too small, aborting");
		printk("Gyro sensitivity auto-calibration failed: measured angle too small.\n");
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}

	// Want measured_deg * scale == expected_deg, independent of rotation direction.
	float scale = expected_deg / measured_deg;
	float over_rotation = measured_deg - expected_deg;
	float off_axis_ratio = axis_motion > 1e-3 ? (float)(off_axis_motion / axis_motion) : 1.0f;
	float equivalent_diff_deg = (1.0f - (1.0f / scale)) * (360.0f * CONFIG_SENSOR_SENS_REV);
	LOG_INF(
		"Sensitivity calibration: measured %.2f deg, expected %.2f deg, over-rotation %.2f deg, off-axis %.3f",
		(double)measured_deg,
		(double)expected_deg,
		(double)over_rotation,
		(double)off_axis_ratio
	);
	LOG_INF("Sensitivity calibration: computed scale %.5f", (double)scale);

	if (!v_finite(&scale, 1)) {
		LOG_WRN("Sensitivity calibration: computed non-finite scale, not applied");
		printk("Gyro sensitivity auto-calibration rejected: invalid scale. Nothing saved.\n");
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}

	if (off_axis_ratio > SENS_CAL_MAX_OFF_AXIS_RATIO) {
		LOG_WRN(
			"Sensitivity calibration: off-axis ratio %.3f above %.3f, not applied",
			(double)off_axis_ratio,
			(double)SENS_CAL_MAX_OFF_AXIS_RATIO
		);
		printk(
			"Gyro sensitivity auto-calibration rejected: too much off-axis motion (%.2f > %.2f). Nothing saved.\n",
			(double)off_axis_ratio,
			(double)SENS_CAL_MAX_OFF_AXIS_RATIO
		);
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}

	// Reject implausible results. The firmware cannot distinguish a wrong revolution
	// count from a genuinely large sensitivity error, so a scale far from 1.0 most
	// likely means the wrong number of turns was performed.
	if (scale < SENS_CAL_MIN_SCALE || scale > SENS_CAL_MAX_SCALE) {
		LOG_WRN(
			"Sensitivity calibration: scale %.5f out of range [%.2f, %.2f], not applied",
			(double)scale,
			(double)SENS_CAL_MIN_SCALE,
			(double)SENS_CAL_MAX_SCALE
		);
		printk(
			"Gyro sensitivity auto-calibration rejected: measured %.2f deg for %.2f deg, scale %.5f outside "
			"%.2f..%.2f. Equivalent sens diff over %u rev: %.3f deg. Nothing saved.\n",
			(double)measured_deg,
			(double)expected_deg,
			(double)scale,
			(double)SENS_CAL_MIN_SCALE,
			(double)SENS_CAL_MAX_SCALE,
			(unsigned int)CONFIG_SENSOR_SENS_REV,
			(double)equivalent_diff_deg
		);
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}

	if (!retained) {
		LOG_ERR("Sensitivity calibration: retained data unavailable, not applied");
		printk("Gyro sensitivity auto-calibration failed: retained data unavailable.\n");
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}

	retained->gyroSensScale[axis] = scale;
	retained_update();
	sys_write(MAIN_GYRO_SENS_ID, &retained->gyroSensScale, retained->gyroSensScale, sizeof(retained->gyroSensScale));

	LOG_INF("Sensitivity calibration: axis %c scale set to %.5f", axis_char, (double)scale);
	if (off_axis_ratio > SENS_CAL_WARN_OFF_AXIS_RATIO) {
		printk(
			"Gyro sensitivity auto-calibration saved: axis %c, scale %.5f, equivalent sens diff over %u rev: %.3f deg, "
			"off-axis %.2f. Axis alignment was loose; repeating may improve accuracy.\n",
			axis_char,
			(double)scale,
			(unsigned int)CONFIG_SENSOR_SENS_REV,
			(double)equivalent_diff_deg,
			(double)off_axis_ratio
		);
	} else {
		printk(
			"Gyro sensitivity auto-calibration saved: axis %c, scale %.5f, equivalent sens diff over %u rev: %.3f deg, "
			"off-axis %.2f.\n",
			axis_char,
			(double)scale,
			(unsigned int)CONFIG_SENSOR_SENS_REV,
			(double)equivalent_diff_deg,
			(double)off_axis_ratio
		);
	}
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
}
#endif

#endif /* CONFIG_SENSOR_USE_SENS_CALIBRATION */
