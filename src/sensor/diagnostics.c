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
#include "diagnostics.h"
#include "sensor.h"

#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#if CONFIG_SENSOR_USE_VQF
#include "fusion/vqf/vqf.h"
#endif

LOG_MODULE_DECLARE(sensor, LOG_LEVEL_INF);

// Debug mode state
typedef struct {
	bool enabled;
	uint64_t session;
	int64_t start_time;
	int64_t duration_ms;
	uint32_t accel_count;    // Count accel samples since last output
	uint32_t output_every_n; // Output every N accel samples
	uint32_t output_count;   // Total output count
} sensor_debug_state_t;

static sensor_debug_state_t debug_state = {
	.enabled = false,
	.output_every_n = 4 // Default: output every 4 accel samples
};
static struct k_spinlock debug_lock;

/* Reset at the FIFO processing boundary, then retained through publish. */
static struct {
	uint64_t session;
	float debug_raw_g_sum[3];
	float debug_raw_a_sum[3];
	float debug_cal_g_sum[3];
	int debug_g_samples;
	int debug_a_samples;
	float debug_raw_m[3];
	float debug_cal_m[3];
	bool debug_mag_valid;
} frame;

#if CONFIG_SENSOR_RANGE_STATS
// Sensor range tracking state - records min/max values during runtime (not persisted)
static sensor_range_stats_t range_stats
	= {.gyro_max = {-INFINITY, -INFINITY, -INFINITY},
	   .gyro_min = {INFINITY, INFINITY, INFINITY},
	   .accel_max = {-INFINITY, -INFINITY, -INFINITY},
	   .accel_min = {INFINITY, INFINITY, INFINITY},
	   .sample_count = 0,
	   .initialized = false};
#endif // CONFIG_SENSOR_RANGE_STATS

#if CONFIG_SENSOR_RANGE_STATS
static float accel_actual_range;
static float gyro_actual_range;

static void sensor_update_range_stats_gyro(const float g[3]);
static void sensor_update_range_stats_accel(const float a[3]);

void sensor_diagnostics_set_ranges(float accel_range, float gyro_range)
{
	accel_actual_range = accel_range;
	gyro_actual_range = gyro_range;
}
#endif

/* Caller holds debug_lock: expiry and the counters it reports are one transition. */
static bool sensor_debug_expire_locked(int64_t now, uint32_t *output_count)
{
	if (debug_state.enabled && now - debug_state.start_time >= debug_state.duration_ms) {
		debug_state.enabled = false;
		*output_count = debug_state.output_count;
		return true;
	}
	return false;
}

static uint64_t sensor_debug_session(void)
{
	uint32_t output_count = 0;
	k_spinlock_key_t key = k_spin_lock(&debug_lock);
	bool expired = sensor_debug_expire_locked(k_uptime_get(), &output_count);
	uint64_t session = debug_state.enabled ? debug_state.session : 0;
	k_spin_unlock(&debug_lock, key);
	if (expired) {
		LOG_INF("Debug mode stopped. %u outputs generated", output_count);
	}
	return session;
}

static bool sensor_debug_frame_is_active(void)
{
	uint64_t session = sensor_debug_session();
	return session != 0 && session == frame.session;
}

/* Reserve/account the output before printing. A console restart may run during
 * slow output, but that output retains its old timestamp and never writes back
 * into the new session. Frames crossing a restart are discarded. */
static bool sensor_debug_reserve_output(int64_t *elapsed_ms)
{
	uint32_t output_count = 0;
	bool ready = false;
	k_spinlock_key_t key = k_spin_lock(&debug_lock);
	int64_t now = k_uptime_get();
	bool expired = sensor_debug_expire_locked(now, &output_count);
	if (debug_state.enabled && debug_state.session == frame.session) {
		debug_state.accel_count += frame.debug_a_samples;
		if (debug_state.accel_count >= debug_state.output_every_n) {
			debug_state.accel_count = 0;
			debug_state.output_count++;
			*elapsed_ms = now - debug_state.start_time;
			ready = true;
		}
	}
	k_spin_unlock(&debug_lock, key);
	if (expired) {
		LOG_INF("Debug mode stopped. %u outputs generated", output_count);
	}
	return ready;
}

void sensor_diagnostics_reset_frame(void)
{
	frame.debug_raw_g_sum[0] = 0;
	frame.debug_raw_g_sum[1] = 0;
	frame.debug_raw_g_sum[2] = 0;
	frame.debug_raw_a_sum[0] = 0;
	frame.debug_raw_a_sum[1] = 0;
	frame.debug_raw_a_sum[2] = 0;
	frame.debug_cal_g_sum[0] = 0;
	frame.debug_cal_g_sum[1] = 0;
	frame.debug_cal_g_sum[2] = 0;
	frame.debug_g_samples = 0;
	frame.debug_a_samples = 0;
	frame.debug_raw_m[0] = 0;
	frame.debug_raw_m[1] = 0;
	frame.debug_raw_m[2] = 0;
	frame.debug_cal_m[0] = 0;
	frame.debug_cal_m[1] = 0;
	frame.debug_cal_m[2] = 0;
	frame.debug_mag_valid = false;
	frame.session = sensor_debug_session();
}

void sensor_diagnostics_on_decoded_gyro(const float g[3])
{
	// Debug: Log gyro values to see if they're all zero
	static int gyro_log_count = 0;
	if (gyro_log_count < 10) {
		LOG_INF("Gyro raw: %.3f, %.3f, %.3f", (double)g[0], (double)g[1], (double)g[2]);
		gyro_log_count++;
	}
}

void sensor_diagnostics_on_raw_gyro(const float g[3])
{
	if (sensor_debug_frame_is_active()) {
		for (int j = 0; j < 3; j++) {
			frame.debug_raw_g_sum[j] += g[j];
		}
		frame.debug_g_samples++;
	}
}

void sensor_diagnostics_on_cal_gyro(const float g[3])
{
	// Accumulate calibrated gyro for debug (after zero bias and sensitivity calibration)
	if (sensor_debug_frame_is_active()) {
		for (int j = 0; j < 3; j++) {
			frame.debug_cal_g_sum[j] += g[j];
		}
	}

#if CONFIG_SENSOR_RANGE_STATS
	sensor_update_range_stats_gyro(g);
#endif
}

void sensor_diagnostics_on_raw_accel(const float a[3])
{
	if (sensor_debug_frame_is_active()) {
		for (int i = 0; i < 3; i++) {
			frame.debug_raw_a_sum[i] += a[i];
		}
		frame.debug_a_samples++;
	}
}

#if CONFIG_SENSOR_RANGE_STATS
void sensor_diagnostics_on_cal_accel(const float a[3])
{
	sensor_update_range_stats_accel(a);
}
#endif

void sensor_diagnostics_on_mag(const float raw[3], const float calibrated[3])
{
	if (sensor_debug_frame_is_active()) {
		memcpy(frame.debug_raw_m, raw, sizeof(frame.debug_raw_m));
		memcpy(frame.debug_cal_m, calibrated, sizeof(frame.debug_cal_m));
		frame.debug_mag_valid = true;
	}
}

void sensor_diagnostics_output(
	const float q[4], const float lin_a[3], const float avg_a[3], float temp, bool mag_enabled
)
{
#if !CONFIG_SENSOR_USE_VQF
	ARG_UNUSED(mag_enabled);
#endif
	// Debug mode output - based on accel sample count, not time interval
	if (sensor_debug_is_active() && frame.debug_a_samples > 0) {
		int64_t elapsed_ms;
		if (sensor_debug_reserve_output(&elapsed_ms)) {
			float elapsed_sec = (float)elapsed_ms / 1000.0f;

			// Calculate average raw and calibrated data
			float avg_raw_g[3] = {0};
			float avg_raw_a[3] = {0};
			float avg_cal_g[3] = {0};
			if (frame.debug_g_samples > 0) {
				for (int i = 0; i < 3; i++) {
					avg_raw_g[i] = frame.debug_raw_g_sum[i] / frame.debug_g_samples;
					avg_cal_g[i] = frame.debug_cal_g_sum[i] / frame.debug_g_samples;
				}
			}
			if (frame.debug_a_samples > 0) {
				for (int i = 0; i < 3; i++) {
					avg_raw_a[i] = frame.debug_raw_a_sum[i] / frame.debug_a_samples;
				}
			}

			// Get VQF debug info
#if CONFIG_SENSOR_USE_VQF
			vqf_debug_info_t vqf_info;
			vqf_get_debug_info(&vqf_info);
#endif

			// Compact output format with raw, calibrated, and fused data
			printk(
				"[%.2fs] RAW: A[%.3f,%.3f,%.3f] G[%.2f,%.2f,%.2f] T:%.2fC\n",
				(double)elapsed_sec,
				(double)avg_raw_a[0],
				(double)avg_raw_a[1],
				(double)avg_raw_a[2],
				(double)avg_raw_g[0],
				(double)avg_raw_g[1],
				(double)avg_raw_g[2],
				(double)temp
			);

			printk(
				"     CAL: A[%.3f,%.3f,%.3f] G[%.2f,%.2f,%.2f]\n",
				(double)avg_a[0],
				(double)avg_a[1],
				(double)avg_a[2],
				(double)avg_cal_g[0],
				(double)avg_cal_g[1],
				(double)avg_cal_g[2]
			);

			if (frame.debug_mag_valid) {
				printk(
					"     MAG: RAW[%.2f,%.2f,%.2f] CAL[%.2f,%.2f,%.2f]\n",
					(double)frame.debug_raw_m[0],
					(double)frame.debug_raw_m[1],
					(double)frame.debug_raw_m[2],
					(double)frame.debug_cal_m[0],
					(double)frame.debug_cal_m[1],
					(double)frame.debug_cal_m[2]
				);
			}

#if CONFIG_SENSOR_USE_VQF
			printk(
				"     VQF: Q[%.3f,%.3f,%.3f,%.3f] LinA[%.2f,%.2f,%.2f]\n",
				(double)q[0],
				(double)q[1],
				(double)q[2],
				(double)q[3],
				(double)lin_a[0],
				(double)lin_a[1],
				(double)lin_a[2]
			);
#if SENSOR_DEBUG_QDEV_QOUT
			float debug_device_quat[4];
			float debug_reported_quat[4];
			sensor_compute_device_and_reported_quat(q, debug_device_quat, debug_reported_quat);
			printk(
				"     Qdev[%.3f,%.3f,%.3f,%.3f] Qout[%.3f,%.3f,%.3f,%.3f]\n",
				(double)debug_device_quat[0],
				(double)debug_device_quat[1],
				(double)debug_device_quat[2],
				(double)debug_device_quat[3],
				(double)debug_reported_quat[0],
				(double)debug_reported_quat[1],
				(double)debug_reported_quat[2],
				(double)debug_reported_quat[3]
			);
#endif
			printk(
				"     Rest:%c RestDev[G:%.3f,A:%.3f] Bias[%.3f,%.3f,%.3f]°/s Sigma:%.3f°/s Delta:%.2f°\n",
				vqf_info.rest_detected ? 'Y' : 'N',
				(double)vqf_info.rest_deviations[0],
				(double)vqf_info.rest_deviations[1],
				(double)vqf_info.bias[0],
				(double)vqf_info.bias[1],
				(double)vqf_info.bias[2],
				(double)vqf_info.bias_sigma,
				(double)vqf_info.delta
			);
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
			printk(
				"     Adapt: tauAcc:%.2fs motInt:%.3f\n",
				(double)vqf_info.tau_acc,
				(double)vqf_info.motion_intensity
			);
#endif
			printk(
				"     RestDiag: enter:%u exit:%u total:%.1fs last:%.1fs up:%.0fs rest%%:%.1f\n",
				vqf_info.rest_enter_count,
				vqf_info.rest_exit_count,
				(double)vqf_info.rest_total_s,
				(double)vqf_info.rest_last_duration_s,
				(double)vqf_info.uptime_s,
				(double)(vqf_info.uptime_s > 0 ? 100.0f * vqf_info.rest_total_s / vqf_info.uptime_s : 0)
			);
			printk(
				"     BiasP[%.1f,%.1f,%.1f]\n",
				(double)vqf_info.biasP[0],
				(double)vqf_info.biasP[1],
				(double)vqf_info.biasP[2]
			);
			{
				uint8_t n = vqf_info.rest_event_count;
				if (n > 8) {
					n = 8;
				}
				if (n > 0) {
					printk("     RestLog(%u events):", vqf_info.rest_event_count);
					for (uint8_t ri = 0; ri < n; ri++) {
						printk(
							" %s@%.0fs",
							vqf_info.rest_events[ri].entered ? "EN" : "EX",
							(double)vqf_info.rest_events[ri].time_s
						);
					}
					printk("\n");
				}
			}
			if (mag_enabled) {
				printk(
					"     Mag: DisAng:%.2f° CorrRate:%.2f°/s\n",
					(double)vqf_info.mag_dis_angle,
					(double)vqf_info.mag_corr_rate
				);
				printk(
					"     MagDist:%c MagRefNorm:%.3f MagRefDip:%.2f° MagNorm:%.3f MagDip:%.2f°\n",
					vqf_info.mag_dist_detected ? 'Y' : 'N',
					(double)vqf_info.mag_ref_norm,
					(double)vqf_info.mag_ref_dip,
					(double)vqf_info.mag_norm,
					(double)vqf_info.mag_dip
				);
				printk(
					"     MagT: undist:%.2fs reject:%.2fs candT:%.2fs candNorm:%.3f candDip:%.2f°\n",
					(double)vqf_info.mag_undisturbed_t,
					(double)vqf_info.mag_reject_t,
					(double)vqf_info.mag_candidate_t,
					(double)vqf_info.mag_candidate_norm,
					(double)vqf_info.mag_candidate_dip
				);
			}
#else
			printk(
				"     Q[%.3f,%.3f,%.3f,%.3f] LinA[%.2f,%.2f,%.2f]\n",
				(double)q[0],
				(double)q[1],
				(double)q[2],
				(double)q[3],
				(double)lin_a[0],
				(double)lin_a[1],
				(double)lin_a[2]
			);
#if SENSOR_DEBUG_QDEV_QOUT
			float debug_device_quat[4];
			float debug_reported_quat[4];
			sensor_compute_device_and_reported_quat(q, debug_device_quat, debug_reported_quat);
			printk(
				"     Qdev[%.3f,%.3f,%.3f,%.3f] Qout[%.3f,%.3f,%.3f,%.3f]\n",
				(double)debug_device_quat[0],
				(double)debug_device_quat[1],
				(double)debug_device_quat[2],
				(double)debug_device_quat[3],
				(double)debug_reported_quat[0],
				(double)debug_reported_quat[1],
				(double)debug_reported_quat[2],
				(double)debug_reported_quat[3]
			);
#endif
#endif
		}
	}
}

// Debug mode control functions
void sensor_debug_start(uint32_t duration_sec)
{
	if (duration_sec == 0 || duration_sec > SENSOR_DEBUG_MAX_DURATION_SEC) {
		duration_sec = 10; // Default to 10 seconds
	}

	k_spinlock_key_t key = k_spin_lock(&debug_lock);
	debug_state.session++;
	if (debug_state.session == 0) {
		debug_state.session++; // Zero denotes an inactive frame.
	}
	debug_state.enabled = true;
	debug_state.start_time = k_uptime_get();
	debug_state.duration_ms = duration_sec * 1000;
	debug_state.accel_count = 0;
	debug_state.output_count = 0;
	uint32_t output_every_n = debug_state.output_every_n;
	k_spin_unlock(&debug_lock, key);

	float accel_odr = sensor_get_accel_odr();
	LOG_INF(
		"Debug mode started for %u seconds (accel ODR: %.1fHz, output every %u samples)",
		duration_sec,
		(double)accel_odr,
		output_every_n
	);
}

void sensor_debug_stop(void)
{
	k_spinlock_key_t key = k_spin_lock(&debug_lock);
	bool enabled = debug_state.enabled;
	debug_state.enabled = false;
	uint32_t output_count = debug_state.output_count;
	k_spin_unlock(&debug_lock, key);
	if (enabled) {
		LOG_INF("Debug mode stopped. %u outputs generated", output_count);
	}
}

bool sensor_debug_is_active(void)
{
	return sensor_debug_session() != 0;
}

#if CONFIG_SENSOR_RANGE_STATS
// Sensor range tracking functions
const sensor_range_stats_t *sensor_get_range_stats(void)
{
	return &range_stats;
}

void sensor_reset_range_stats(void)
{
	for (int i = 0; i < 3; i++) {
		range_stats.gyro_max[i] = -INFINITY;
		range_stats.gyro_min[i] = INFINITY;
		range_stats.accel_max[i] = -INFINITY;
		range_stats.accel_min[i] = INFINITY;
	}
	range_stats.sample_count = 0;
	range_stats.initialized = false;
	LOG_INF("Range statistics reset");
}

// Internal function to update range statistics with new gyro data
static void sensor_update_range_stats_gyro(const float g[3])
{
	if (!range_stats.initialized) {
		range_stats.initialized = true;
	}
	for (int i = 0; i < 3; i++) {
		if (g[i] > range_stats.gyro_max[i]) {
			range_stats.gyro_max[i] = g[i];
		}
		if (g[i] < range_stats.gyro_min[i]) {
			range_stats.gyro_min[i] = g[i];
		}
	}
}

// Internal function to update range statistics with new accel data
static void sensor_update_range_stats_accel(const float a[3])
{
	if (!range_stats.initialized) {
		range_stats.initialized = true;
	}
	for (int i = 0; i < 3; i++) {
		if (a[i] > range_stats.accel_max[i]) {
			range_stats.accel_max[i] = a[i];
		}
		if (a[i] < range_stats.accel_min[i]) {
			range_stats.accel_min[i] = a[i];
		}
	}
	range_stats.sample_count++;
}

void sensor_print_range_stats(void)
{
	if (!range_stats.initialized) {
		printk("Range statistics not initialized (no data collected yet)\n");
		return;
	}

	printk("\n=== Sensor Range Statistics ===\n");
	printk("Total samples: %llu\n", range_stats.sample_count);

	printk("\nGyroscope (deg/s):\n");
	printk(
		"  X: min=%.2f, max=%.2f, peak=%.2f\n",
		(double)range_stats.gyro_min[0],
		(double)range_stats.gyro_max[0],
		(double)fmaxf(fabsf(range_stats.gyro_min[0]), fabsf(range_stats.gyro_max[0]))
	);
	printk(
		"  Y: min=%.2f, max=%.2f, peak=%.2f\n",
		(double)range_stats.gyro_min[1],
		(double)range_stats.gyro_max[1],
		(double)fmaxf(fabsf(range_stats.gyro_min[1]), fabsf(range_stats.gyro_max[1]))
	);
	printk(
		"  Z: min=%.2f, max=%.2f, peak=%.2f\n",
		(double)range_stats.gyro_min[2],
		(double)range_stats.gyro_max[2],
		(double)fmaxf(fabsf(range_stats.gyro_min[2]), fabsf(range_stats.gyro_max[2]))
	);

	// Calculate overall peak gyro value
	float gyro_peak = 0;
	for (int i = 0; i < 3; i++) {
		float axis_peak = fmaxf(fabsf(range_stats.gyro_min[i]), fabsf(range_stats.gyro_max[i]));
		if (axis_peak > gyro_peak) {
			gyro_peak = axis_peak;
		}
	}
	// Use actual range if available, otherwise fall back to config value
	float gyro_fs = (gyro_actual_range > 0) ? gyro_actual_range : (float)CONFIG_SENSOR_GYRO_FS;
	printk("  Overall peak: %.2f deg/s (FS=%.0f)\n", (double)gyro_peak, (double)gyro_fs);
	if (gyro_peak > gyro_fs * 0.9f) {
		printk("  WARNING: Peak value exceeds 90%% of full scale!\n");
	}

	printk("\nAccelerometer (g):\n");
	printk(
		"  X: min=%.3f, max=%.3f, peak=%.3f\n",
		(double)range_stats.accel_min[0],
		(double)range_stats.accel_max[0],
		(double)fmaxf(fabsf(range_stats.accel_min[0]), fabsf(range_stats.accel_max[0]))
	);
	printk(
		"  Y: min=%.3f, max=%.3f, peak=%.3f\n",
		(double)range_stats.accel_min[1],
		(double)range_stats.accel_max[1],
		(double)fmaxf(fabsf(range_stats.accel_min[1]), fabsf(range_stats.accel_max[1]))
	);
	printk(
		"  Z: min=%.3f, max=%.3f, peak=%.3f\n",
		(double)range_stats.accel_min[2],
		(double)range_stats.accel_max[2],
		(double)fmaxf(fabsf(range_stats.accel_min[2]), fabsf(range_stats.accel_max[2]))
	);

	// Calculate overall peak accel value
	float accel_peak = 0;
	for (int i = 0; i < 3; i++) {
		float axis_peak = fmaxf(fabsf(range_stats.accel_min[i]), fabsf(range_stats.accel_max[i]));
		if (axis_peak > accel_peak) {
			accel_peak = axis_peak;
		}
	}
	// Use actual range if available, otherwise fall back to config value
	float accel_fs = (accel_actual_range > 0) ? accel_actual_range : (float)CONFIG_SENSOR_ACCEL_FS;
	printk("  Overall peak: %.3f g (FS=%.0f)\n", (double)accel_peak, (double)accel_fs);
	if (accel_peak > accel_fs * 0.9f) {
		printk("  WARNING: Peak value exceeds 90%% of full scale!\n");
	}

	printk("================================\n");
}
#endif // CONFIG_SENSOR_RANGE_STATS
