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
#ifndef SLIMENRF_SENSOR_DIAGNOSTICS
#define SLIMENRF_SENSOR_DIAGNOSTICS

#include <stdbool.h>
#include <stdint.h>

// Set to 1 to temporarily enable the extra Qdev/Qout debug line
#ifndef SENSOR_DEBUG_QDEV_QOUT
#define SENSOR_DEBUG_QDEV_QOUT 0
#endif

#define SENSOR_DEBUG_MAX_DURATION_SEC 60U

/* Console-safe session transitions. Expiry and output accounting are atomic;
 * already reserved output can finish printing after a stop/restart. */
void sensor_debug_start(uint32_t duration_sec);
void sensor_debug_stop(void);
bool sensor_debug_is_active(void);

#if CONFIG_SENSOR_RANGE_STATS
// Sensor range tracking - records min/max values during runtime (not persisted)
typedef struct {
	float gyro_max[3];     // Maximum gyro values per axis (deg/s)
	float gyro_min[3];     // Minimum gyro values per axis (deg/s)
	float accel_max[3];    // Maximum accel values per axis (g)
	float accel_min[3];    // Minimum accel values per axis (g)
	uint64_t sample_count; // Total samples processed
	bool initialized;      // Whether tracking has been initialized
} sensor_range_stats_t;

// Get the current range statistics
const sensor_range_stats_t *sensor_get_range_stats(void);
// Reset range statistics
void sensor_reset_range_stats(void);
// Print range statistics to console
void sensor_print_range_stats(void);
#endif // CONFIG_SENSOR_RANGE_STATS

/* Sample hooks and frame output are owned exclusively by the sensor thread.
 * A frame belongs to the session captured at reset; restarting discards an
 * in-progress frame rather than counting its samples into the new session. */
void sensor_diagnostics_reset_frame(void);
void sensor_diagnostics_on_decoded_gyro(const float g[3]);
void sensor_diagnostics_on_raw_gyro(const float g[3]);
void sensor_diagnostics_on_cal_gyro(const float g[3]);
void sensor_diagnostics_on_raw_accel(const float a[3]);
void sensor_diagnostics_on_mag(const float raw[3], const float calibrated[3]);
void sensor_diagnostics_output(
	const float q[4], const float lin_a[3], const float avg_a[3], float temp, bool mag_enabled
);

#if CONFIG_SENSOR_RANGE_STATS
void sensor_diagnostics_on_cal_accel(const float a[3]);
void sensor_diagnostics_set_ranges(float accel_range, float gyro_range);
#endif

#if SENSOR_DEBUG_QDEV_QOUT
/* Frame correction remains sensor-owned; computed only for a debug output. */
void sensor_compute_device_and_reported_quat(
	const float *fused_quat, float *device_quat, float *reported_quat
);
#endif

#endif
