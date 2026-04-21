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
#ifndef SLIMENRF_SENSOR
#define SLIMENRF_SENSOR

#include "interface.h"

const char* sensor_get_sensor_imu_name(void);
const char* sensor_get_sensor_mag_name(void);
const char* sensor_get_sensor_fusion_name(void);
bool sensor_is_initialized(void);

int sensor_get_sensor_temperature(float *);

int sensor_request_scan(bool force);

void sensor_scan_read(void);
void sensor_scan_write(void);
void sensor_scan_clear(void);

void sensor_retained_read(void);
void sensor_retained_write(void);

void sensor_shutdown(void);
uint8_t sensor_setup_WOM(void);

void sensor_set_mag_enabled(bool enabled);
bool sensor_get_mag_enabled(void);
bool sensor_get_mag_available(void);
bool sensor_get_mag_calibrated(void);
void sensor_refresh_sensor_ids(void);
void sensor_mag_ref_reset(void);

void sensor_fusion_invalidate(void);
void sensor_fusion_update_bias(float *g_off);

void wait_for_threads(void);
void main_imu_suspend(void);
void main_imu_resume(void);
void main_imu_wakeup(void);
void main_imu_restart(void);

#if CONFIG_SENSOR_USE_TCAL
float sensor_get_current_imu_temperature(void);
#endif

// Get actual sensor ODR (Output Data Rate) in Hz
float sensor_get_accel_odr(void);
float sensor_get_gyro_odr(void);

// Debug mode functions
void sensor_debug_start(uint32_t duration_sec);
void sensor_debug_stop(void);
bool sensor_debug_is_active(void);

#if CONFIG_SENSOR_RANGE_STATS
// Sensor range tracking - records min/max values during runtime (not persisted)
typedef struct {
	float gyro_max[3];   // Maximum gyro values per axis (deg/s)
	float gyro_min[3];   // Minimum gyro values per axis (deg/s)
	float accel_max[3];  // Maximum accel values per axis (g)
	float accel_min[3];  // Minimum accel values per axis (g)
	uint64_t sample_count;  // Total samples processed
	bool initialized;    // Whether tracking has been initialized
} sensor_range_stats_t;

// Get the current range statistics
const sensor_range_stats_t* sensor_get_range_stats(void);
// Reset range statistics
void sensor_reset_range_stats(void);
// Print range statistics to console
void sensor_print_range_stats(void);
#endif // CONFIG_SENSOR_RANGE_STATS

typedef struct sensor_fusion {
	void (*init)(float, float, float);  // gyro_time, accel_time, mag_time
	void (*load)(const void*);
	void (*save)(void*);

	void (*update_gyro)(float*, float);  // deg/s
	void (*update_accel)(float*, float);  // g
	void (*update_mag)(float*, float);  // any unit (usually gauss)
	void (*update)(float*, float*, float*, float);

	void (*get_gyro_bias)(float*);
	void (*set_gyro_bias)(float*);

	void (*update_gyro_sanity)(float*, float*);
	int (*get_gyro_sanity)(void);

	void (*get_lin_a)(float*);
	void (*get_quat)(float*);
} sensor_fusion_t;

typedef struct sensor_imu {
	int (*init)(float, float, float, float*, float*); // first float is clock_rate, nonzero means use CLKIN, return update time, return 0 if success, -1 if general error
	void (*shutdown)(void);

	void (*update_fs)(float, float, float*, float*); // return actual range
	int (*update_odr)(float, float, float*, float*); // return actual update time, return 0 if success, 1 if odr is same, -1 if general error

	uint16_t (*fifo_read)(uint8_t*, uint16_t);
	int (*fifo_process)(uint16_t, uint8_t*, float[3], float[3]); // g, deg/s
	void (*accel_read)(float[3]); // g
	void (*gyro_read)(float[3]); // deg/s
	float (*temp_read)(void); // deg C

	uint8_t (*setup_DRDY)(uint16_t);
	uint8_t (*setup_WOM)(void);

	int (*ext_setup)(void); // register write/writeread with interface, return 0 if success, -1 if error or not available
	int (*ext_passthrough)(bool); // enable/disable passthrough mode, return 0 if success, -1 if error or not available
} sensor_imu_t;

typedef struct sensor_mag {
	int (*init)(float, float*); // return update time, return 0 if success, 1 if general error
	void (*shutdown)(void);

	int (*update_odr)(float, float*); // return actual update time, return 0 if success, 1 if odr is same, -1 if general error

	void (*mag_oneshot)(void); // trigger oneshot if exists
	bool (*mag_read)(float[3]); // any unit (usually gauss); returns true if new data was available
	float (*temp_read)(float[3]); // deg C

	void (*mag_process)(uint8_t*, float[3]); // use if magnetometer is present as an auxiliary sensor, from data read by IMU
	uint8_t ext_min_burst; // minimum supported burst length for external interface
	uint8_t ext_burst; // default supported burst length
} sensor_mag_t;

#endif
