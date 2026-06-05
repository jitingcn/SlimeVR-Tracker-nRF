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
#ifndef SLIMENRF_SENSOR_CALIBRATION
#define SLIMENRF_SENSOR_CALIBRATION

/* Sensor feeds data to calibration */
void sensor_calibration_process_accel(float a[3]);
void sensor_calibration_process_gyro(float g[3]);
void sensor_calibration_process_mag(float m[3]);

void sensor_calibration_update_sensor_ids(int imu);
uint8_t *sensor_calibration_get_sensor_data();

void sensor_calibration_read(void);

int sensor_calibration_validate(float *a_bias, float *g_bias, bool write);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
int sensor_calibration_validate_6_side(float a_inv[][3], bool write);
#endif
int sensor_calibration_validate_mag(float m_inv[][3], bool write);

void sensor_calibration_clear(float *a_bias, float *g_bias, bool write);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
void sensor_calibration_clear_6_side(float a_inv[][3], bool write);
#endif
void sensor_calibration_clear_mag(float m_inv[][3], bool write); // "request" mag cal

void sensor_request_calibration(void);
void sensor_request_calibration_6_side(void);
void sensor_request_calibration_mag(void);
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
int sensor_request_calibration_sens(uint8_t axis, uint16_t revolutions);
#endif
void sensor_calibration_online_mag_sample(const float m[3]);
int sensor_calibration_online_mag_status(float *dir_bias);
void sensor_calibration_track_mag_norm(float cal_norm);
float sensor_calibration_get_mag_quality(void);
void sensor_calibration_online_mag_retained_save(void);
void sensor_calibration_online_mag_retained_clear(void);
void sensor_calibration_online_mag_cold_start(void);

#if CONFIG_SENSOR_USE_TCAL

// Quality assessment structure for T-Cal
typedef struct {
	bool curve_valid;          // Curve is valid
	uint16_t point_count;      // Number of calibration points
	float curve_error;         // Fitting error
	float temp_min;            // Minimum calibrated temperature
	float temp_max;            // Maximum calibrated temperature
	bool temp_in_range;        // Current temperature is in range
} tcal_quality_t;

void sensor_calibration_get_last_gyro_offset(float offset[3]);

// T-Cal maintenance/status
void sensor_tcal_clear(void);
void sensor_tcal_status(void);
void sensor_tcal_remove_point(int index_to_remove);
bool sensor_tcal_is_temp_outside_range(float temp, float *min_temp, float *max_temp);
void sensor_tcal_check_auto_calibration(float current_temp);
void sensor_tcal_set_auto_calibration(bool enabled);
bool sensor_tcal_get_auto_calibration(void);

// T-Cal compensation enable/disable (persisted)
void sensor_tcal_set_enabled(bool enabled);
bool sensor_tcal_get_enabled(void);

// Continuous bucket-based T-Cal sampling
void sensor_tcal_feed_continuous_sample(const float g[3], float temp);
void sensor_tcal_continuous_motion_detected(void);

// Quality assessment function - returns true if quality is sufficient
bool sensor_tcal_assess_quality(float current_temp, tcal_quality_t *quality);

// Boot calibration functions
void sensor_tcal_boot_calibration_check(void);
void sensor_boot_cal_set_enabled(bool enabled);
bool sensor_boot_cal_is_completed(void);
void sensor_boot_cal_get_doffset(float offset[3]);
void sensor_boot_cal_reset(void); // Reset boot calibration state (call before reboot/shutdown, not before WoM)

// Runtime periodic calibration functions
void sensor_runtime_calibration_check(bool is_resting);
void sensor_runtime_cal_get_status(int64_t *last_cal_time, int64_t *rest_duration);

// Test function for comparing calibration methods
void sensor_tcal_test_methods(float temp);
#endif

#endif
