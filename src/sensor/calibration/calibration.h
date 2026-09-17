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

#include <stdbool.h>
#include <stdint.h>

/* Sensor feeds data to calibration */
void sensor_calibration_process_accel(float a[3]);
void sensor_calibration_process_gyro(float g[3]);
void sensor_calibration_process_mag(float m[3]);

void sensor_calibration_update_sensor_ids(int imu);
uint8_t sensor_calibration_get_imu_id(void);
uint8_t *sensor_calibration_get_sensor_data();

void sensor_calibration_read(void);

typedef struct {
	float accel_bias[3];
	float gyro_bias[3];
	float accel_matrix[4][3];
} sensor_imu_calibration_t;

/* Coherent applied coefficients, never a writable view of owner storage. */
void sensor_calibration_snapshot(sensor_imu_calibration_t *out);
/* Nonblocking submission: -EAGAIN without a ready consumer, -EBUSY for an
 * occupied transaction/reset-all barrier, -ESHUTDOWN once terminally closed.
 * Accepted candidates survive suspend/failed rescan for recovery or power drain;
 * reset-all cancels them. Application remains at a sensor frame boundary. */
int sensor_calibration_commit_bias(const float a_bias[3], const float g_bias[3], bool persist_gyro, uint16_t operation_id);
int sensor_calibration_commit_accel(const float matrix[4][3], uint16_t operation_id);
int sensor_calibration_reset_imu(void);
int sensor_calibration_reset_accel(void);
/* Power owner calls only after the sensor is quiescent. No live fusion mutation. */
void sensor_calibration_prepare_power_down(void);
/* System reset-all barrier: call before storage lock, end after releasing it.
 * Waits for in-flight persistence and cancels all pre-clear IMU transactions. */
void sensor_calibration_clear_begin(void);
void sensor_calibration_clear_end(void);

int sensor_calibration_set_sensitivity(const float degrees[3]);
int sensor_calibration_reset_sensitivity(void);
int sensor_calibration_validate_mag(float m_inv[][3], bool write);

/* Candidate initialization only; live coefficients change through commits. */
void sensor_calibration_identity_accel(float matrix[4][3]);
void sensor_calibration_clear_mag(float m_inv[][3], bool write); // "request" mag cal

enum sensor_calibration_request_id {
	CAL_REQUEST_CLEAR = -1,
	CAL_REQUEST_QUERY = 0,
	CAL_REQUEST_IMU = 1,
	CAL_REQUEST_ACCEL_6_SIDE = 2,
	CAL_REQUEST_TCAL_BOOT = 3,
	CAL_REQUEST_TCAL_RUNTIME = 4,
	CAL_REQUEST_GYRO_SENS = 5,
	/* Occupies the shared request slot throughout manual mag collection. */
	CAL_REQUEST_MAG = 6,
};

enum cal_request_origin {
	CAL_REQUEST_USER = 0,
	CAL_REQUEST_AUTO = 1,
	CAL_REQUEST_AUTO_SILENT = 2,
};

/* QUERY returns the pending request ID, or 0 when idle. CLEAR ends sample
 * admission and clears the slot, not a candidate already handed to its owner. */
int sensor_calibration_request(int id, enum cal_request_origin origin);
uint16_t sensor_calibration_current_operation(void);

void sensor_request_calibration(void);
void sensor_request_calibration_6_side(void);
void sensor_request_calibration_mag(void);
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
int sensor_request_calibration_sens(uint8_t axis, uint16_t revolutions);
#endif
/* Always service ownership, even when disabled. Gravity is reliable independent
 * 6D up expressed in the raw magnetometer axis basis, never magnetic heading. */
void sensor_calibration_online_mag_sample(const float raw[3], const float gravity_raw[3], bool gravity_valid);
int sensor_calibration_online_mag_status(float *dir_bias);
void sensor_calibration_track_mag_norm(float cal_norm);
float sensor_calibration_get_mag_quality(void);
void sensor_calibration_set_online_mag_enabled(bool enabled);
bool sensor_calibration_get_online_mag_enabled(void);
void sensor_calibration_online_mag_retained_save(void);
void sensor_calibration_online_mag_retained_clear(void);
void sensor_calibration_online_mag_cold_start(void);
void sensor_calibration_online_mag_prepare_power_down(void);

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
bool sensor_tcal_needs_nearby_point(float temp, float *closest_temp, float *distance_c);
void sensor_tcal_check_auto_calibration(float current_temp);
void sensor_tcal_set_auto_calibration(bool enabled);
bool sensor_tcal_get_auto_calibration(void);

// T-Cal compensation enable/disable (persisted)
void sensor_tcal_set_enabled(bool enabled);
bool sensor_tcal_get_enabled(void);

/* What process_gyro actually subtracts right now (flag vs curve readiness). */
typedef enum {
	SENSOR_TCAL_APPLY_DISABLED = 0,   /* flag off → static ZRO */
	SENSOR_TCAL_APPLY_ZRO_FALLBACK,   /* flag on, points < MLS min → ZRO */
	SENSOR_TCAL_APPLY_CURVE,          /* LUT/MLS active */
} sensor_tcal_apply_mode_t;

sensor_tcal_apply_mode_t sensor_tcal_get_apply_mode(void);
const char *sensor_tcal_get_apply_mode_name(void);
/* Hot-path: cached (enabled && points>=min). Updated on enable/point changes. */
bool sensor_tcal_curve_apply_ready(void);
void sensor_tcal_refresh_apply_cache(void);

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
