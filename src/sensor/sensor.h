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

const char *sensor_get_sensor_imu_name(void);
const char *sensor_get_sensor_mag_name(void);
const char *sensor_get_sensor_fusion_name(void);
bool sensor_is_initialized(void);

int sensor_get_sensor_temperature(float *);

int sensor_request_scan(bool force);

void sensor_scan_read(void);
void sensor_scan_write(void);
void sensor_scan_clear(void);

void sensor_retained_read(void);
void sensor_retained_write(void);
void sensor_record_wom_sleep(void);

void sensor_shutdown(void);
uint8_t sensor_setup_WOM(void);

void sensor_set_mag_enabled(bool enabled);
bool sensor_get_mag_enabled(void);
bool sensor_get_mag_available(void);
bool sensor_get_mag_calibrated(void);
void sensor_refresh_sensor_ids(void);
void sensor_mag_ref_reset(void);

/* Fusion policy accessors (backend-agnostic; prefer over vqf_* / eqf_*). */
bool sensor_fusion_get_rest_detected(void);
bool sensor_fusion_get_mag_dist_detected(void);
/* Thread-safe requests, applied only by sensor before the next magnetic feed. */
void sensor_fusion_reset_mag_ref(void);
void sensor_fusion_set_mag_ref(float norm, float dip);
bool sensor_fusion_get_mag_ref(float *norm, float *dip);

/* Nonblocking requests consumed by the sensor at the next frame boundary. */
void sensor_request_fusion_reset(void);
void sensor_request_fusion_bias_reset(void);

void wait_for_threads(void);
void main_imu_suspend(void);
bool main_imu_is_suspended(void);
void main_imu_resume(void);
void main_imu_wakeup(void);
void main_imu_restart(void);

#if CONFIG_SENSOR_USE_TCAL
float sensor_get_current_imu_temperature(void);
#endif

// Get actual sensor ODR (Output Data Rate) in Hz
float sensor_get_accel_odr(void);
float sensor_get_gyro_odr(void);
float sensor_get_mag_odr(void);     /* driver-reported Hz; 0.0f => n/a */
float sensor_get_mag_feed_hz(void); /* measured fusion feed Hz; 0.0f => n/a */
float sensor_get_fusion_rate(void); /* effective gyro feed into fusion */

/* Batch raw collection: fractional-rate emission, 0 = accelerometer ODR. */
void sensor_set_batch_collect(bool active, float emit_hz);
float sensor_get_processing_work_time_ms(void); /* processing-work EMA, not full loop period */

#include "diagnostics.h"

typedef struct sensor_fusion {
	void (*init)(float, float, float); // gyro_time, accel_time, mag_time
	void (*load)(const void *);
	void (*save)(void *);

	void (*update_gyro)(float *, float);  // deg/s
	void (*update_accel)(float *, float); // g
	void (*update_mag)(float *, float);   // any unit (usually gauss)
	void (*update)(float *, float *, float *, float);

	void (*get_gyro_bias)(float *);
	void (*set_gyro_bias)(float *);
	/* Input coordinate change (new input = old + delta), not a measurement. */
	void (*rebase_gyro_bias)(const float delta_dps[3]);

	void (*update_gyro_sanity)(float *, float *);
	int (*get_gyro_sanity)(void);

	void (*get_lin_a)(float *);
	void (*get_quat)(float *);
	/* NULL when no magnetically independent inclination estimate is available. */
	void (*get_quat6)(float *);

	/* Rest / mag-quality policy (both VQF and EqF implement these). */
	bool (*get_rest_detected)(void);
	/* Consume one new detector observation; NULL discards it. Call only from
	 * the sensor owner, or an exclusive lifecycle owner after the sensor
	 * thread is confirmed stopped/suspended (not merely requested to stop). */
	bool (*take_rest_observation)(bool *out);
	bool (*get_mag_dist_detected)(void);
	/* Sensor-thread only: replace magnetic domain, preserving attitude/bias. */
	void (*rebase_mag)(float norm, float dip);
	void (*get_mag_ref)(float *norm, float *dip);
} sensor_fusion_t;

/* How the MCU reaches an auxiliary sensor through the selected IMU.
 * External-sensor FIFO delivery is a separate, currently unsupported contract. */
enum sensor_ext_mode {
	SENSOR_EXT_MODE_OFF,
	SENSOR_EXT_MODE_I2C_PASSTHROUGH,
	SENSOR_EXT_MODE_I2CM_PROXY,
};

typedef struct sensor_imu {
	/* Periods are in seconds; a positive clock rate requests external CLKIN
	 * where supported. Return 0 on success, negative on failure. */
	int (*init)(
		float clock_rate_hz,
		float accel_period_s,
		float gyro_period_s,
		float *actual_accel_period_s,
		float *actual_gyro_period_s
	);
	void (*shutdown)(void);

	/* Requested and actual full-scale ranges: g for accel, deg/s for gyro. */
	void (*update_fs)(
		float accel_range_g,
		float gyro_range_dps,
		float *actual_accel_range_g,
		float *actual_gyro_range_dps
	);
	/* Return 0 on success, including an unchanged configuration; negative on
	 * failure. Off/standby support for <= 0 and INFINITY is driver-specific.
	 * Actual-period outputs are not guaranteed on failure. */
	int (*update_odr)(
		float accel_period_s,
		float gyro_period_s,
		float *actual_accel_period_s,
		float *actual_gyro_period_s
	);

	/* Buffer capacity is in bytes; return the number of complete driver
	 * records available to fifo_process (possibly a prefix on I/O failure). */
	uint16_t (*fifo_read)(uint8_t *data, uint16_t capacity_bytes);
	/* Index selects a driver record, not a byte offset. Return 0 to consume,
	 * nonzero to skip. Initialize both vectors before calling: a record may
	 * update only one channel; invalid-channel handling is driver-specific. */
	int (*fifo_process)(uint16_t packet_index, uint8_t *data, float accel_g[3], float gyro_dps[3]);
	void (*accel_read)(float accel_g[3]);
	void (*gyro_read)(float gyro_dps[3]);
	float (*temp_read)(void); // deg C; failure sentinel is driver-specific

	/* Return packed nRF GPIO pull (high nibble) and sense (low nibble)
	 * settings. Zero means unavailable; some drivers only log I/O failures
	 * and still return pin settings. Watermark encoding is chip-specific. */
	uint8_t (*setup_DRDY)(uint16_t threshold);
	uint8_t (*setup_WOM)(void);

	int (*ext_setup)(enum sensor_ext_mode mode); // select auxiliary route; 0 on success, negative on error/unsupported
} sensor_imu_t;

typedef struct sensor_mag {
	/* Periods are in seconds. Return 0 on success, negative on failure;
	 * actual-period output is not guaranteed on failure. */
	int (*init)(float period_s, float *actual_period_s);
	void (*shutdown)(void);

	/* Return 0 on success, including an unchanged configuration; negative on
	 * failure. <= 0 and INFINITY may select idle, oneshot or a supported
	 * continuous rate, depending on the chip. Check the actual period. */
	int (*update_odr)(float period_s, float *actual_period_s);

	void (*mag_oneshot)(void); // trigger a single measurement where supported
	/* True means the output was updated; freshness checks are driver-specific.
	 * Vector units are driver-specific (usually gauss). */
	bool (*mag_read)(float mag[3]);
	/* Return deg C. Drivers with SET/RESET offset measurement may update bias
	 * in the same units as mag_read; other drivers leave it untouched. */
	float (*temp_read)(float bias[3]);

	/* Decode the chip-specific raw magnetic payload, without initiating a
	 * bus read. Frame/status validation belongs to the acquisition path. */
	void (*mag_process)(uint8_t *raw_mag, float mag[3]);
	uint8_t ext_min_burst; // minimum external-interface read transaction length
	uint8_t ext_burst;     // preferred full burst length
} sensor_mag_t;

#endif
