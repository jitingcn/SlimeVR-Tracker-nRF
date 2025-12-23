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
#include "util.h"

#include <math.h>

#include "sensors_enum.h"
#include "magneto/magneto1_4.h"
#include "imu/BMI270.h"

#include "calibration.h"

static uint8_t imu_id;
static uint8_t sensor_data[128]; // any use sensor data

static float accelBias[3] = {0}, gyroBias[3] = {0}, magBias[3] = {0}; // offset biases

static float accBAinv[4][3];
static float magBAinv[4][3];

static uint8_t magneto_progress;
static uint8_t last_magneto_progress;
static int64_t magneto_progress_time;

static double ata[100]; // init calibration
static double norm_sum;
static double sample_count;

// #define DEBUG true

#if DEBUG
LOG_MODULE_REGISTER(calibration, LOG_LEVEL_DBG);
#else
LOG_MODULE_REGISTER(calibration, LOG_LEVEL_INF);
#endif

static void sensor_sample_accel(const float a[3]);
static int sensor_wait_accel(float a[3], k_timeout_t timeout);

static void sensor_sample_gyro(const float g[3]);
static int sensor_wait_gyro(float g[3], k_timeout_t timeout);

static void sensor_sample_mag(const float m[3]);
static int sensor_wait_mag(float m[3], k_timeout_t timeout);

static void sensor_calibrate_imu(void);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static void sensor_calibrate_6_side(void);
#endif
static int sensor_calibrate_mag(void);

#if CONFIG_SENSOR_USE_TCAL_MANUAL_POLYNOMIAL

#define TEMP_TO_IDX(temp) (int)((((float)temp) - CONFIG_SENSOR_POLY_TEMP_MIN) * CONFIG_SENSOR_POLY_STEPS_PER_DEGREE)
#define IDX_TO_TEMP(idx) (float)(((float)(idx) / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE) + CONFIG_SENSOR_POLY_TEMP_MIN)
#define TEMP_MAX_DEVIATION 0.2f

// Temperature stability threshold for calibration (max allowed temp change during sampling)
#define TCAL_TEMP_STABILITY_THRESHOLD 0.5f  // °C

// Auto-calibration control
static bool tcal_auto_calibration_enabled = false;

static float last_gyro_tcal_offset[3] = {0.0f, 0.0f, 0.0f};
static inline bool is_invalid_float(float val)
{
	return isnan(val);
}

static int solve_linear_system(double *A, double *B, int n, double *x);
static int polyfit(int degree, float coeffs_out[3][CONFIG_SENSOR_POLY_DEGREE + 1]);
static void update_poly_tcal(void);                 // Function to calculate the curve
void sensor_tcal_clear_poly(void);                  // Public function for 'tcal clear'
void sensor_tcal_remove_point(int index_to_remove); // Public function for 'tcal remove'
static void recalculate_tcal_correction_offset(void);

#endif

// helpers
static bool wait_for_motion(bool motion, int samples);
static int check_sides(const float *);
static void magneto_reset(void);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static int isAccRest(float *, float *, float, int *, int);
#endif

// calibration logic
static int sensor_offsetBias(float *dest1, float *dest2, float *avg_temp, float *temp_range);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static int sensor_6_sideBias(float a_inv[][3]);
#endif
static void sensor_sample_mag_magneto_sample(const float a[3], const float m[3]);

static int sensor_calibration_request(int id);

static void calibration_thread(void);
K_THREAD_DEFINE(calibration_thread_id, 1024, calibration_thread, NULL, NULL, NULL, 6, 0, 0);

void sensor_calibration_process_accel(float a[3])
{
	sensor_sample_accel(a);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	apply_BAinv(a, accBAinv);
#else
	for (int i = 0; i < 3; i++) {
		a[i] -= accelBias[i];
	}
#endif
}

void sensor_calibration_process_gyro(float g[3])
{
	sensor_sample_gyro(g);
#if CONFIG_SENSOR_USE_TCAL_MANUAL_POLYNOMIAL
	float calculated_offset[3] = {0.0f, 0.0f, 0.0f}; // Local variable to hold the calculated offset for this frame
	float temp = sensor_get_current_imu_temperature();
	if (!isnan(temp) && retained->tempCalState.valid) {
		// Calculate the offset using polynomial coefficients
		for (int axis = 0; axis < 3; axis++) {
			// Start with the highest-order coefficient for the stored degree
			float offset = retained->tempCalCoeffs[axis][retained->tempCalState.degree];
			// Loop down to the constant term
			for (int i = retained->tempCalState.degree - 1; i >= 0; i--) {
				offset = offset * temp + retained->tempCalCoeffs[axis][i];
			}
			calculated_offset[axis] = offset + retained->tempCalCorrectionOffset[axis];
			g[axis] -= calculated_offset[axis];
		}
	} else {
		// Fallback to the default ZRO bias if no valid T-Cal is available
		for (int i = 0; i < 3; i++) {
			calculated_offset[i] = gyroBias[i];
			g[i] -= calculated_offset[i];
		}
	}
	memcpy(last_gyro_tcal_offset, calculated_offset, sizeof(last_gyro_tcal_offset));
#else
	for (int i = 0; i < 3; i++) {
		g[i] -= gyroBias[i];
	}
#endif
}

void sensor_calibration_process_mag(float m[3])
{
	//	for (int i = 0; i < 3; i++)
	//		m[i] -= magBias[i];
	sensor_sample_mag(m);
	apply_BAinv(m, magBAinv);
}

void sensor_calibration_update_sensor_ids(int imu)
{
	imu_id = imu;
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
	memcpy(magBAinv, retained->magBAinv, sizeof(magBAinv));
	memcpy(accBAinv, retained->accBAinv, sizeof(accBAinv));
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
	float zero[3] = {0};
	float diagonal[3];
	for (int i = 0; i < 3; i++) {
		diagonal[i] = m_inv[i + 1][i];
	}
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	if (!v_epsilon(m_inv[0], zero, 1)
		|| !v_epsilon(
			diagonal,
			average,
			MAX(magnitude * 0.2f, 0.1f)
		)) // check offset is <1 unit and diagonals are within 20%
	{
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
	}

	sensor_fusion_invalidate();
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
	if (m_inv == NULL) {
		m_inv = magBAinv;
	}
	memset(m_inv, 0, sizeof(magBAinv)); // zeroed matrix will disable magnetometer in fusion
	if (write) {
		LOG_INF("Clearing stored calibration data");
		sys_write(MAIN_MAG_BIAS_ID, &retained->magBAinv, m_inv, sizeof(magBAinv));
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

void sensor_request_calibration_mag(void)
{
	magneto_progress |= 1 << 7;
	if (magneto_progress == 0b10111111) {
		magneto_progress |= 1 << 6;
	}
}

static float aBuf[3] = {0};
uint64_t accel_sample = 0;
uint64_t accel_wait_sample = 0;

static void sensor_sample_accel(const float a[3])
{
	memcpy(aBuf, a, sizeof(aBuf));
	accel_sample++;
	if (accel_wait_sample) {
		k_usleep(1); // yield to waiting thread
	}
}

static int sensor_wait_accel(float a[3], k_timeout_t timeout)
{
	int64_t sample_end_time = MAX(k_uptime_ticks() + timeout.ticks, timeout.ticks);
	accel_wait_sample = accel_sample;
	while (accel_sample <= accel_wait_sample && k_uptime_ticks() < sample_end_time) {
		k_usleep(1);
	}
	accel_wait_sample = 0;
	if (k_uptime_ticks() >= sample_end_time) {
		LOG_ERR("Accelerometer wait timed out");
		return -1;
	}
	memcpy(a, aBuf, sizeof(aBuf));
	return 0;
}

static float gBuf[3] = {0};
uint64_t gyro_sample = 0;
uint64_t gyro_wait_sample = 0;

static void sensor_sample_gyro(const float g[3])
{
	memcpy(gBuf, g, sizeof(gBuf));
	gyro_sample++;
	if (gyro_wait_sample) {
		k_usleep(1); // yield to waiting thread
	}
}

static int sensor_wait_gyro(float g[3], k_timeout_t timeout)
{
	int64_t sample_end_time = MAX(k_uptime_ticks() + timeout.ticks, timeout.ticks);
	gyro_wait_sample = gyro_sample;
	while (gyro_sample <= gyro_wait_sample && k_uptime_ticks() < sample_end_time) {
		k_usleep(1);
	}
	gyro_wait_sample = 0;
	if (k_uptime_ticks() >= sample_end_time) {
		LOG_ERR("Gyroscope wait timed out");
		return -1;
	}
	memcpy(g, gBuf, sizeof(gBuf));
	return 0;
}

static float mBuf[3] = {0};
uint64_t mag_sample = 0;
uint64_t mag_wait_sample = 0;

static void sensor_sample_mag(const float m[3])
{
	memcpy(mBuf, m, sizeof(mBuf));
	mag_sample++;
	if (mag_wait_sample) {
		k_usleep(1); // yield to waiting thread
	}
}

static int sensor_wait_mag(float m[3], k_timeout_t timeout)
{
	int64_t sample_end_time = MAX(k_uptime_ticks() + timeout.ticks, timeout.ticks);
	mag_wait_sample = mag_sample;
	while (mag_sample <= mag_wait_sample && k_uptime_ticks() < sample_end_time) {
		k_usleep(1);
	}
	mag_wait_sample = 0;
	if (k_uptime_ticks() >= sample_end_time) {
		LOG_ERR("Magnetometer wait timed out");
		return -1;
	}
	memcpy(m, mBuf, sizeof(mBuf));
	return 0;
}

static void sensor_calibrate_imu()
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

#if CONFIG_SENSOR_USE_TCAL_MANUAL_POLYNOMIAL
	// Variables to store average temperature and temperature range from calibration
	float avg_temp = NAN;
	float temp_range = NAN;
#endif

	if (imu_id == IMU_BMI270) // bmi270 specific
	{
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
		sys_write(MAIN_SENSOR_DATA_ID, &retained->sensor_data, sensor_data, sizeof(sensor_data));
		sensor_fusion_invalidate(); // only invalidate fusion if calibration was successful
		k_msleep(500);              // Delay before beginning acquisition
	}

	LOG_INF("Reading data");
	sensor_calibration_clear(a_bias, g_bias, false);
#if CONFIG_SENSOR_USE_TCAL_MANUAL_POLYNOMIAL
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
#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		LOG_INF("Accelerometer bias: %.5f %.5f %.5f", (double)a_bias[0], (double)a_bias[1], (double)a_bias[2]);
#endif
		LOG_INF("Gyroscope bias: %.5f %.5f %.5f", (double)g_bias[0], (double)g_bias[1], (double)g_bias[2]);
	}
	if (sensor_calibration_validate(a_bias, g_bias, false)) {
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		LOG_INF("Restoring previous calibration");
#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		LOG_INF("Accelerometer bias: %.5f %.5f %.5f", (double)accelBias[0], (double)accelBias[1], (double)accelBias[2]);
#endif
		LOG_INF("Gyroscope bias: %.5f %.5f %.5f", (double)gyroBias[0], (double)gyroBias[1], (double)gyroBias[2]);
		sensor_calibration_validate(NULL, NULL, true); // additionally verify old calibration
		return;
	} else {
		LOG_INF("Applying calibration");
		memcpy(accelBias, a_bias, sizeof(accelBias));
		memcpy(gyroBias, g_bias, sizeof(gyroBias));
		sensor_fusion_invalidate(); // only invalidate fusion if calibration was successful
	}
	sys_write(MAIN_ACCEL_BIAS_ID, &retained->accelBias, accelBias, sizeof(accelBias));
	sys_write(MAIN_GYRO_BIAS_ID, &retained->gyroBias, gyroBias, sizeof(gyroBias));

#if CONFIG_SENSOR_USE_TCAL_MANUAL_POLYNOMIAL
	// 2. Add the new data as a T-Cal point using average temperature
	sys_write(MAIN_GYRO_TEMP_ID, &retained->gyroTemp, &avg_temp, sizeof(avg_temp));
	if (!isnan(avg_temp)) {
		LOG_INF("T-Cal: Saving calibration point at average temp %.2fC (range: %.2fC)",
			(double)avg_temp, (double)temp_range);
		int idx = TEMP_TO_IDX(avg_temp);
		if (idx >= 0 && idx < TCAL_BUFFER_SIZE) {

			// Remove nearby points to avoid numerical instability
			float min_separation = 0.3f;

			// Check lower indices (sorted by temp)
			for (int i = idx - 1; i >= 0; i--) {
				if (retained->tempCalPoints[i].temp != 0.0f) {
					if (fabsf(retained->tempCalPoints[i].temp - avg_temp) < min_separation) {
						retained->tempCalPoints[i].temp = 0.0f;
						memset(retained->tempCalPoints[i].bias, 0, sizeof(retained->tempCalPoints[i].bias));
						if (retained->tempCalState.count > 0) {
							retained->tempCalState.count--;
						}
						LOG_INF("T-Cal: Removed conflict point at index %d", i);
					} else {
						break; // Further points are even further away
					}
				}
			}
			// Check upper indices (sorted by temp)
			for (int i = idx + 1; i < TCAL_BUFFER_SIZE; i++) {
				if (retained->tempCalPoints[i].temp != 0.0f) {
					if (fabsf(retained->tempCalPoints[i].temp - avg_temp) < min_separation) {
						retained->tempCalPoints[i].temp = 0.0f;
						memset(retained->tempCalPoints[i].bias, 0, sizeof(retained->tempCalPoints[i].bias));
						if (retained->tempCalState.count > 0) {
							retained->tempCalState.count--;
						}
						LOG_INF("T-Cal: Removed conflict point at index %d", i);
					} else {
						break; // Further points are even further away
					}
				}
			}

			if (retained->tempCalPoints[idx].temp == 0.0f) {
				retained->tempCalState.count++; // Use struct member
			}
			retained->tempCalPoints[idx].temp = avg_temp;
			memcpy(retained->tempCalPoints[idx].bias, g_bias, sizeof(g_bias));
			retained->tempCalState.valid = false; // Invalidate old curve
			update_poly_tcal();

		} else {
			LOG_WRN(
				"T-Cal: Temperature %.2fC is outside the configured calibration range. Point not saved.",
				(double)avg_temp
			);
		}
	}
#endif

	LOG_INF("Finished calibration");
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static void sensor_calibrate_6_side(void)
{
	float a_inv[4][3];
	LOG_INF("Calibrating main accelerometer 6-side offset");
	LOG_INF("Rest the device on a stable surface");

	sensor_calibration_clear_6_side(a_inv, false);
	int err = sensor_6_sideBias(a_inv);
	if (err) {
		magneto_reset();
		if (err == -1) {
			LOG_INF("Motion detected");
		}
		a_inv[0][0] = NAN; // invalidate calibration
	} else {
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

static int sensor_calibrate_mag(void)
{
	float zero[3] = {0};
	if (v_diff_mag(magBAinv[0], zero) != 0) {
		return -1; // magnetometer calibration already exists
	}

	float m[3];
	if (sensor_wait_mag(m, K_MSEC(1000))) {
		return -1; // Timeout
	}
	sensor_sample_mag_magneto_sample(aBuf, m); // 400us
	if (magneto_progress != 0b11111111) {
		return 0;
	}

	float m_inv[4][3];
	LOG_INF("Calibrating magnetometer hard/soft iron offset");

	// max allocated 1072 bytes
#if DEBUG
	printk("ata:\n");
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			printk("%7.2f, ", (double)ata[i * 10 + j]);
		}
		printk("\n");
		k_msleep(3);
	}
	printk("norm_sum: %.2f, sample_count: %.0f\n", norm_sum, sample_count);
#endif
	wait_for_threads();
	magneto_current_calibration(m_inv, ata, norm_sum, sample_count); // 25ms
	magneto_reset();

	LOG_INF("Magnetometer matrix:");
	for (int i = 0; i < 3; i++) {
		LOG_INF(
			"%.5f %.5f %.5f %.5f",
			(double)m_inv[0][i],
			(double)m_inv[1][i],
			(double)m_inv[2][i],
			(double)m_inv[3][i]
		);
	}
	if (sensor_calibration_validate_mag(m_inv, false)) {
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		LOG_INF("Restoring previous calibration");
		LOG_INF("Magnetometer matrix:");
		for (int i = 0; i < 3; i++) {
			LOG_INF(
				"%.5f %.5f %.5f %.5f",
				(double)magBAinv[0][i],
				(double)magBAinv[1][i],
				(double)magBAinv[2][i],
				(double)magBAinv[3][i]
			);
		}
		sensor_calibration_validate_mag(NULL, true); // additionally verify old calibration
		return -1;
	} else {
		LOG_INF("Applying calibration");
		memcpy(magBAinv, m_inv, sizeof(magBAinv));
		// fusion invalidation not necessary
	}
	sys_write(MAIN_MAG_BIAS_ID, &retained->magBAinv, magBAinv, sizeof(magBAinv));

	LOG_INF("Finished calibration");
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
	return 0;
}

// TODO: isAccRest
static bool wait_for_motion(bool motion, int samples)
{
	uint8_t counts = 0;
	float a[3], last_a[3];
	if (sensor_wait_accel(last_a, K_MSEC(1000))) {
		return false;
	}
	LOG_INF("Accelerometer: %.5f %.5f %.5f", (double)last_a[0], (double)last_a[1], (double)last_a[2]);
	for (int i = 0; i < samples + counts; i++) {
		k_msleep(500);
		if (sensor_wait_accel(a, K_MSEC(1000))) {
			return false;
		}
		LOG_INF("Accelerometer: %.5f %.5f %.5f", (double)a[0], (double)a[1], (double)a[2]);
		if (v_epsilon(a, last_a, 0.1) != motion) {
			LOG_INF("No motion detected");
			counts++;
			if (counts == 2) {
				return true;
			}
		} else {
			counts = 0;
		}
		memcpy(last_a, a, sizeof(a));
	}
	LOG_INF("Motion detected");
	return false;
}

static int check_sides(const float *a)
{
	return (-1.2f < a[0] && a[0] < -0.8f ? 1 << 0 : 0) | (1.2f > a[0] && a[0] > 0.8f ? 1 << 1 : 0)
		 | // dumb check if all accel axes were reached for calibration, assume the user is intentionally doing this
		   (-1.2f < a[1] && a[1] < -0.8f ? 1 << 2 : 0) | (1.2f > a[1] && a[1] > 0.8f ? 1 << 3 : 0)
		 | (-1.2f < a[2] && a[2] < -0.8f ? 1 << 4 : 0) | (1.2f > a[2] && a[2] > 0.8f ? 1 << 5 : 0);
}

static void magneto_reset(void)
{
	magneto_progress = 0; // reusing ata, so guarantee cleared mag progress
	last_magneto_progress = 0;
	magneto_progress_time = 0;
	memset(ata, 0, sizeof(ata));
	norm_sum = 0;
	sample_count = 0;
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static int isAccRest(float *acc, float *pre_acc, float threshold, int *t, int restdelta)
{
	float delta_x = acc[0] - pre_acc[0];
	float delta_y = acc[1] - pre_acc[1];
	float delta_z = acc[2] - pre_acc[2];

	float norm_diff = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

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

// TODO: setup 6 sided calibration (bias and scale, and maybe gyro ZRO?), setup temp calibration (particulary for gyro
// ZRO)
int sensor_offsetBias(float *dest1, float *dest2, float *avg_temp, float *temp_range)
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

#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	double accel_sum[3] = {0};
#endif
	double gyro_sum[3] = {0};

#if CONFIG_SENSOR_USE_TCAL_MANUAL_POLYNOMIAL
	// Temperature sampling for T-Cal: record start, middle, end temps
	float temp_sum = 0.0;
	int temp_sample_count = 0;
	float temp_start = NAN, temp_mid = NAN, temp_end = NAN;
	float current_temp;

	// Record start temperature
	current_temp = sensor_get_current_imu_temperature();
	if (!isnan(current_temp) && current_temp > -10.0f && current_temp < 60.0f) {
		temp_start = current_temp;
		temp_sum += current_temp;
		temp_sample_count++;
	}
#endif

	int64_t sampling_start_time = k_uptime_get();
	int i = 0;
	while (k_uptime_get() < sampling_start_time + 3000) {
		// Accumulate Accelerometer
		if (sensor_wait_accel(rawData, K_MSEC(1000))) {
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
			if (max_a[j] - min_a[j] > 0.1f) { // Threshold 0.1G
				LOG_INF("Accel motion detected: axis %d range %.4f", j, (double)(max_a[j] - min_a[j]));
				return -1;
			}
		}

#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		accel_sum[0] += (double)rawData[0];
		accel_sum[1] += (double)rawData[1];
		accel_sum[2] += (double)rawData[2];
#endif
		// Accumulate Gyroscope
		if (sensor_wait_gyro(rawData, K_MSEC(1000))) {
			return -2; // Timeout
		}

		// Check Gyro Motion (Optimization)
		for (int j = 0; j < 3; j++) {
			if (rawData[j] < min_g[j]) {
				min_g[j] = rawData[j];
			}
			if (rawData[j] > max_g[j]) {
				max_g[j] = rawData[j];
			}
			if (max_g[j] - min_g[j] > 2.0f) { // Threshold 2.0 dps
				LOG_INF("Gyro motion detected: axis %d range %.4f", j, (double)(max_g[j] - min_g[j]));
				return -1;
			}
		}

		gyro_sum[0] += (double)rawData[0];
		gyro_sum[1] += (double)rawData[1];
		gyro_sum[2] += (double)rawData[2];
		i++;

#if CONFIG_SENSOR_USE_TCAL_MANUAL_POLYNOMIAL
		// Sample temperature throughout calibration
		current_temp = sensor_get_current_imu_temperature();
		if (!isnan(current_temp) && current_temp > -10.0f && current_temp < 60.0f) {
			temp_sum += current_temp;
			temp_sample_count++;

			// Record middle temperature (around 1.5s)
			if (isnan(temp_mid) && k_uptime_get() >= sampling_start_time + 1500) {
				temp_mid = current_temp;
			}
		}
#endif
	}
	LOG_INF("Samples: %d", i);

	if (i == 0) {
		return -2;
	}

#if CONFIG_SENSOR_USE_TCAL_MANUAL_POLYNOMIAL
	// Record end temperature
	current_temp = sensor_get_current_imu_temperature();
	if (!isnan(current_temp) && current_temp > -10.0f && current_temp < 60.0f) {
		temp_end = current_temp;
		temp_sum += current_temp;
		temp_sample_count++;
	}

	// Calculate average temperature
	if (temp_sample_count > 0 && avg_temp != NULL) {
		*avg_temp = (float)(temp_sum / temp_sample_count);
		LOG_INF("T-Cal: Average temperature during calibration: %.2fC (%d samples)",
			(double)*avg_temp, temp_sample_count);
	} else if (avg_temp != NULL) {
		*avg_temp = NAN;
	}

	// Check temperature stability
	if (!isnan(temp_start) && !isnan(temp_end) && temp_range != NULL) {
		float min_temp = temp_start;
		float max_temp = temp_start;

		if (!isnan(temp_mid)) {
			if (temp_mid < min_temp) min_temp = temp_mid;
			if (temp_mid > max_temp) max_temp = temp_mid;
		}
		if (temp_end < min_temp) min_temp = temp_end;
		if (temp_end > max_temp) max_temp = temp_end;

		*temp_range = max_temp - min_temp;

		LOG_INF("T-Cal: Temperature range during calibration: %.2fC (%.2fC to %.2fC)",
			(double)*temp_range, (double)min_temp, (double)max_temp);

		// Check if temperature changed too much
		if (*temp_range > TCAL_TEMP_STABILITY_THRESHOLD) {
			LOG_WRN("T-Cal: Temperature changed too much (%.2fC) during calibration. "
				"Rejecting calibration. Please wait for temperature to stabilize.",
				(double)*temp_range);
			return -3; // Temperature instability error
		}
	} else if (temp_range != NULL) {
		*temp_range = NAN;
	}
#endif

#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	dest1[0] = (float)(accel_sum[0] / i);
	dest1[1] = (float)(accel_sum[1] / i);
	dest1[2] = (float)(accel_sum[2] / i);
	if (dest1[0] > 0.9f) {
		dest1[0] -= 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
	} else if (dest1[0] < -0.9f) {
		dest1[0] += 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
	} else if (dest1[1] > 0.9f) {
		dest1[1] -= 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
	} else if (dest1[1] < -0.9f) {
		dest1[1] += 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
	} else if (dest1[2] > 0.9f) {
		dest1[2] -= 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
	} else if (dest1[2] < -0.9f) {
		dest1[2] += 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
	} else {
		return -1;
	}
#endif
	dest2[0] = (float)(gyro_sum[0] / i);
	dest2[1] = (float)(gyro_sum[1] / i);
	dest2[2] = (float)(gyro_sum[2] / i);
	return 0;
}

// TODO: can be used to get a better gyro bias
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
int sensor_6_sideBias(float a_inv[][3])
{
	// Acc 6 side calibrate
	float rawData[3];
	float pre_acc[3] = {0};

	const float THRESHOLD_ACC = 0.05;
	int resttime = 0;

	magneto_reset();
	int c = 0;
	printk("Starting accelerometer calibration.\n");
	while (1) {
		set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_SENSOR);
		printk("Waiting for a resting state...\n");
		while (1) {
			if (sensor_wait_accel(rawData, K_MSEC(1000))) {
				return -2; // Timeout, magneto state not handled here
			}
			int rest = isAccRest(rawData, pre_acc, THRESHOLD_ACC, &resttime, 100);
			pre_acc[0] = rawData[0];
			pre_acc[1] = rawData[1];
			pre_acc[2] = rawData[2];

			// force not resting until a new side is detected and stable
			uint8_t new_magneto_progress = magneto_progress;
			new_magneto_progress |= check_sides(rawData);
			if (new_magneto_progress > magneto_progress && new_magneto_progress == last_magneto_progress) {
				if (k_uptime_get() < magneto_progress_time) {
					rest = 0;
				}
			} else {
				magneto_progress_time = k_uptime_get() + 1000;
				last_magneto_progress = new_magneto_progress;
				rest = 0;
			}

			if (rest == 1) {
				magneto_progress = new_magneto_progress;
				printk("Rest detected, starting recording. Please do not move. %d\n", c);
				set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_SENSOR);
				k_msleep(100);

				int64_t sampling_start_time = k_uptime_get();
				uint8_t i = 0;
				// Increased sampling time from 1000ms to 5000ms for better calibration quality
				while (k_uptime_get() < sampling_start_time + 5000) {
					if (sensor_wait_accel(rawData, K_MSEC(1000))) {
						return -2; // Timeout, magneto state not handled here
					}
					if (!v_epsilon(rawData, pre_acc, 0.1)) {
						return -1; // Motion detected
					}
					magneto_sample(rawData[0], rawData[1], rawData[2], ata, &norm_sum, &sample_count);
					// Progress indicator every 500ms for 5 second sampling
					if (k_uptime_get() >= sampling_start_time + i * 500) {
						printk("#");
						i++;
					}
				}
				set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
				printk("Recorded values!\n");
				printk("%d side done\n", c);
				c++;
				break;
			}
			k_msleep(100);
		}
		if (c >= 6) {
			break;
		}
		printk("Waiting for the next side... %d \n", c);
		while (1) {
			k_msleep(100);
			if (sensor_wait_accel(rawData, K_MSEC(1000))) {
				return -2; // Timeout, magneto state not handled here
			}
			int rest = isAccRest(rawData, pre_acc, THRESHOLD_ACC, &resttime, 100);
			pre_acc[0] = rawData[0];
			pre_acc[1] = rawData[1];
			pre_acc[2] = rawData[2];

			if (rest == 0) {
				resttime = 0;
				break;
			}
		}
	}

	printk("Calculating the data....\n");
#if DEBUG
	printk("ata:\n");
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			printk("%7.2f, ", (double)ata[i * 10 + j]);
		}
		printk("\n");
		k_msleep(3);
	}
	printk("norm_sum: %.2f, sample_count: %.0f\n", norm_sum, sample_count);
#endif
	wait_for_threads(); // TODO: let the data cook or something idk why this has to be here to work
	magneto_current_calibration(a_inv, ata, norm_sum, sample_count);
	magneto_reset();

	printk("Calibration is complete.\n");
	return 0;
}
#endif

// TODO: terrible name
static void sensor_sample_mag_magneto_sample(const float a[3], const float m[3])
{
	magneto_sample(m[0], m[1], m[2], ata, &norm_sum, &sample_count); // 400us
	uint8_t new_magneto_progress = magneto_progress;
	new_magneto_progress |= check_sides(a);
	if (new_magneto_progress > magneto_progress && new_magneto_progress == last_magneto_progress) {
		if (k_uptime_get() > magneto_progress_time) {
			magneto_progress = new_magneto_progress;
			LOG_INF(
				"Magnetometer calibration progress: %s %s %s %s %s %s",
				(new_magneto_progress & 0x01) ? "-X" : "--",
				(new_magneto_progress & 0x02) ? "+X" : "--",
				(new_magneto_progress & 0x04) ? "-Y" : "--",
				(new_magneto_progress & 0x08) ? "+Y" : "--",
				(new_magneto_progress & 0x10) ? "-Z" : "--",
				(new_magneto_progress & 0x20) ? "+Z" : "--"
			);
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
		}
	} else {
		magneto_progress_time = k_uptime_get() + 1000;
		last_magneto_progress = new_magneto_progress;
	}
	if (magneto_progress == 0b10111111) {
		set_led(SYS_LED_PATTERN_FLASH, SYS_LED_PRIORITY_SENSOR); // Magnetometer calibration is ready to apply
	}
}

static int sensor_calibration_request(int id)
{
	static int requested = 0;
	switch (id) {
	case -1:
		requested = 0;
		return 0;
	case 0:
		return requested;
	default:
		if (requested != 0) {
			LOG_ERR("Sensor calibration is already running");
			return -1;
		}
		requested = id;
		return 0;
	}
}

static void calibration_thread(void)
{
	sensor_calibration_read();
	// TODO: be able to block the sensor while doing certain operations
	// TODO: reset fusion on calibration finished
	// TODO: start and run thread from request?
	// TODO: replace wait_for_motion with isAccRest

	// Verify calibrations
	sensor_calibration_validate(NULL, NULL, true);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	sensor_calibration_validate_6_side(NULL, true);
#endif
	sensor_calibration_validate_mag(NULL, true);

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
		default:
			if (magneto_progress & 0b10000000) {
				requested = sensor_calibrate_mag();
			}
			break;
		}
		if (requested < 0) {
			k_msleep(5);
		} else {
			k_msleep(100);
		}
	}
}

#if CONFIG_SENSOR_USE_TCAL_MANUAL_POLYNOMIAL

void sensor_tcal_status_poly(void)
{
	printk("Polynomial Temperature Calibration Status:\n");
	printk("  - Curve calculated: %s\n", retained->tempCalState.valid ? "Yes" : "No");
	printk("  - Points collected: %u / %d\n", retained->tempCalState.count, TCAL_BUFFER_SIZE);

	float min_temp = 1000.0f, max_temp = -1000.0f;
	if (retained->tempCalState.count > 0) {
		for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
			if (retained->tempCalPoints[i].temp != 0.0f) { // Check if slot has data
				if (retained->tempCalPoints[i].temp < min_temp) {
					min_temp = retained->tempCalPoints[i].temp;
				}
				if (retained->tempCalPoints[i].temp > max_temp) {
					max_temp = retained->tempCalPoints[i].temp;
				}
			}
		}
		printk("  - Calibrated temp range: %.2fC to %.2fC\n", (double)min_temp, (double)max_temp);
	}
}

// Solves a system of linear equations A*x = b using Gaussian elimination with partial pivoting.

// A: Pointer to the start of an n x n matrix (row-major order). Modified in place.
// b: Pointer to the start of a vector of size n. Modified in place.
// n: The dimension of the system.
// x: Pointer to a vector of size n where the solution will be stored.

static int solve_linear_system(double *A, double *b, int n, double *x)
{
	for (int i = 0; i < n; i++) {
		// --- Partial Pivoting ---
		// Find the row with the largest value in the current column i to use as the pivot.
		int max_row = i;
		for (int k = i + 1; k < n; k++) {
			if (fabs(A[k * n + i]) > fabs(A[max_row * n + i])) {
				max_row = k;
			}
		}

		// Swap the entire max_row with the current row i in both matrix A and vector b.
		for (int k = i; k < n; k++) {
			double temp = A[i * n + k];
			A[i * n + k] = A[max_row * n + k];
			A[max_row * n + k] = temp;
		}
		double temp = b[i];
		b[i] = b[max_row];
		b[max_row] = temp;

		// Check if the matrix is singular. A pivot element close to zero means no unique solution exists.
		if (fabs(A[i * n + i]) < 1e-12) {
			LOG_ERR("Matrix is singular. Cannot solve. Pivot at [%d,%d] is near zero.", i, i);
			return -1;
		}

		// --- Forward Elimination ---
		// For every row below the pivot row...
		for (int k = i + 1; k < n; k++) {
			// Calculate the factor to multiply the pivot row by.
			double factor = A[k * n + i] / A[i * n + i];

			// Subtract this multiple of the pivot row from the current row.
			// This creates a zero in the current column for this row.
			for (int j = i; j < n; j++) {
				A[k * n + j] -= factor * A[i * n + j];
			}
			// Do the same for the result vector b.
			b[k] -= factor * b[i];
		}
	}

	// --- Back Substitution ---
	// At this point, A is an upper-triangular matrix. We can solve for x from bottom to top.
	for (int i = n - 1; i >= 0; i--) {
		// Start with the known result for this row.
		x[i] = b[i];

		// Subtract the effect of the variables we've already solved for.
		for (int j = i + 1; j < n; j++) {
			x[i] -= A[i * n + j] * x[j];
		}

		// Divide by the diagonal element to get the final value for x[i].
		x[i] = x[i] / A[i * n + i];
	}

	return 0; // Success
}

// Performs a polynomial least-squares fit.
static int polyfit(int degree, float coeffs_out[3][CONFIG_SENSOR_POLY_DEGREE + 1])
{
	if (retained->tempCalState.count < degree + 1) {
		LOG_WRN("T-Cal: Not enough points (%u) to fit a degree %d polynomial.", retained->tempCalState.count, degree);
		return -1;
	}

	int n_coeffs = degree + 1;

	// The Normal Equation for least squares is (X^T * X) * a = (X^T * y)
	// Let A = (X^T * X) and b = (X^T * y). We solve A*a = b for the coefficients 'a'.

	// Matrix A is a square matrix of size n_coeffs x n_coeffs.
	// A[i][j] = sum of (t ^ (i+j)) over all data points.
	double A[n_coeffs * n_coeffs];

	// Vector b contains the results for each axis (X, Y, Z).
	// b[i] = sum of (bias * (t ^ i)) over all data points.
	double b_vectors[3][n_coeffs];

	memset(A, 0, sizeof(A));
	memset(b_vectors, 0, sizeof(b_vectors));

	LOG_DBG("Polyfit: Using %u points to calculate degree %d curve.", retained->tempCalState.count, degree);

	// 1. Build the A matrix and b vectors from the scattered data points.
	for (int p_idx = 0; p_idx < TCAL_BUFFER_SIZE; ++p_idx) {
		// Skip empty slots in the buffer
		if (retained->tempCalPoints[p_idx].temp == 0.0f) {
			continue;
		}

		double temp = retained->tempCalPoints[p_idx].temp;

		// Pre-calculate powers of the current temperature 't' up to t^(2*degree).
		double t_powers[2 * degree + 1];
		t_powers[0] = 1.0; // t^0
		for (int j = 1; j <= 2 * degree; j++) {
			t_powers[j] = t_powers[j - 1] * temp;
		}

		// Sum the powers into the matrix A
		for (int i = 0; i < n_coeffs; i++) {
			for (int j = 0; j < n_coeffs; j++) {
				A[i * n_coeffs + j] += t_powers[i + j];
			}
		}

		// Sum the bias * powers into the vectors b
		for (int i = 0; i < n_coeffs; i++) {
			b_vectors[0][i] += (double)retained->tempCalPoints[p_idx].bias[0] * t_powers[i];
			b_vectors[1][i] += (double)retained->tempCalPoints[p_idx].bias[1] * t_powers[i];
			b_vectors[2][i] += (double)retained->tempCalPoints[p_idx].bias[2] * t_powers[i];
		}
	}

	// 2. Solve the system A*x=b for each axis.
	for (int axis = 0; axis < 3; axis++) {
		// Create copies because the solver modifies the inputs in place.
		double A_copy[n_coeffs * n_coeffs];
		memcpy(A_copy, A, sizeof(A));
		double b_copy[n_coeffs];
		memcpy(b_copy, b_vectors[axis], sizeof(b_copy));

		double solution[n_coeffs]; // The calculated coefficients will be stored here.

		if (solve_linear_system(A_copy, b_copy, n_coeffs, solution) != 0) {
			LOG_ERR("T-Cal: Failed to solve for polynomial coefficients. Matrix may be singular.");
			return -1;
		}

		// Copy the double-precision solution to the float output array.
		for (int i = 0; i < n_coeffs; i++) {
			coeffs_out[axis][i] = (float)solution[i];
		}
	}

	LOG_INF("Polynomial coefficients calculated successfully for degree %d.", degree);
	return 0;
}

// Function to handle recalculating the curve after a point is added/removed
static void update_poly_tcal(void)
{
	memset(retained->tempCalCoeffs, 0, sizeof(retained->tempCalCoeffs));
	retained->tempCalState.degree = 0;
	retained->tempCalState.valid = false;

	retained_update();

	if (retained->tempCalState.count < 2) {
		LOG_INF("T-Cal: Not enough points (%u)...", retained->tempCalState.count);
	} else {
		int degree = retained->tempCalState.count - 1;
		if (degree > CONFIG_SENSOR_POLY_DEGREE) {
			degree = CONFIG_SENSOR_POLY_DEGREE;
		}

		LOG_INF("T-Cal: Recalculating curve with %u points", retained->tempCalState.count);

		if (polyfit(degree, retained->tempCalCoeffs) == 0) {
			retained->tempCalState.valid = true;
			retained->tempCalState.degree = degree;
			printk("T-Cal: New curve calculated successfully.\n");
		} else {
			printk("T-Cal: Failed to calculate new curve.\n");
		}
	}

	retained_update();

	// Save updated state to NVS
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

	recalculate_tcal_correction_offset();

	// Invalidate sensor fusion to apply new calibration
	sensor_fusion_invalidate();
}

// Public function for 'tcal clear' and 'reset tcal'
void sensor_tcal_clear_poly(void)
{
	if (sensor_calibration_request(0) != 0) {
		LOG_ERR("Another calibration is running. Cannot clear T-Cal data.");
		printk("Error: Another calibration is running.\n");
		return;
	}

	LOG_INF("Clearing all manual polynomial T-Cal data.");
	memset(retained->tempCalPoints, 0, sizeof(retained->tempCalPoints));
	memset(retained->tempCalCoeffs, 0, sizeof(retained->tempCalCoeffs));
	memset(&retained->tempCalState, 0, sizeof(retained->tempCalState)); // Clear the whole state struct
	memset(retained->tempCalCorrectionOffset, 0, sizeof(retained->tempCalCorrectionOffset));

	// Save cleared state to NVS directly (don't call update_poly_tcal which recalculates the curve)
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
	sys_write(
		MAIN_GYRO_TCAL_CORRECTION_ID,
		retained->tempCalCorrectionOffset,
		retained->tempCalCorrectionOffset,
		sizeof(retained->tempCalCorrectionOffset)
	);

	// Invalidate sensor fusion to apply cleared calibration
	sensor_fusion_invalidate();

	printk("All polynomial temperature calibration data has been cleared.\n");
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

		printk("Point at index %d removed. Recalculating curve...\n", index_to_remove);
		update_poly_tcal(); // Recalculate and save
	} else {
		printk("No data found at index %d. Nothing to remove.\n", index_to_remove);
	}
}

// Check if current temperature needs calibration (missing nearby calibration point)
bool sensor_tcal_is_temp_outside_range(float temp, float *min_temp, float *max_temp)
{
	if (retained->tempCalState.count < 1) {
		if (min_temp) *min_temp = NAN;
		if (max_temp) *max_temp = NAN;
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
	if (min_temp) *min_temp = closest_temp;
	if (max_temp) *max_temp = closest_distance;

	// Need calibration if closest point is farther than sampling interval
	// Add small tolerance (10%) to avoid triggering at boundary
	return (closest_distance > sampling_interval * 1.1f);
}

// Set auto-calibration enabled/disabled
void sensor_tcal_set_auto_calibration(bool enabled)
{
	tcal_auto_calibration_enabled = enabled;
	LOG_INF("T-Cal Auto-calibration %s", enabled ? "enabled" : "disabled");
}

// Get auto-calibration enabled status
bool sensor_tcal_get_auto_calibration(void)
{
	return tcal_auto_calibration_enabled;
}

// Check and request auto calibration if conditions are met
void sensor_tcal_check_auto_calibration(float current_temp)
{
	static int64_t last_check_time = 0;
	static int64_t last_auto_cal_time = 0;

	int64_t now = k_uptime_get();

	// Check if auto-calibration is enabled
	if (!tcal_auto_calibration_enabled) {
		return;
	}

	// Only check every 3 seconds to avoid excessive checking
	if ((now - last_check_time) < 3000) {
		return;
	}
	last_check_time = now;

	// Don't auto-calibrate too frequently (at least 13 seconds between auto cals)
	if ((now - last_auto_cal_time) < 13000) {
		return;
	}

	// Check if we need periodic recalibration (every 5 minutes)
	const int64_t periodic_cal_interval = 300000; // 5 minutes
	bool needs_periodic_cal = (now - last_auto_cal_time) > periodic_cal_interval;

	// Check if temperature is outside calibrated range
	float min_temp, max_temp;
	bool needs_temp_cal = sensor_tcal_is_temp_outside_range(current_temp, &min_temp, &max_temp);

	// Trigger calibration if either condition is met
	if (needs_temp_cal) {
		LOG_INF("T-Cal Auto: Temp %.2fC needs calibration (closest: %.2fC, distance: %.2fC)",
			(double)current_temp, (double)min_temp, (double)max_temp);
	} else if (needs_periodic_cal) {
		LOG_INF("T-Cal Auto: Periodic recalibration (>5min since last cal, temp: %.2fC)",
			(double)current_temp);
	} else {
		return; // No calibration needed
	}

	LOG_INF("T-Cal Auto: Requesting auto-calibration (device is resting)");
	last_auto_cal_time = now;

	// Request standard calibration (which will include tcal)
	// The calibration routine itself will check if device is still using wait_for_motion()
	sensor_request_calibration();
}

// Recalculates the T-Cal correction offset based on the current curve and anchor point.
static void recalculate_tcal_correction_offset(void)
{
	float anchor_temp;
	const float *anchor_bias;
	bool anchor_found = false;

	// Select the anchor point
	if (!anchor_found && !isnan(retained->gyroTemp)) {
		anchor_temp = retained->gyroTemp;
		anchor_bias = retained->gyroBias;
		anchor_found = true;
	}

	// Calculate the offset using the selected anchor
	if (anchor_found && retained->tempCalState.valid) {
		LOG_INF("Recalculating T-Cal correction offset using anchor temp %.2fC", (double)anchor_temp);

		// 1. Evaluate the polynomial at the anchor temperature
		float poly_bias_at_anchor[3];
		for (int axis = 0; axis < 3; axis++) {
			float offset = retained->tempCalCoeffs[axis][retained->tempCalState.degree];
			for (int i = retained->tempCalState.degree - 1; i >= 0; i--) {
				offset = offset * anchor_temp + retained->tempCalCoeffs[axis][i];
			}
			poly_bias_at_anchor[axis] = offset;
		}

		// 2. Calculate the new correction vector: Correction = RealAnchorBias - CurveBiasAtAnchor
		for (int i = 0; i < 3; i++) {
			retained->tempCalCorrectionOffset[i] = anchor_bias[i] - poly_bias_at_anchor[i];
		}

		LOG_INF(
			"New T-Cal correction offset: [%.5f, %.5f, %.5f]",
			(double)retained->tempCalCorrectionOffset[0],
			(double)retained->tempCalCorrectionOffset[1],
			(double)retained->tempCalCorrectionOffset[2]
		);

	} else {
		// If no valid anchor or curve exists, the offset must be zero.
		if (!retained->tempCalState.valid) {
			LOG_WRN("T-Cal curve not valid. Cannot calculate offset.");
		}
		if (!anchor_found) {
			LOG_WRN("No valid anchor point found. Cannot calculate offset.");
		}
		memset(retained->tempCalCorrectionOffset, 0, sizeof(retained->tempCalCorrectionOffset));
	}

	// Always save the resulting correction offset (either new or zeroed out) to NVS
	sys_write(
		MAIN_GYRO_TCAL_CORRECTION_ID,
		retained->tempCalCorrectionOffset,
		retained->tempCalCorrectionOffset,
		sizeof(retained->tempCalCorrectionOffset)
	);
}

void sensor_calibration_get_last_gyro_offset(float offset[3])
{
	memcpy(offset, last_gyro_tcal_offset, sizeof(last_gyro_tcal_offset));
}
#endif
