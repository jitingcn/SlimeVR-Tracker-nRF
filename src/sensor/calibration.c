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
#include "sensor.h"
#include "system/system.h"
#include "system/watchdog.h"
#include "util.h"

#include <math.h>
#include <stdlib.h>

#if CONFIG_CMSIS_DSP
#include <arm_math.h>
#endif

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

// =============================================================================
// Offset-bias collection constants
// =============================================================================
#ifndef BIAS_COLLECT_TEMP_RANGE_THRESHOLD
#define BIAS_COLLECT_TEMP_RANGE_THRESHOLD 1.0f // °C - stop early if temp changes this much
#endif

#ifndef BIAS_COLLECT_MAX_SAMPLE_TIME_MS
#define BIAS_COLLECT_MAX_SAMPLE_TIME_MS 6000 // 6 seconds max
#endif

#ifndef BIAS_COLLECT_MIN_SAMPLE_TIME_MS
#define BIAS_COLLECT_MIN_SAMPLE_TIME_MS 4000 // 4 seconds min
#endif

#ifndef BIAS_COLLECT_TEMP_CHECK_TIME_MS
#define BIAS_COLLECT_TEMP_CHECK_TIME_MS 4000 // 4 seconds - prioritize sampling time over temp stability
#endif

#ifndef BIAS_COLLECT_GYRO_MOTION_THRESHOLD
#define BIAS_COLLECT_GYRO_MOTION_THRESHOLD 2.0f // dps (range method)
#endif

#ifndef BIAS_COLLECT_ACCEL_MOTION_THRESHOLD
#define BIAS_COLLECT_ACCEL_MOTION_THRESHOLD 0.12f // G (range method)
#endif

#if CONFIG_SENSOR_USE_TCAL

#define TEMP_TO_IDX(temp) (int)((((float)temp) - CONFIG_SENSOR_POLY_TEMP_MIN) * CONFIG_SENSOR_POLY_STEPS_PER_DEGREE)
#define IDX_TO_TEMP(idx) (float)(((float)(idx) / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE) + CONFIG_SENSOR_POLY_TEMP_MIN)

// =============================================================================
// Boot Calibration - Runtime D_offset Calculation
// =============================================================================

// Boot calibration constants
#define BOOT_CAL_TIME_WINDOW_START_MS 15000 // 15 seconds after boot
#define BOOT_CAL_TIME_WINDOW_END_MS 50000  // 50 seconds after boot
#define BOOT_CAL_MAX_ATTEMPTS 3            // Maximum retry attempts
#define BOOT_CAL_MIN_CURVE_POINTS 4        // Minimum calibration points

// =============================================================================
// Runtime Periodic Zero Bias Calibration
// =============================================================================
// When the device is at rest for a short period, perform quick zero bias
// calibration to update D_offset. This helps maintain accuracy over long usage
// sessions when temperature curves may differ or fixed zero bias changes.

// Runtime calibration constants
#define RUNTIME_CAL_REST_TIME_MS 8000         // 8 seconds of rest before triggering
#define RUNTIME_CAL_COOLDOWN_MS 60000         // 1 minute cooldown between runtime calibrations
#define RUNTIME_CAL_MIN_UPTIME_MS 60000       // Wait at least 60s after boot before runtime cal
#define RUNTIME_CAL_TEMP_CHANGE_MIN 1.0f      // Minimum temperature change (°C) to trigger recalibration
#define RUNTIME_CAL_SAMPLE_TIME_MS 3000       // 3 seconds sampling time for runtime calibration
#define RUNTIME_CAL_FAILURE_COOLDOWN_MS 30000 // 30 seconds cooldown after calibration failure

// Runtime calibration state (not persisted)
static bool runtime_cal_enabled = false;       // Runtime calibration enabled (default: disabled)
static int64_t runtime_cal_last_time = 0;      // Last time runtime calibration was performed
static int64_t runtime_cal_rest_start = 0;     // When rest period started
static bool runtime_cal_rest_tracking = false; // Currently tracking rest period
static float runtime_cal_last_temp = NAN;      // Temperature at last calibration

// Forward declarations for boot calibration
static int sensor_boot_bias_collect(float *dest_bias, float *avg_temp);
static int sensor_runtime_bias_collect(float *dest_bias, float *avg_temp);
static int sensor_tcal_calculate_doffset(const float measured_bias[3], float temp);
static int sensor_perform_boot_calibration(void);
static int sensor_perform_runtime_calibration(void);


// Auto-calibration control
static bool tcal_auto_calibration_enabled = false;

static float last_gyro_tcal_offset[3] = {0.0f, 0.0f, 0.0f};

static void update_tcal_state(void); // Function to refresh T-Cal state

// =============================================================================
// T-Cal Moving Least Squares (MLS) Implementation
// =============================================================================
// MLS provides smooth, continuous bias estimation without discontinuities
// caused by method switching. Uses inverse distance weighting with local
// linear fitting for optimal balance of smoothness and responsiveness.

// MLS Configuration
#define MLS_BANDWIDTH 1.5f       // Temperature bandwidth (°C) - controls locality
#define MLS_MIN_WEIGHT 0.05f     // Minimum weight threshold (~6.5°C distance cutoff)
#define MLS_MAX_POINTS 10        // Maximum points to consider for efficiency
#define MLS_MIN_POINTS_FOR_FIT 4 // Minimum points with significant weight for MLS to be valid

/**
 * Moving Least Squares (MLS) lookup function
 * Computes weighted local linear fit at the query temperature
 *
 * For 1D linear MLS, we solve the weighted least squares problem:
 *   minimize Σ w_i * (b_i - (a + c*(t_i - t_query)))²
 * where w_i is the weight for point i, inversely related to distance
 *
 * The solution is:
 *   c = Σ w_i*(t_i-t_q)*(b_i-b_weighted_avg) / Σ w_i*(t_i-t_q)²
 *   a = b_weighted_avg
 *
 * @param temp Query temperature
 * @param bias_out Output: computed 3-axis bias
 * @return 0 on success, -1 if insufficient data
 */
static int sensor_tcal_mls_lookup(float temp, float bias_out[3]);

// =============================================================================
// MLS Cache - Performance Optimization
// =============================================================================
// Since temperature changes slowly (over seconds), we can cache the MLS result
// and only recompute when temperature changes significantly.
// Temperature sensor noise can be ~0.1°C, so we use a larger threshold
// to avoid unnecessary recomputation.
//
// We use multiple cache slots to cover a larger temperature range, which helps
// when temperature oscillates slightly within a small range.

// =============================================================================
// LUT (Look-Up Table) + Linear Interpolation - O(1) Runtime Lookup
// =============================================================================
// When calibration points change, pre-compute MLS output at fixed temperature
// grid points. Runtime lookup simply does linear interpolation between two
// adjacent grid points, achieving O(1) complexity without expensive MLS
// computation per query.
//
// LUT Configuration:
// - Step size: 0.5°C (2 steps per degree) - good balance of precision vs RAM
// - Temperature range: CONFIG_SENSOR_POLY_TEMP_MIN to CONFIG_SENSOR_POLY_TEMP_MAX
// - RAM usage: ~0.9KB for standard 10-45°C range
//
// Incremental Build Strategy:
// - At boot, first build entries within ±3°C of current temperature (priority zone)
// - Return quickly to allow other threads to run
// - Continue building remaining entries in small batches during idle time
// - LUT lookup falls back to MLS for entries not yet computed

#define MLS_LUT_STEP_PER_DEGREE 2   // Steps per degree (0.5°C per step)
#define MLS_LUT_STEP_SIZE (1.0f / MLS_LUT_STEP_PER_DEGREE)  // 0.5°C
#define MLS_LUT_TEMP_MIN ((float)CONFIG_SENSOR_POLY_TEMP_MIN)
#define MLS_LUT_TEMP_MAX ((float)CONFIG_SENSOR_POLY_TEMP_MAX)
#define MLS_LUT_SIZE ((int)((CONFIG_SENSOR_POLY_TEMP_MAX - CONFIG_SENSOR_POLY_TEMP_MIN) * MLS_LUT_STEP_PER_DEGREE) + 1)

// Incremental build configuration
#define MLS_LUT_PRIORITY_RANGE 3.0f  // ±3°C around current temp is priority zone
#define MLS_LUT_BATCH_SIZE 10        // Entries to compute per incremental batch
#define MLS_LUT_BATCH_YIELD_MS 10     // Sleep between batches to yield CPU

// Convert temperature to LUT index (continuous, for interpolation)
#define MLS_LUT_TEMP_TO_IDX(temp) (((temp) - MLS_LUT_TEMP_MIN) * MLS_LUT_STEP_PER_DEGREE)
// Convert LUT index to temperature
#define MLS_LUT_IDX_TO_TEMP(idx) (MLS_LUT_TEMP_MIN + (float)(idx) * MLS_LUT_STEP_SIZE)

typedef struct {
	float bias[3];  // Pre-computed MLS bias at this temperature
	bool computed;  // Whether this entry has been computed
} MlsLutEntry;

// LUT build state
typedef enum {
	MLS_LUT_BUILD_IDLE,         // No build in progress, LUT may be invalid or complete
	MLS_LUT_BUILD_PRIORITY,     // Building priority zone (±3°C around current temp)
	MLS_LUT_BUILD_BACKGROUND,   // Building remaining entries in background
	MLS_LUT_BUILD_COMPLETE      // All entries computed
} MlsLutBuildState;

static struct {
	MlsLutEntry entries[MLS_LUT_SIZE]; // Pre-computed bias values
	uint32_t version;                   // Point count when LUT was built (for invalidation)
	bool valid;                         // LUT has at least priority zone computed
	MlsLutBuildState build_state;       // Current build state
	int build_next_idx;                 // Next index to compute in background build
	int priority_idx_min;               // Priority zone minimum index
	int priority_idx_max;               // Priority zone maximum index
	int computed_count;                 // Number of entries computed so far
} mls_lut = {
	.entries = {{{0}}},
	.version = 0,
	.valid = false,
	.build_state = MLS_LUT_BUILD_IDLE,
	.build_next_idx = 0,
	.priority_idx_min = 0,
	.priority_idx_max = 0,
	.computed_count = 0
};

// Forward declarations for LUT functions
static void sensor_tcal_build_lut_priority(float current_temp);
static bool sensor_tcal_build_lut_continue(void);
static int sensor_tcal_lut_lookup(float temp, float bias_out[3]);

// =============================================================================
// Legacy MLS Cache (kept for fallback and LUT building)
// =============================================================================

#define MLS_CACHE_SLOTS 5            // Number of cache slots
#define MLS_CACHE_TEMP_THRESHOLD 0.5f // Match within this threshold of cached temp

typedef struct {
	float temp;        // Temperature at which cache was computed
	float bias[3];     // Cached bias values at temp
	float slope[3];    // Local d(bias)/d(temp) slope used for smooth cached interpolation
	bool valid;        // Cache validity flag
} MlsCacheSlot;

static struct {
	MlsCacheSlot slots[MLS_CACHE_SLOTS]; // Cache slots covering temperature range
	uint32_t count;                       // Point count when cached (invalidate all if points change)
} mls_cache = {
	.slots = {{0}},
	.count = 0
};

/**
 * Select the best cache slot to use for a new entry at the given temperature.
 * Strategy:
 * 1. If an invalid slot exists, use it
 * 2. Find the slot with the largest distance from the query temperature
 *    (this preserves nearby cached values for interpolation)
 * @param temp Query temperature for the new cache entry
 * @return Best slot index to use
 */
static int sensor_tcal_cache_select_slot(float temp)
{
	int best_slot = 0;
	float best_distance = -1.0f;

	for (int i = 0; i < MLS_CACHE_SLOTS; i++) {
		// Prefer invalid slots first
		if (!mls_cache.slots[i].valid) {
			return i;
		}
		// Find slot with largest distance from query temperature
		float distance = fabsf(mls_cache.slots[i].temp - temp);
		if (distance > best_distance) {
			best_distance = distance;
			best_slot = i;
		}
	}
	return best_slot;
}

// =============================================================================
// T-Cal Cache/LUT Invalidation (called when calibration points change)
// =============================================================================
static void sensor_tcal_cache_invalidate(void)
{
	// Invalidate legacy cache slots
	for (int i = 0; i < MLS_CACHE_SLOTS; i++) {
		mls_cache.slots[i].valid = false;
	}
	// Invalidate LUT and stop any incremental build in progress
	mls_lut.valid = false;
	mls_lut.build_state = MLS_LUT_BUILD_IDLE;
	mls_lut.computed_count = 0;
	// Mark all entries as not computed
	for (int i = 0; i < MLS_LUT_SIZE; i++) {
		mls_lut.entries[i].computed = false;
	}
	LOG_DBG("T-Cal cache/LUT invalidated, incremental build stopped");
}

#endif

// helpers
static bool wait_for_motion(bool motion, int samples);
static int check_sides(const float *);
static void magneto_reset(void);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static int isAccRest(float *, float *, float, int *, int);
#endif

// calibration logic
static int sensor_offsetBias_internal(
	float *dest1,
	float *dest2,
	float *avg_temp,
	float *temp_range,
	int max_sample_time_ms,
	int min_sample_time_ms
);
static int sensor_offsetBias(float *dest1, float *dest2, float *avg_temp, float *temp_range);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static int sensor_6_sideBias(float a_inv[][3], int *captured_count_out);
#endif
static void sensor_sample_mag_magneto_sample(const float a[3], const float m[3]);

static int sensor_calibration_request(int id);

static void calibration_thread(void);
K_THREAD_DEFINE(calibration_thread_id, 4096, calibration_thread, NULL, NULL, NULL, 6, 0, 0);

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
	float calculated_offset[3] = {0.0f, 0.0f, 0.0f};
	float temp = sensor_get_current_imu_temperature();
	bool offset_calculated = false;

	// ==========================================================================
	// Unified T-Cal Strategy: LUT -> MLS -> Static Bias
	// D_offset is always applied when valid
	// ==========================================================================

	if (!isnan(temp) && retained->tempCalState.count >= 1) {
		// Strategy 1: Try LUT lookup (preferred - O(1) linear interpolation)
		if (retained->tempCalState.count >= MLS_MIN_POINTS_FOR_FIT) {
			if (sensor_tcal_lut_lookup(temp, calculated_offset) == 0) {
				offset_calculated = true;
			}
			// Strategy 2: Fallback to MLS if LUT not available
			else if (sensor_tcal_mls_lookup(temp, calculated_offset) == 0) {
				offset_calculated = true;
			}
		}
	}

	// Strategy 2: Final fallback to static bias (when no T-Cal data available)
	if (!offset_calculated) {
		for (int i = 0; i < 3; i++) {
			calculated_offset[i] = gyroBias[i];
		}
		// Note: offset_calculated remains false but we still apply D_offset below
	}

	// Apply boot/runtime calibration D_offset
	// D_offset is now applied regardless of whether T-Cal is used or not
	// This allows runtime bias tracking even without temperature calibration
	if (retained->bootCalState.doffset_valid) {
#if CONFIG_CMSIS_DSP
		arm_add_f32(calculated_offset, retained->bootCalState.doffset, calculated_offset, 3);
#else
		for (int axis = 0; axis < 3; axis++) {
			calculated_offset[axis] += retained->bootCalState.doffset[axis];
		}
#endif
	}

	// Apply the calculated offset to gyro data
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

#if CONFIG_SENSOR_USE_TCAL
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
	// Always save gyroscope bias (ZRO is orientation-independent)
	sys_write(MAIN_GYRO_BIAS_ID, &retained->gyroBias, gyroBias, sizeof(gyroBias));

#if CONFIG_SENSOR_USE_TCAL
	// 2. Check if T-Cal coverage is good - if so, only calculate D_offset instead of saving point
	sys_write(MAIN_GYRO_TEMP_ID, &retained->gyroTemp, &avg_temp, sizeof(avg_temp));
	if (!isnan(avg_temp)) {
		// Check if current temperature has good T-Cal coverage
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
			// 2. Has both upper and lower bounds AND closest is within 2x sampling interval
			if (closest_distance <= sampling_interval) {
				// Very close to existing point - definitely good coverage
				has_good_coverage = true;
				LOG_INF(
					"T-Cal: Excellent coverage at %.2fC (closest: %.2fC away, within sampling interval)",
					(double)avg_temp,
					(double)closest_distance
				);
			} else if (has_lower_bound && has_upper_bound && closest_distance <= sampling_interval * 2.0f) {
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
						(double)(sampling_interval * 2.0f)
					);
				}
			}
		}

		if (has_good_coverage) {
			// Polynomial fit correction offset was removed; keep the retained field for compatibility,
			// but do not update/apply it. With good coverage, avoid saving redundant points.
			LOG_INF(
				"T-Cal: Coverage sufficient at %.2fC, skipping point save",
				(double)avg_temp
			);
		}

		if (!has_good_coverage) {
			// Normal path: save as new calibration point
			LOG_INF(
				"T-Cal: Saving calibration point at average temp %.2fC (range: %.2fC)",
				(double)avg_temp,
				(double)temp_range
			);
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
				update_tcal_state();

			} else {
				LOG_WRN(
					"T-Cal: Temperature %.2fC is outside the configured calibration range. Point not saved.",
					(double)avg_temp
				);
			}
		}
	}
#endif

	LOG_INF("Finished calibration");
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
// Minimum poses required for partial calibration save (must be before sensor_calibrate_6_side)
#define CALIB_MIN_POSES_FOR_PARTIAL 6

static void sensor_calibrate_6_side(void)
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
		/* Feed watchdog during long wait periods */
		watchdog_feed(WDT_CHANNEL_CALIBRATION);
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
	float delta[3];
	delta[0] = acc[0] - pre_acc[0];
	delta[1] = acc[1] - pre_acc[1];
	delta[2] = acc[2] - pre_acc[2];

#if CONFIG_CMSIS_DSP
	float norm_sq;
	arm_dot_prod_f32(delta, delta, 3, &norm_sq);
	float norm_diff;
	arm_sqrt_f32(norm_sq, &norm_diff);
#else
	float norm_diff = sqrtf(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
#endif

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

static int sensor_offsetBias_internal(
	float *dest1,
	float *dest2,
	float *avg_temp,
	float *temp_range,
	int max_sample_time_ms,
	int min_sample_time_ms
)
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

	double gyro_sum[3] = {0};

#if CONFIG_SENSOR_USE_TCAL
	double temp_sum = 0;
	float temp_min = INFINITY, temp_max = -INFINITY;
	float current_temp;
	int valid_temp_count = 0;

	// Record start temperature
	current_temp = sensor_get_current_imu_temperature();
	if (!isnan(current_temp) && current_temp > -10.0f && current_temp < 60.0f) {
		temp_min = current_temp;
		temp_max = current_temp;
	}
#endif

	int64_t sampling_start_time = k_uptime_get();
	int i = 0;
#if CONFIG_SENSOR_USE_TCAL
	bool temp_threshold_reached = false;
#endif

	// Accel motion check counter - check every N gyro samples to avoid blocking
	float actual_gyro_odr = sensor_get_gyro_odr();
	float actual_accel_odr = sensor_get_accel_odr();

	// Calculate effective ODRs after oversampling
	// sensor_wait_gyro receives data at effective_gyro_odr (after gyro oversampling)
	// sensor_wait_accel receives data at effective_accel_odr (after accel oversampling)
#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
	float effective_gyro_odr = actual_gyro_odr / CONFIG_SENSOR_GYRO_OVERSAMPLING;
#else
	float effective_gyro_odr = actual_gyro_odr;
#endif

#if CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1
	float effective_accel_odr = actual_accel_odr / CONFIG_SENSOR_ACCEL_OVERSAMPLING;
#else
	float effective_accel_odr = actual_accel_odr;
#endif

	int accel_check_interval = (int)(effective_gyro_odr / effective_accel_odr + 0.5f); // Round to nearest
	if (accel_check_interval < 1) {
		accel_check_interval = 1; // Ensure at least 1
	}
	int accel_check_counter = 0;

#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1 || CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1
	LOG_INF(
		"Calibration: ODR - Gyro: %.2fHz (eff: %.2fHz, %dx OS), Accel: %.2fHz (eff: %.2fHz, %dx OS), Check interval: "
		"%d",
		(double)actual_gyro_odr,
		(double)effective_gyro_odr,
		CONFIG_SENSOR_GYRO_OVERSAMPLING,
		(double)actual_accel_odr,
		(double)effective_accel_odr,
		CONFIG_SENSOR_ACCEL_OVERSAMPLING,
		accel_check_interval
	);
#else
	LOG_INF(
		"Calibration: Using actual ODR - Gyro: %.2fHz, Accel: %.2fHz, Check interval: %d",
		(double)actual_gyro_odr,
		(double)actual_accel_odr,
		accel_check_interval
	);
#endif

	// Collect samples with smart stop conditions
	// Main loop runs at gyro ODR, accel checked periodically
	int wdt_feed_counter = 0;
	while (true) {
		int64_t elapsed = k_uptime_get() - sampling_start_time;

		// Check stop conditions
		if (elapsed >= max_sample_time_ms) {
			LOG_INF("Max sampling time reached (%lld ms)", elapsed);
			break;
		}

		// Feed watchdog periodically during long sampling (~every 1 second based on gyro ODR)
		wdt_feed_counter++;
		if (wdt_feed_counter >= (int)effective_gyro_odr) {
			watchdog_feed(WDT_CHANNEL_CALIBRATION);
			wdt_feed_counter = 0;
		}

#if CONFIG_SENSOR_USE_TCAL
		// Check temperature threshold only after min_sample_time_ms
		if (elapsed >= min_sample_time_ms && temp_threshold_reached) {
			LOG_INF("Temperature threshold reached after %lld ms with %d samples", elapsed, i);
			break;
		}
#endif

		// Check accelerometer motion periodically (not every loop iteration)
		// This prevents the loop from being blocked by slower accel ODR
		if (accel_check_counter >= accel_check_interval) {
			if (sensor_wait_accel(rawData, K_MSEC(100))) {
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
				if (max_a[j] - min_a[j] > BIAS_COLLECT_ACCEL_MOTION_THRESHOLD) {
					LOG_INF("Accel motion detected: axis %d range %.4f", j, (double)(max_a[j] - min_a[j]));
					return -1;
				}
			}
			accel_check_counter = 0;
		}
		accel_check_counter++;

		// Accumulate Gyroscope
		if (sensor_wait_gyro(rawData, K_MSEC(100))) {
			return -2; // Timeout
		}

		// Check Gyro Motion
		for (int j = 0; j < 3; j++) {
			if (rawData[j] < min_g[j]) {
				min_g[j] = rawData[j];
			}
			if (rawData[j] > max_g[j]) {
				max_g[j] = rawData[j];
			}
			if (max_g[j] - min_g[j] > BIAS_COLLECT_GYRO_MOTION_THRESHOLD) {
				LOG_INF("Gyro motion detected: axis %d range %.4f", j, (double)(max_g[j] - min_g[j]));
				return -1;
			}
		}

		// Accumulate gyro data using online algorithm
		gyro_sum[0] += (double)rawData[0];
		gyro_sum[1] += (double)rawData[1];
		gyro_sum[2] += (double)rawData[2];

#if CONFIG_SENSOR_USE_TCAL
		// Sample and accumulate temperature
		current_temp = sensor_get_current_imu_temperature();
		if (!isnan(current_temp) && current_temp > -10.0f && current_temp < 60.0f) {
			temp_sum += (double)current_temp;
			valid_temp_count++;

			if (current_temp < temp_min) {
				temp_min = current_temp;
			}
			if (current_temp > temp_max) {
				temp_max = current_temp;
			}

			// Check if temperature range threshold exceeded
			if ((temp_max - temp_min) >= BIAS_COLLECT_TEMP_RANGE_THRESHOLD) {
				temp_threshold_reached = true;
			}
		}
#endif
		i++;
	}

	LOG_INF("Samples collected: %d", i);

	// Calculate minimum samples based on actual gyro ODR and min_sample_time_ms
	// Convert min_sample_time_ms to seconds for calculation
	float min_sample_time_sec = (float)min_sample_time_ms / 1000.0f;
#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1
	// With oversampling enabled, the effective sample rate sent to calibration is reduced
	// by the oversampling factor (e.g., 1600Hz / 4 = 400Hz effective)
	int min_samples_required = (int)(effective_gyro_odr * min_sample_time_sec);
	LOG_INF(
		"Calibration: Gyro oversampling %dx, effective ODR: %.2fHz",
		CONFIG_SENSOR_GYRO_OVERSAMPLING,
		(double)effective_gyro_odr
	);
#else
	int min_samples_required = (int)(actual_gyro_odr * min_sample_time_sec);
#endif

	if (i < min_samples_required) {
		LOG_WRN(
			"Not enough samples: %d < %d (based on actual gyro ODR: %.2fHz, min time: %dms)",
			i,
			min_samples_required,
			(double)actual_gyro_odr,
			min_sample_time_ms
		);
		return -2;
	}

#if CONFIG_SENSOR_USE_TCAL
	if (avg_temp != NULL && valid_temp_count > 0) {
		*avg_temp = (float)(temp_sum / valid_temp_count);
		LOG_INF("T-Cal: Average temperature: %.2fC (%d samples)", (double)*avg_temp, valid_temp_count);
	}

	if (temp_range != NULL) {
		*temp_range = temp_max - temp_min;
		LOG_INF(
			"T-Cal: Temperature range: %.2fC (%.2fC to %.2fC)",
			(double)*temp_range,
			(double)temp_min,
			(double)temp_max
		);
	}
#endif

	// Calculate averages
	dest2[0] = (float)(gyro_sum[0] / i);
	dest2[1] = (float)(gyro_sum[1] / i);
	dest2[2] = (float)(gyro_sum[2] / i);

#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	// In single-side calibration mode, do NOT calculate accelerometer bias.
	dest1[0] = 0.0f;
	dest1[1] = 0.0f;
	dest1[2] = 0.0f;
#endif

	return 0;
}

/**
 * Standard sensor offset bias collection function
 * Uses default timing: BIAS_COLLECT_MAX_SAMPLE_TIME_MS max, BIAS_COLLECT_MIN_SAMPLE_TIME_MS min
 */
int sensor_offsetBias(float *dest1, float *dest2, float *avg_temp, float *temp_range)
{
	return sensor_offsetBias_internal(
		dest1,
		dest2,
		avg_temp,
		temp_range,
		BIAS_COLLECT_MAX_SAMPLE_TIME_MS,
		BIAS_COLLECT_MIN_SAMPLE_TIME_MS
	);
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
// Target number of samples, 6 faces + 12 edges
#define CALIB_TARGET_SAMPLES 18
// Orientation difference threshold (cosine value).
// cos(25 degrees) ≈ 0.90. If dot product > 0.90, the angle between two directions is less than 25 degrees, considered
// duplicate.
#define MIN_ORIENTATION_DIFF_COS 0.90f
// Acceleration threshold for determining stationary state
#define THRESHOLD_ACC 0.02f
// Number of samples to collect for each orientation
#define SAMPLES_PER_ORIENTATION 500
// Timeout for waiting for new pose (90 seconds in milliseconds)
#define CALIB_POSE_TIMEOUT_MS 90000
typedef struct {
	float x, y, z;
} Vector3;

int sensor_6_sideBias(float a_inv[][3], int *captured_count_out)
{
	float rawData[3];
	float pre_acc[3] = {0};
	int resttime = 0;

	Vector3 captured_dirs[CALIB_TARGET_SAMPLES];
	int captured_count = 0;
	int64_t last_new_pose_time = k_uptime_get(); // Track time of last new pose

	// Initialize output parameter
	if (captured_count_out) {
		*captured_count_out = 0;
	}

	magneto_reset();

	LOG_INF("Starting Multi-Position Calibration (Target: %d poses)", CALIB_TARGET_SAMPLES);
	LOG_INF("Please rotate device to random orientations and hold still.");

	// Main loop: until target number of samples collected
	while (captured_count < CALIB_TARGET_SAMPLES) {

		// 1. Wait for device to be stationary
		set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_SENSOR); // Indicate searching for stationary state
		bool pose_timeout = false;
		while (1) {
			/* Feed watchdog during user interaction wait */
			watchdog_feed(WDT_CHANNEL_CALIBRATION);

			/* Check for timeout - no new pose in CALIB_POSE_TIMEOUT_MS */
			if ((k_uptime_get() - last_new_pose_time) > CALIB_POSE_TIMEOUT_MS) {
				LOG_WRN("Timeout: No new pose detected for %d seconds", CALIB_POSE_TIMEOUT_MS / 1000);
				pose_timeout = true;
				break;
			}

			if (sensor_wait_accel(rawData, K_MSEC(1000))) {
				return -2; // Timeout, magneto state not handled here
			}

			int rest = isAccRest(rawData, pre_acc, THRESHOLD_ACC, &resttime, 100);
			memcpy(pre_acc, rawData, sizeof(rawData)); // Update previous data

			if (rest == 1) {
				// Device is stationary, now check if this pose is new
				// Calculate current vector magnitude
#if CONFIG_CMSIS_DSP
				float norm_sq;
				arm_dot_prod_f32(rawData, rawData, 3, &norm_sq);
				float norm;
				arm_sqrt_f32(norm_sq, &norm);
#else
				float norm = sqrtf(rawData[0] * rawData[0] + rawData[1] * rawData[1] + rawData[2] * rawData[2]);
#endif
				if (norm < 0.1f) {
					continue; // Prevent division by zero (unlikely under gravity)
				}

				// Normalize current vector
				float curr_dir_x = rawData[0] / norm;
				float curr_dir_y = rawData[1] / norm;
				float curr_dir_z = rawData[2] / norm;

				bool is_duplicate = false;
				// Compare with historical data
				for (int i = 0; i < captured_count; i++) {
					// Calculate cosine of angle using dot product
					float dot = curr_dir_x * captured_dirs[i].x + curr_dir_y * captured_dirs[i].y
							  + curr_dir_z * captured_dirs[i].z;

					// If dot product is close to 1, the directions are almost the same,
					// or a slight wobble of a previous direction
					if (dot > MIN_ORIENTATION_DIFF_COS) {
						is_duplicate = true;
						break;
					}
				}

				if (!is_duplicate) {
					// This is a new valid pose! Break the wait loop and start capturing data
					// Save this direction vector
					captured_dirs[captured_count].x = curr_dir_x;
					captured_dirs[captured_count].y = curr_dir_y;
					captured_dirs[captured_count].z = curr_dir_z;
					break;
				} else {
					// Duplicate pose detected, briefly flash LED to prompt user to change orientation, but do not error
					set_led(SYS_LED_PATTERN_FLASH, SYS_LED_PRIORITY_SENSOR);
					k_msleep(100); // Debounce slightly
				}
			}
			k_msleep(20);
		}

		// Check if we timed out waiting for a new pose
		if (pose_timeout) {
			if (captured_count_out) {
				*captured_count_out = captured_count;
			}
			// Return -3 for timeout with captured_count available for partial save decision
			return -3;
		}

		// Reset timeout counter when a new valid pose is found
		last_new_pose_time = k_uptime_get();

		LOG_INF("Capturing pose %d/%d...", captured_count + 1, CALIB_TARGET_SAMPLES);
		set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_SENSOR);

		int sample_idx = 0;
		while (sample_idx < SAMPLES_PER_ORIENTATION) {
			if (sensor_wait_accel(rawData, K_MSEC(1000))) {
				return -2;
			}

			if (!v_epsilon(rawData, pre_acc, 0.03f)) {
				LOG_INF("Motion detected during capture, retrying...");
				sample_idx = -1;
				break;
			}
			memcpy(pre_acc, rawData, sizeof(rawData));

			magneto_sample(rawData[0], rawData[1], rawData[2], ata, &norm_sum, &sample_count);

			sample_idx++;

			if (sample_idx % 50 == 0) {
				printk(".");
				/* Feed watchdog periodically during sampling */
				watchdog_feed(WDT_CHANNEL_CALIBRATION);
			}
		}

		if (sample_idx == -1) {
			continue;
		}

		captured_count++;
		set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
		LOG_INF("Pose %d saved!", captured_count);

		k_msleep(500);
	}

	if (captured_count_out) {
		*captured_count_out = captured_count;
	}

	LOG_INF("Calculating calibration matrix...");

	wait_for_threads();
	magneto_current_calibration(a_inv, ata, norm_sum, sample_count);

	magneto_reset();

	LOG_INF("Calibration calculation complete.");
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
	/* Register calibration thread with watchdog - use long timeout for lengthy operations */
	watchdog_register_thread(WDT_CHANNEL_CALIBRATION, 0);

	sensor_calibration_read();

#if CONFIG_SENSOR_USE_TCAL
	// Wait for sensor to be initialized before building LUT
	// This prevents issues when sensor fails to initialize
	int init_wait_count = 0;
	while (!sensor_is_initialized()) {
		watchdog_feed(WDT_CHANNEL_CALIBRATION);
		k_msleep(100);
		init_wait_count++;
		// Timeout after 10 seconds to avoid infinite loop if sensor never initializes
		if (init_wait_count >= 100) {
			LOG_WRN("T-Cal: Timeout waiting for sensor initialization, skipping LUT build");
			break;
		}
	}

	// Build LUT at startup if T-Cal data is available and sensor is initialized
	// LUT is only in RAM and needs to be rebuilt after every boot
	// Use incremental build: priority zone first, then background completion
	if (sensor_is_initialized() && retained->tempCalState.count >= MLS_MIN_POINTS_FOR_FIT) {
		// Validate tempCalState.count to prevent issues with corrupted retained data
		if (retained->tempCalState.count > TCAL_BUFFER_SIZE) {
			LOG_ERR("T-Cal: Invalid point count %u (max %d), resetting",
			        retained->tempCalState.count, TCAL_BUFFER_SIZE);
			retained->tempCalState.count = 0;
			retained->tempCalState.valid = false;
		} else {
			float current_temp = sensor_get_current_imu_temperature();
			if (!isnan(current_temp)) {
				LOG_INF("T-Cal: Starting incremental LUT build at startup (current temp: %.1f°C)", (double)current_temp);
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
		default:
			if (magneto_progress & 0b10000000) {
				requested = sensor_calibrate_mag();
			}
			break;
		}

#if CONFIG_SENSOR_USE_TCAL
		// Continue LUT background build if in progress
		if (mls_lut.build_state == MLS_LUT_BUILD_BACKGROUND) {
			if (sensor_tcal_build_lut_continue()) {
				LOG_INF("T-Cal LUT: Background build complete (%d/%d entries)",
				        mls_lut.computed_count, MLS_LUT_SIZE);
			}
		}
#endif

		/* Feed watchdog at end of each loop iteration */
		watchdog_feed(WDT_CHANNEL_CALIBRATION);

		if (requested < 0) {
			k_msleep(5);
		} else {
			k_msleep(100);
		}
	}
}

#if CONFIG_SENSOR_USE_TCAL

void sensor_tcal_status(void)
{
	printk("Temperature Calibration Status (MLS):\n");
	printk("  - MLS available: %s\n", retained->tempCalState.valid ? "Yes" : "No");
	printk("  - Points collected: %u / %d\n", retained->tempCalState.count, TCAL_BUFFER_SIZE);

	// Display LUT status
	const char *lut_state_str;
	switch (mls_lut.build_state) {
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
	printk("  - LUT valid: %s, state: %s, entries: %d/%d\n",
	       mls_lut.valid ? "Yes" : "No",
	       lut_state_str,
	       mls_lut.computed_count,
	       MLS_LUT_SIZE);

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

static void update_tcal_state(void)
{
	// Invalidate lookup cache since calibration data changed
	sensor_tcal_cache_invalidate();

	// Polynomial coefficients are no longer used; keep persisted storage zeroed
	memset(retained->tempCalCoeffs, 0, sizeof(retained->tempCalCoeffs));
	retained->tempCalState.degree = 0;

	// Mark MLS availability based on point count
	retained->tempCalState.valid = (retained->tempCalState.count >= 1);

	if (retained->tempCalState.valid) {
		LOG_INF("T-Cal: MLS state refreshed with %u points", retained->tempCalState.count);
		printk("T-Cal: MLS data refreshed successfully.\n");

		// Start incremental LUT build for O(1) runtime lookup
		// Priority zone (current temp ±3°C) is built immediately
		// Remaining entries are built in background by calibration_thread
		float current_temp = sensor_get_current_imu_temperature();
		if (!isnan(current_temp)) {
			sensor_tcal_build_lut_priority(current_temp);
		}
	} else {
		LOG_INF("T-Cal: No points available");
		printk("T-Cal: No points available.\n");
	}

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

	// Update fusion bias while preserving orientation
	sensor_fusion_update_bias(NULL);
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
		update_tcal_state(); // Refresh and save
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
	static int64_t last_calibration_time = 0;
	static float last_temp = NAN;
	static float min_temp_since_last_check = NAN;
	static int64_t temp_history_time = 0;

	int64_t now = k_uptime_get();

	// Check if auto-calibration is enabled
	if (!tcal_auto_calibration_enabled) {
		return;
	}

	// Prevent starting new calibration while previous one is still running
	// Use calibration cooldown time (BIAS_COLLECT_MAX_SAMPLE_TIME_MS + margin)
	const int64_t calibration_cooldown_ms = BIAS_COLLECT_MAX_SAMPLE_TIME_MS + 10000; // 5s sampling + 10s margin = 15s
	if ((now - last_calibration_time) < calibration_cooldown_ms) {
		return;
	}

	// Check if temperature is outside calibrated range
	float closest_temp, distance;
	bool needs_temp_cal = sensor_tcal_is_temp_outside_range(current_temp, &closest_temp, &distance);

	// Only proceed if temperature calibration is needed
	if (!needs_temp_cal) {
		// Update temperature tracking even when calibration is not needed
		if (!isnan(current_temp)) {
			if (isnan(min_temp_since_last_check) || current_temp < min_temp_since_last_check) {
				min_temp_since_last_check = current_temp;
			}
		}
		return;
	}

	// Track temperature trend - only calibrate when temperature is rising
	bool is_temp_rising = false;

	// Validate current temperature reading
	if (isnan(current_temp) || current_temp < -10.0f || current_temp > 60.0f) {
		LOG_WRN("T-Cal Auto: Invalid temperature reading %.2fC - skipping", (double)current_temp);
		return;
	}

	// Initialize tracking on first run
	if (isnan(last_temp)) {
		last_temp = current_temp;
		min_temp_since_last_check = current_temp;
		temp_history_time = now;
		LOG_DBG("T-Cal Auto: Initializing temperature tracking at %.2fC", (double)current_temp);
		return;
	}

	// Update minimum temperature tracker
	if (isnan(min_temp_since_last_check) || current_temp < min_temp_since_last_check) {
		min_temp_since_last_check = current_temp;
	}

	// Check temperature trend every 10 seconds to smooth out sensor noise
	const int64_t temp_check_interval_ms = 10000;
	if ((now - temp_history_time) >= temp_check_interval_ms) {
		float temp_change = current_temp - last_temp;
		float temp_rise_from_min = current_temp - min_temp_since_last_check;

		// Temperature thresholds adjusted for typical IMU sensor accuracy (~1°C) and noise (~0.1-0.5°C)
		const float short_term_rise_threshold = 0.3f; // 0.3°C over 10s - clear rising trend
		const float long_term_rise_threshold = 0.5f;  // 0.5°C from minimum - significant rise after drop

		// Temperature is rising if:
		// 1. It increased compared to last check (short-term trend, above noise level)
		// 2. OR it's rising significantly from recent minimum (handles temp drop then rise case)
		if (temp_change >= short_term_rise_threshold || temp_rise_from_min >= long_term_rise_threshold) {
			is_temp_rising = true;
			if (temp_rise_from_min >= long_term_rise_threshold) {
				LOG_INF(
					"T-Cal Auto: Temperature rising from recent minimum (%.2fC -> %.2fC, +%.2fC)",
					(double)min_temp_since_last_check,
					(double)current_temp,
					(double)temp_rise_from_min
				);
			} else {
				LOG_INF(
					"T-Cal Auto: Temperature rising (%.2fC -> %.2fC, +%.2fC over %llds)",
					(double)last_temp,
					(double)current_temp,
					(double)temp_change,
					temp_check_interval_ms / 1000
				);
			}
		} else {
			LOG_DBG(
				"T-Cal Auto: Temperature not rising (%.2fC -> %.2fC, change: %.2fC, from min: +%.2fC) - skipping",
				(double)last_temp,
				(double)current_temp,
				(double)temp_change,
				(double)temp_rise_from_min
			);
		}

		// Update temperature history
		last_temp = current_temp;
		temp_history_time = now;
	}

	// Only trigger calibration if temperature is rising
	if (!is_temp_rising) {
		return;
	}

	// Provide friendly log message based on whether calibration points exist
	if (isnan(closest_temp)) {
		LOG_INF("T-Cal Auto: Temp %.2fC needs calibration (no existing calibration points)", (double)current_temp);
	} else {
		LOG_INF(
			"T-Cal Auto: Temp %.2fC needs calibration (closest: %.2fC, distance: %.2fC)",
			(double)current_temp,
			(double)closest_temp,
			(double)distance
		);
	}

	LOG_INF("T-Cal Auto: Requesting auto-calibration (device is resting, temperature rising)");

	// Request standard calibration (which will include tcal)
	// The calibration routine itself will check if device is still using wait_for_motion()
	int request_result = sensor_calibration_request(1);

	// Only update timing if request was successful
	if (request_result == 0) {
		// Record calibration start time
		last_calibration_time = now;
		// Reset minimum temperature tracker after triggering calibration
		min_temp_since_last_check = current_temp;
	} else {
		LOG_DBG("T-Cal Auto: Calibration request rejected (already running), will retry later");
	}
}

void sensor_calibration_get_last_gyro_offset(float offset[3])
{
	memcpy(offset, last_gyro_tcal_offset, sizeof(last_gyro_tcal_offset));
}

// =============================================================================
// Boot Calibration Implementation
// =============================================================================

/**
 * Assess temperature calibration quality for boot calibration
 * Checks if MLS has enough points with significant weight at the current temperature
 */
bool sensor_tcal_assess_quality(float current_temp, tcal_quality_t *quality)
{
	if (!quality) {
		return false;
	}

	// Initialize quality structure
	quality->curve_valid = retained->tempCalState.valid;
	quality->point_count = retained->tempCalState.count;
	quality->curve_error = 0.0f;
	quality->temp_min = INFINITY;
	quality->temp_max = -INFINITY;
	quality->temp_in_range = false;

	// Check minimum global point count
	if (quality->point_count < BOOT_CAL_MIN_CURVE_POINTS) {
		LOG_DBG("T-Cal: Insufficient points (%u < %d)", quality->point_count, BOOT_CAL_MIN_CURVE_POINTS);
		return false;
	}

	// Count points with significant weight at current temperature (MLS bandwidth check)
	// This ensures MLS will actually have usable data at this temperature
	float bandwidth_sq = MLS_BANDWIDTH * MLS_BANDWIDTH;
	int points_with_weight = 0;

	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (retained->tempCalPoints[i].temp != 0.0f) {
			float point_temp = retained->tempCalPoints[i].temp;

			// Track temperature range
			if (point_temp < quality->temp_min) {
				quality->temp_min = point_temp;
			}
			if (point_temp > quality->temp_max) {
				quality->temp_max = point_temp;
			}

			// Calculate weight at current temperature
			float d = point_temp - current_temp;
			float d_sq = d * d;
			float weight = 1.0f / (1.0f + d_sq / bandwidth_sq);

			// Count if weight is significant (>= MLS_MIN_WEIGHT)
			if (weight >= MLS_MIN_WEIGHT) {
				points_with_weight++;
			}
		}
	}

	// Check if current temperature is within or near the calibrated range
	if (current_temp >= quality->temp_min && current_temp <= quality->temp_max) {
		quality->temp_in_range = true;
	}

	// MLS needs at least 2 points with significant weight for linear fit
	if (points_with_weight < MLS_MIN_POINTS_FOR_FIT) {
		LOG_DBG(
			"T-Cal: Only %d point(s) with significant weight at %.2fC (need %d within %.1fC bandwidth)",
			points_with_weight,
			(double)current_temp,
			MLS_MIN_POINTS_FOR_FIT,
			(double)MLS_BANDWIDTH
		);
		return false;
	}

	// Log quality status (only once to avoid spam)
	static bool logged_quality = false;
	if (!logged_quality) {
		LOG_INF(
			"T-Cal: Quality check passed - %d points with weight at %.2fC (range: [%.2fC, %.2fC])",
			points_with_weight,
			(double)current_temp,
			(double)quality->temp_min,
			(double)quality->temp_max
		);
		logged_quality = true;
	}

	return true;
}

/**
 * Collect bias at current temperature (reuses standard calibration logic)
 * Does NOT save the point to calibration data
 */
static int sensor_boot_bias_collect(float *dest_bias, float *avg_temp)
{
	LOG_INF("Boot Cal: Starting bias collection (4-6 seconds)...");

	// Use the existing sensor_offsetBias function with same parameters
	// This ensures consistent quality between boot cal and normal cal
	float temp_range = NAN;
	float dummy_accel_bias[3] = {0};

	int err = sensor_offsetBias(dummy_accel_bias, dest_bias, avg_temp, &temp_range);

	if (err) {
		if (err == -1) {
			LOG_INF("Boot Cal: Motion detected during collection");
		} else if (err == -2) {
			LOG_ERR("Boot Cal: Timeout during collection");
		} else if (err == -3) {
			LOG_WRN("Boot Cal: Temperature unstable during collection");
		}
		return err;
	}

	LOG_INF(
		"Boot Cal: Collected bias [%.5f, %.5f, %.5f] at temp %.2fC (range: %.2fC)",
		(double)dest_bias[0],
		(double)dest_bias[1],
		(double)dest_bias[2],
		(double)*avg_temp,
		(double)temp_range
	);

	return 0;
}

/**
 * Collect bias for runtime calibration with shorter sampling time
 * Uses RUNTIME_CAL_SAMPLE_TIME_MS instead of BIAS_COLLECT_MAX_SAMPLE_TIME_MS
 * Does NOT save the point to calibration data
 */
static int sensor_runtime_bias_collect(float *dest_bias, float *avg_temp)
{
	LOG_INF("Runtime Cal: Starting bias collection (%d seconds)...", RUNTIME_CAL_SAMPLE_TIME_MS / 1000);

	// Use internal function with shorter sampling time for runtime calibration
	float temp_range = NAN;
	float dummy_accel_bias[3] = {0};

	// Runtime calibration uses shorter max time (3s) and shorter min time (2s)
	int min_sample_time = RUNTIME_CAL_SAMPLE_TIME_MS * 2 / 3; // ~2 seconds minimum
	int err = sensor_offsetBias_internal(
		dummy_accel_bias,
		dest_bias,
		avg_temp,
		&temp_range,
		RUNTIME_CAL_SAMPLE_TIME_MS,
		min_sample_time
	);

	if (err) {
		if (err == -1) {
			LOG_INF("Runtime Cal: Motion detected during collection");
		} else if (err == -2) {
			LOG_ERR("Runtime Cal: Timeout during collection");
		} else if (err == -3) {
			LOG_WRN("Runtime Cal: Temperature unstable during collection");
		}
		return err;
	}

	LOG_INF(
		"Runtime Cal: Collected bias [%.5f, %.5f, %.5f] at temp %.2fC (range: %.2fC)",
		(double)dest_bias[0],
		(double)dest_bias[1],
		(double)dest_bias[2],
		(double)*avg_temp,
		(double)temp_range
	);

	return 0;
}

/**
 * Calculate D_offset and store in runtime state (not persisted)
 * Uses unified strategy: MLS -> Skip if insufficient quality
 *
 * Skip D_offset calculation if:
 * 1. No valid temperature calibration (< 5 points or current temp not covered)
 * 2. Only basic single-point zero bias calibration exists
 *
 * This prevents using unreliable bias estimates from incomplete calibration.
 * Requires more than 4 sampling points to ensure proper temperature coverage.
 */
static int sensor_tcal_calculate_doffset(const float measured_bias[3], float temp)
{
	// Check temperature calibration quality first
	tcal_quality_t quality;
	bool has_valid_tcal = sensor_tcal_assess_quality(temp, &quality);

	// Skip D_offset calculation if:
	// 1. No T-Cal data at all (count == 0)
	// 2. Not enough points (need > 4 points, i.e., at least 5 points)
	// 3. Current temperature is not covered by calibration points
	if (!has_valid_tcal || quality.point_count <= 4 || !quality.temp_in_range) {
		LOG_INF("Boot Cal: Skipping D_offset calculation - insufficient T-Cal quality");
		if (quality.point_count <= 4) {
			LOG_INF("Boot Cal: Only %u calibration point(s), need more than 4 for reliable offset", quality.point_count);
		}
		if (!quality.temp_in_range && quality.point_count > 0) {
			LOG_INF(
				"Boot Cal: Current temp %.2fC outside calibrated range [%.2fC, %.2fC]",
				(double)temp,
				(double)quality.temp_min,
				(double)quality.temp_max
			);
		}

		// Mark D_offset as invalid - use existing ZRO calibration only
		retained->bootCalState.doffset_valid = false;
		retained->bootCalState.doffset[0] = 0.0f;
		retained->bootCalState.doffset[1] = 0.0f;
		retained->bootCalState.doffset[2] = 0.0f;
		return 0; // Not an error, just skipped
	}

	// Calculate curve value at current temperature using MLS
	float curve_bias[3];
	bool offset_calculated = false;
	const char *method_name = "MLS";

	if (sensor_tcal_mls_lookup(temp, curve_bias) == 0) {
		offset_calculated = true;
		LOG_INF("D_offset: Using MLS method");
	}

	// If method failed, this should not happen since we checked quality
	// but handle it gracefully
	if (!offset_calculated) {
		LOG_ERR("D_offset: Failed to calculate curve bias despite passing quality check");
		retained->bootCalState.doffset_valid = false;
		retained->bootCalState.doffset[0] = 0.0f;
		retained->bootCalState.doffset[1] = 0.0f;
		retained->bootCalState.doffset[2] = 0.0f;
		return -1;
	}

	LOG_INF(
		"D_offset: Baseline (%s) [%.5f, %.5f, %.5f] at temp %.2fC",
		method_name,
		(double)curve_bias[0],
		(double)curve_bias[1],
		(double)curve_bias[2],
		(double)temp
	);

// Calculate D_offset = measured - curve
// Apply a minimum threshold to filter out noise - values below threshold are set to 0
#define BOOT_CAL_DOFFSET_MIN_THRESHOLD 0.001f // dps - ignore tiny corrections

	for (int axis = 0; axis < 3; axis++) {
		float doffset = measured_bias[axis] - curve_bias[axis];

		// Apply threshold: if D_offset is too small, it's likely noise - don't correct
		if (fabsf(doffset) < BOOT_CAL_DOFFSET_MIN_THRESHOLD) {
			retained->bootCalState.doffset[axis] = 0.0f;
		} else {
			retained->bootCalState.doffset[axis] = doffset;
		}
	}

	retained->bootCalState.doffset_valid = true;

	LOG_INF(
		"D_offset: Calculated [%.5f, %.5f, %.5f] (stored in retained memory)",
		(double)retained->bootCalState.doffset[0],
		(double)retained->bootCalState.doffset[1],
		(double)retained->bootCalState.doffset[2]
	);

	return 0;
}

/**
 * Main boot calibration check function
 * Called from sensor loop, manages state and timing
 * This function only checks conditions and requests calibration,
 * the actual calibration is performed by the calibration thread
 *
 * Boot calibration now works in three modes:
 * 1. With T-Cal data: Calculate D_offset as difference from T-Cal curve
 * 2. With static gyroBias: Calculate D_offset as difference from static bias
 * 3. No calibration data: Measure and store runtime bias directly
 */
void sensor_tcal_boot_calibration_check(void)
{
	// Check if feature is enabled
	if (!retained->bootCalState.enabled) {
		return;
	}

	// Check if already completed or requested
	if (retained->bootCalState.completed) {
		return;
	}

	// Check time window using uptime
	int64_t uptime = k_uptime_get();

	// Before window starts
	if (uptime < BOOT_CAL_TIME_WINDOW_START_MS) {
		return;
	}

	// After window ends - give up
	if (uptime >= BOOT_CAL_TIME_WINDOW_END_MS) {
		if (!retained->bootCalState.completed) {
			LOG_INF("Boot Cal: Time window expired (uptime: %lld ms), giving up", uptime);
			retained->bootCalState.completed = true;
		}
		return;
	}

	// Check if another calibration is running
	if (sensor_calibration_request(0) != 0) {
		return; // Calibration in progress, wait
	}

	// Get current temperature
	float current_temp = sensor_get_current_imu_temperature();
	if (isnan(current_temp) || current_temp < -10.0f || current_temp > 60.0f) {
		return; // Invalid temperature
	}

	// Check T-Cal quality before proceeding
	// Boot calibration is only useful with sufficient T-Cal data
	// Skip if we have insufficient calibration points (<=4)
	tcal_quality_t quality;
	bool has_tcal = sensor_tcal_assess_quality(current_temp, &quality);

	// Log entry info (only once)
	static bool logged_entry = false;
	if (!logged_entry) {
		if (has_tcal && quality.point_count > BOOT_CAL_MIN_CURVE_POINTS) {
			LOG_INF("Boot Cal: Will use T-Cal data (%u points) for D_offset calculation", quality.point_count);
		} else if (quality.point_count > 0 && quality.point_count <= BOOT_CAL_MIN_CURVE_POINTS) {
			LOG_INF("Boot Cal: Insufficient T-Cal points (%u <= %d), skipping boot calibration", quality.point_count, BOOT_CAL_MIN_CURVE_POINTS);
			retained->bootCalState.completed = true; // Mark as completed to avoid repeated checks
			return; // Skip boot calibration
		} else {
			LOG_INF("Boot Cal: No T-Cal data, skipping boot calibration");
			retained->bootCalState.completed = true; // Mark as completed to avoid repeated checks
			return; // Skip boot calibration
		}
		logged_entry = true;
	} else {
		// Check already logged, but still need to verify quality for this iteration
		if (!has_tcal || quality.point_count <= BOOT_CAL_MIN_CURVE_POINTS) {
			// Skip silently - already logged on first check
			if (!retained->bootCalState.completed) {
				retained->bootCalState.completed = true;
			}
			return;
		}
	}

	// Log entry into time window (only once)
	static bool logged_window_entry = false;
	if (!logged_window_entry) {
		LOG_INF("Boot Cal: In time window (5-30s), uptime: %lld ms, waiting for stationary condition...", uptime);
		logged_window_entry = true;
	}

	// Request boot calibration through calibration request system
	// This will be executed by the calibration thread, avoiding deadlock
	int request_result = sensor_calibration_request(3); // Use ID 3 for boot calibration
	if (request_result == 0) {
		LOG_INF("Boot Cal: Requested calibration through calibration thread");
	}
}

/**
 * Perform boot calibration (called by calibration thread)
 * Returns 0 on success, non-zero on failure
 *
 * Note: This is an automatic calibration - no LED changes to keep it
 * transparent to the user. LED state is preserved throughout.
 */
static int sensor_perform_boot_calibration(void)
{
	LOG_INF("Boot Cal: Starting boot calibration");
	// Note: No LED changes for automatic boot calibration - keep it transparent

	// Get current temperature
	float current_temp = sensor_get_current_imu_temperature();
	if (isnan(current_temp) || current_temp < -10.0f || current_temp > 60.0f) {
		LOG_ERR("Boot Cal: Invalid temperature");
		return -1;
	}

	// Wait for device to be stationary
	if (!wait_for_motion(false, 6)) {
		LOG_WRN("Boot Cal: Device not stationary");
		retained->bootCalState.attempt_count++;

		if (retained->bootCalState.attempt_count >= BOOT_CAL_MAX_ATTEMPTS) {
			LOG_WRN("Boot Cal: Maximum attempts (%d) reached, giving up", BOOT_CAL_MAX_ATTEMPTS);
			retained->bootCalState.completed = true;
		}
		return -1;
	}

	k_msleep(500); // Delay before beginning acquisition

	// Attempt to collect bias
	float measured_bias[3];
	float avg_temp;

	int err = sensor_boot_bias_collect(measured_bias, &avg_temp);

	if (err) {
		// Collection failed - check if we should trigger a full calibration
		retained->bootCalState.attempt_count++;

		if (retained->bootCalState.attempt_count >= BOOT_CAL_MAX_ATTEMPTS) {
			LOG_WRN("Boot Cal: Maximum attempts (%d) reached", BOOT_CAL_MAX_ATTEMPTS);

			// Check if we should auto-trigger single-side calibration to collect data
			tcal_quality_t quality;
			if (!sensor_tcal_assess_quality(current_temp, &quality)) {
				LOG_INF("Boot Cal: T-Cal quality insufficient, triggering single-side calibration");
				retained->bootCalState.completed = true; // Mark boot cal as complete to avoid re-entry

				// Request standard calibration to collect tcal data
				sensor_request_calibration();
				return -2; // Special error code indicating auto-calibration triggered
			}

			retained->bootCalState.completed = true;
		}
		return err;
	}

	// Calculate D_offset
	err = sensor_tcal_calculate_doffset(measured_bias, avg_temp);
	if (err) {
		LOG_ERR("Boot Cal: Failed to calculate D_offset");
		retained->bootCalState.completed = true;
		return err;
	}

	// Success! Update fusion bias while preserving orientation
	retained->bootCalState.completed = true;

	// Record temperature and time for runtime calibration comparison
	runtime_cal_last_temp = avg_temp;
	runtime_cal_last_time = k_uptime_get();

	LOG_INF("Boot Cal: Completed successfully at %.2fC (uptime: %lld ms)", (double)avg_temp, runtime_cal_last_time);
	sensor_fusion_update_bias(NULL);

	// Note: No LED flash for automatic boot calibration - keep it transparent
	return 0;
}

// Enable/disable boot calibration
void sensor_boot_cal_set_enabled(bool enabled)
{
	retained->bootCalState.enabled = enabled;
	LOG_INF("Boot Cal: %s", enabled ? "Enabled" : "Disabled");
}

// Get boot calibration status
bool sensor_boot_cal_is_completed(void)
{
	return retained->bootCalState.completed;
}

// Get boot calibration D_offset
void sensor_boot_cal_get_doffset(float offset[3])
{
	if (retained->bootCalState.doffset_valid) {
		memcpy(offset, retained->bootCalState.doffset, sizeof(retained->bootCalState.doffset));
	} else {
		memset(offset, 0, sizeof(retained->bootCalState.doffset));
	}
}

// Reset boot calibration state (call before reboot/shutdown, not before WoM)
void sensor_boot_cal_reset(void)
{
	retained->bootCalState.completed = false;
	retained->bootCalState.attempt_count = 0;
	retained->bootCalState.doffset_valid = false;
	retained->bootCalState.doffset[0] = 0.0f;
	retained->bootCalState.doffset[1] = 0.0f;
	retained->bootCalState.doffset[2] = 0.0f;
	LOG_INF("Boot Cal: State reset (will recalibrate on next boot)");
}

// =============================================================================
// Runtime Periodic Zero Bias Calibration Implementation
// =============================================================================

/**
 * Perform runtime zero bias calibration
 * Called by calibration thread when device has been at rest for extended period
 * This updates D_offset to track bias drift during long usage sessions
 *
 * Uses shorter sampling time (3 seconds) compared to normal calibration (4-6 seconds)
 * for quicker response while maintaining reasonable accuracy
 *
 * Note: This is an automatic calibration - no LED changes to keep it
 * transparent to the user. LED state is preserved throughout.
 */
static int sensor_perform_runtime_calibration(void)
{
	LOG_INF("Runtime Cal: Starting quick zero bias calibration (~3 seconds)");
	// Note: No LED changes for automatic runtime calibration - keep it transparent

	// Get current temperature
	float current_temp = sensor_get_current_imu_temperature();
	if (isnan(current_temp) || current_temp < -10.0f || current_temp > 60.0f) {
		LOG_ERR("Runtime Cal: Invalid temperature");
		// Apply failure cooldown to prevent immediate retry
		runtime_cal_last_time = k_uptime_get() - RUNTIME_CAL_COOLDOWN_MS + RUNTIME_CAL_FAILURE_COOLDOWN_MS;
		return -1;
	}

	// Collect bias using short sampling period
	// Uses sensor_runtime_bias_collect with RUNTIME_CAL_SAMPLE_TIME_MS
	float measured_bias[3];
	float avg_temp;

	int err = sensor_runtime_bias_collect(measured_bias, &avg_temp);

	if (err) {
		LOG_WRN("Runtime Cal: Bias collection failed (err: %d)", err);
		// Apply failure cooldown to prevent immediate retry
		runtime_cal_last_time = k_uptime_get() - RUNTIME_CAL_COOLDOWN_MS + RUNTIME_CAL_FAILURE_COOLDOWN_MS;
		return err;
	}

	// Calculate D_offset using the unified function
	// This works regardless of whether T-Cal data exists
	err = sensor_tcal_calculate_doffset(measured_bias, avg_temp);
	if (err) {
		LOG_ERR("Runtime Cal: Failed to calculate D_offset");
		return err;
	}

	// Update runtime calibration timestamp and temperature
	runtime_cal_last_time = k_uptime_get();
	runtime_cal_last_temp = avg_temp;

	// Update fusion bias while preserving orientation
	LOG_INF("Runtime Cal: Completed at %.2fC, D_offset updated", (double)avg_temp);
	sensor_fusion_update_bias(NULL);

	return 0;
}

/**
 * Check if runtime calibration should be triggered
 * Called from sensor loop when device is at rest
 *
 * @param is_resting true if device is currently at rest
 */
void sensor_runtime_calibration_check(bool is_resting)
{
	// Skip if runtime calibration is disabled
	if (!runtime_cal_enabled) {
		return;
	}

	// Skip if boot calibration hasn't completed yet
	if (!retained->bootCalState.completed) {
		return;
	}

	int64_t now = k_uptime_get();

	// Enforce minimum uptime before runtime calibration
	if (now < RUNTIME_CAL_MIN_UPTIME_MS) {
		return;
	}

	// Check cooldown period
	if (runtime_cal_last_time != 0 && (now - runtime_cal_last_time) < RUNTIME_CAL_COOLDOWN_MS) {
		return;
	}

	// Check if another calibration is running
	if (sensor_calibration_request(0) != 0) {
		runtime_cal_rest_tracking = false;
		runtime_cal_rest_start = 0;
		return;
	}

	// Get current temperature for comparison
	float current_temp = sensor_get_current_imu_temperature();

	if (is_resting) {
		// Start or continue tracking rest period
		if (!runtime_cal_rest_tracking) {
			runtime_cal_rest_tracking = true;
			runtime_cal_rest_start = now;
			LOG_DBG("Runtime Cal: Started tracking rest period");
		} else {
			// Check if we've been resting long enough
			int64_t rest_duration = now - runtime_cal_rest_start;
			if (rest_duration >= RUNTIME_CAL_REST_TIME_MS) {
				// Check temperature change since last calibration
				// Skip if temperature hasn't changed enough
				if (!isnan(runtime_cal_last_temp) && !isnan(current_temp)) {
					float temp_change = fabsf(current_temp - runtime_cal_last_temp);
					if (temp_change < RUNTIME_CAL_TEMP_CHANGE_MIN) {
						LOG_DBG(
							"Runtime Cal: Skipping - temp change %.2fC < %.2fC threshold",
							(double)temp_change,
							(double)RUNTIME_CAL_TEMP_CHANGE_MIN
						);
						// Reset tracking but don't request calibration
						runtime_cal_rest_tracking = false;
						runtime_cal_rest_start = 0;
						return;
					}
				}

				LOG_INF(
					"Runtime Cal: Device at rest for %lld ms, temp %.2fC (last: %.2fC), requesting calibration",
					rest_duration,
					(double)current_temp,
					isnan(runtime_cal_last_temp) ? 0.0 : (double)runtime_cal_last_temp
				);

				// Request runtime calibration (uses calibration request ID 4)
				int request_result = sensor_calibration_request(4);
				if (request_result == 0) {
					LOG_INF("Runtime Cal: Calibration requested");
					runtime_cal_rest_tracking = false;
					runtime_cal_rest_start = 0;
				}
			}
		}
	} else {
		// Device moved, reset rest tracking
		if (runtime_cal_rest_tracking) {
			LOG_DBG("Runtime Cal: Rest tracking reset due to motion");
			runtime_cal_rest_tracking = false;
			runtime_cal_rest_start = 0;
		}
	}
}

/**
 * Get runtime calibration status information
 */
void sensor_runtime_cal_get_status(int64_t *last_cal_time, int64_t *rest_duration)
{
	if (last_cal_time) {
		*last_cal_time = runtime_cal_last_time;
	}
	if (rest_duration) {
		if (runtime_cal_rest_tracking) {
			*rest_duration = k_uptime_get() - runtime_cal_rest_start;
		} else {
			*rest_duration = 0;
		}
	}
}

// =============================================================================
// T-Cal Test/Debug Functions
// =============================================================================

/**
 * Test and compare different calibration methods at a given temperature
 * Useful for debugging and understanding method differences
 */
void sensor_tcal_test_methods(float temp)
{
	if (retained->tempCalState.count < 1) {
		printk("No calibration data available.\n");
		return;
	}

	printk("\n=== T-Cal Method Comparison at %.2fC ===\n", (double)temp);
	printk("Total calibration points: %u\n\n", retained->tempCalState.count);

	// Show available calibration points
	float min_temp = INFINITY, max_temp = -INFINITY;
	int point_count = 0;
	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (retained->tempCalPoints[i].temp != 0.0f) {
			float t = retained->tempCalPoints[i].temp;
			if (t < min_temp) {
				min_temp = t;
			}
			if (t > max_temp) {
				max_temp = t;
			}
			point_count++;
		}
	}
	printk("Calibrated range: %.2fC to %.2fC\n", (double)min_temp, (double)max_temp);

	// Show temperature position
	if (temp < min_temp) {
		printk("Test temp is %.2fC BELOW calibrated range\n", (double)(min_temp - temp));
	} else if (temp > max_temp) {
		printk("Test temp is %.2fC ABOVE calibrated range\n", (double)(temp - max_temp));
	} else {
		printk("Test temp is WITHIN calibrated range\n");
	}
	printk("\n");

	// Method 1: MLS (Moving Least Squares - primary method)
	if (retained->tempCalState.count >= MLS_MIN_POINTS_FOR_FIT) {
		float mls_bias[3];
		int result = sensor_tcal_mls_lookup(temp, mls_bias);

		if (result == 0) {
			printk("MLS Method (bandwidth=%.1fC):\n", (double)MLS_BANDWIDTH);
			printk("  Bias: [%.5f, %.5f, %.5f] dps\n", (double)mls_bias[0], (double)mls_bias[1], (double)mls_bias[2]);
		} else {
			printk("MLS Method: FAILED\n");
		}
		printk("\n");
	} else {
		printk("MLS Method: Not enough points (need >= %d)\n\n", MLS_MIN_POINTS_FOR_FIT);
	}

	// Show boot cal D_offset if active
	if (retained->bootCalState.doffset_valid) {
		printk("Additional Offsets:\n");
		printk(
			"  Boot cal D_offset: [%.5f, %.5f, %.5f] dps\n",
			(double)retained->bootCalState.doffset[0],
			(double)retained->bootCalState.doffset[1],
			(double)retained->bootCalState.doffset[2]
		);
		printk("\n");
	}

	// Show final effective bias that would be applied
	printk("Final Effective Bias (as applied to gyro data):\n");

	// Calculate what would actually be used using unified strategy: MLS -> Static
	float final_bias[3] = {0.0f, 0.0f, 0.0f};
	bool calculated = false;
	const char *method_used = "static";

	if (sensor_tcal_mls_lookup(temp, final_bias) == 0) {
		calculated = true;
		method_used = "MLS";
	}

	if (calculated) {
		// tempCalCorrectionOffset is retained for compatibility only; no longer used.

		// Add boot cal D_offset if valid
		if (retained->bootCalState.doffset_valid) {
			for (int i = 0; i < 3; i++) {
				final_bias[i] += retained->bootCalState.doffset[i];
			}
		}

		printk(
			"  Total: [%.5f, %.5f, %.5f] dps\n",
			(double)final_bias[0],
			(double)final_bias[1],
			(double)final_bias[2]
		);
		printk("  Method: %s (Unified Strategy: MLS -> Static)\n", method_used);
	} else {
		printk(
			"  Fallback to static bias: [%.5f, %.5f, %.5f] dps\n",
			(double)retained->gyroBias[0],
			(double)retained->gyroBias[1],
			(double)retained->gyroBias[2]
		);
		printk("  (No valid T-Cal method available)\n");
	}

	printk("\n=== End of T-Cal Method Comparison ===\n");
}

/**
 * Moving Least Squares (MLS) lookup function implementation
 *
 * This function provides a unified, smooth bias estimation that:
 * - Eliminates discontinuities from method switching
 * - Naturally handles boundaries and extrapolation
 * - Uses local linear fitting with distance-based weighting
 *
 * Weight function: w(d) = 1 / (1 + (d/bandwidth)²)
 * This is a Cauchy-like weight that provides smooth falloff without exp()
 *
 * @param temp Query temperature
 * @param bias_out Output: computed 3-axis bias
 * @return 0 on success, -1 if insufficient data
 */
static int sensor_tcal_mls_lookup(float temp, float bias_out[3])
{
	// Check if we have any calibration data
	if (retained->tempCalState.count < 1) {
		LOG_ERR("T-Cal MLS: No calibration data available");
		return -1;
	}

	// Check multi-slot cache:
	// First, check if point count changed (invalidates all cache slots)
	if (mls_cache.count != retained->tempCalState.count) {
		// Invalidate all slots
		for (int i = 0; i < MLS_CACHE_SLOTS; i++) {
			mls_cache.slots[i].valid = false;
		}
		mls_cache.count = retained->tempCalState.count;
	}

	// Search for a matching cache slot
	for (int i = 0; i < MLS_CACHE_SLOTS; i++) {
		if (mls_cache.slots[i].valid &&
		    fabsf(mls_cache.slots[i].temp - temp) < MLS_CACHE_TEMP_THRESHOLD) {
			// Cache hit - return smoothly interpolated value using cached local slope.
			// This prevents the output from becoming piecewise-constant within the
			// cache threshold window (e.g. 36.10C vs 36.20C).
			float dt = temp - mls_cache.slots[i].temp;
			for (int axis = 0; axis < 3; axis++) {
				bias_out[axis] = mls_cache.slots[i].bias[axis] + mls_cache.slots[i].slope[axis] * dt;
			}
			return 0;
		}
	}

	// If only one point, just return it (no fitting possible)
	if (retained->tempCalState.count == 1) {
		// Find the single point
		for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
			if (retained->tempCalPoints[i].temp != 0.0f) {
				memcpy(bias_out, retained->tempCalPoints[i].bias, sizeof(float) * 3);
				// Update cache (select best slot based on distance)
				int slot = sensor_tcal_cache_select_slot(temp);
				mls_cache.slots[slot].temp = temp;
				memcpy(mls_cache.slots[slot].bias, bias_out, sizeof(float) * 3);
				memset(mls_cache.slots[slot].slope, 0, sizeof(mls_cache.slots[slot].slope));
				mls_cache.slots[slot].valid = true;
				LOG_DBG("T-Cal MLS: Single point at %.2fC (cached in slot %d)", (double)retained->tempCalPoints[i].temp, slot);
				return 0;
			}
		}
		return -1; // Should not reach here
	}

	// Collect points and compute weights
	// Using simple structure to hold point data with weights
	typedef struct {
		float temp;
		float bias[3];
		float weight;
	} WeightedPoint;

	// First pass: collect all points with sufficient weight
	// With MLS_MIN_WEIGHT=0.05, points beyond ~6.5°C are filtered out early
	WeightedPoint all_points[MLS_MAX_POINTS * 2];  // Temporary buffer
	int all_count = 0;

	float bandwidth_sq = MLS_BANDWIDTH * MLS_BANDWIDTH;

	for (int i = 0; i < TCAL_BUFFER_SIZE && all_count < MLS_MAX_POINTS * 2; i++) {
		if (retained->tempCalPoints[i].temp == 0.0f) {
			continue; // Skip empty slots
		}

		float point_temp = retained->tempCalPoints[i].temp;
		float d = point_temp - temp;
		float d_sq = d * d;

		// Cauchy-like weight: w = 1 / (1 + (d/h)²)
		// This provides smooth falloff and is computationally efficient
		float weight = 1.0f / (1.0f + d_sq / bandwidth_sq);

		// Skip points with negligible weight
		if (weight < MLS_MIN_WEIGHT) {
			continue;
		}

		all_points[all_count].temp = point_temp;
		all_points[all_count].weight = weight;
		memcpy(all_points[all_count].bias, retained->tempCalPoints[i].bias, sizeof(float) * 3);
		all_count++;
	}

	// Second pass: if we have more than MLS_MAX_POINTS, select top-k by weight
	// Uses partial selection sort - O(k*n) which is efficient for small k
	WeightedPoint points[MLS_MAX_POINTS];
	int point_count = (all_count <= MLS_MAX_POINTS) ? all_count : MLS_MAX_POINTS;

	if (all_count <= MLS_MAX_POINTS) {
		// Just copy all points
		memcpy(points, all_points, all_count * sizeof(WeightedPoint));
	} else {
		// Partial selection: pick top MLS_MAX_POINTS by weight
		for (int k = 0; k < MLS_MAX_POINTS; k++) {
			int max_idx = k;
			float max_weight = all_points[k].weight;
			for (int j = k + 1; j < all_count; j++) {
				if (all_points[j].weight > max_weight) {
					max_weight = all_points[j].weight;
					max_idx = j;
				}
			}
			// Swap to position k
			if (max_idx != k) {
				WeightedPoint tmp = all_points[k];
				all_points[k] = all_points[max_idx];
				all_points[max_idx] = tmp;
			}
			points[k] = all_points[k];
		}
	}

	if (point_count == 0) {
		LOG_ERR("T-Cal MLS: No points with sufficient weight");
		return -1;
	}

	// If only one point has significant weight, just return it
	if (point_count == 1) {
		memcpy(bias_out, points[0].bias, sizeof(float) * 3);

		// Cache as a locally constant model (slope = 0)
		int slot = sensor_tcal_cache_select_slot(temp);
		mls_cache.slots[slot].temp = temp;
		memcpy(mls_cache.slots[slot].bias, bias_out, sizeof(float) * 3);
		memset(mls_cache.slots[slot].slope, 0, sizeof(mls_cache.slots[slot].slope));
		mls_cache.slots[slot].valid = true;

		LOG_DBG(
			"T-Cal MLS: Single significant point at %.2fC (w=%.3f)",
			(double)points[0].temp,
			(double)points[0].weight
		);
		return 0;
	}

	// Perform weighted linear least squares fit for each axis
	// We solve: minimize Σ w_i * (b_i - (a + c*(t_i - t_query)))²
	//
	// Normal equations:
	//   Σ w_i * b_i = a * Σ w_i + c * Σ w_i * (t_i - t_q)
	//   Σ w_i * (t_i - t_q) * b_i = a * Σ w_i * (t_i - t_q) + c * Σ w_i * (t_i - t_q)²
	//
	// Since t_q is our query point, let delta_t = t_i - t_q, then:
	//   sum_w = Σ w_i
	//   sum_wdt = Σ w_i * delta_t_i  (often ~0 if query is centered)
	//   sum_wdt2 = Σ w_i * delta_t_i²
	//   sum_wb[axis] = Σ w_i * b_i[axis]
	//   sum_wdtb[axis] = Σ w_i * delta_t_i * b_i[axis]
	//
	// Solution (Cramer's rule):
	//   det = sum_w * sum_wdt2 - sum_wdt * sum_wdt
	//   a = (sum_wb * sum_wdt2 - sum_wdtb * sum_wdt) / det
	//   c = (sum_w * sum_wdtb - sum_wdt * sum_wb) / det
	//
	// Result at query point (delta_t = 0): bias = a

	double sum_w = 0.0;
	double sum_wdt = 0.0;
	double sum_wdt2 = 0.0;
	double sum_wb[3] = {0.0, 0.0, 0.0};
	double sum_wdtb[3] = {0.0, 0.0, 0.0};

	for (int i = 0; i < point_count; i++) {
		double w = (double)points[i].weight;
		double dt = (double)(points[i].temp - temp);

		sum_w += w;
		sum_wdt += w * dt;
		sum_wdt2 += w * dt * dt;

		for (int axis = 0; axis < 3; axis++) {
			double b = (double)points[i].bias[axis];
			sum_wb[axis] += w * b;
			sum_wdtb[axis] += w * dt * b;
		}
	}

	// Calculate determinant
	double det = sum_w * sum_wdt2 - sum_wdt * sum_wdt;

	// Check for degenerate case (all points at same temperature)
	if (fabs(det) < 1e-10) {
		// Fallback to weighted average
		for (int axis = 0; axis < 3; axis++) {
			bias_out[axis] = (float)(sum_wb[axis] / sum_w);
		}
		// Update cache (select best slot based on distance)
		int slot = sensor_tcal_cache_select_slot(temp);
		mls_cache.slots[slot].temp = temp;
		memcpy(mls_cache.slots[slot].bias, bias_out, sizeof(float) * 3);
		memset(mls_cache.slots[slot].slope, 0, sizeof(mls_cache.slots[slot].slope));
		mls_cache.slots[slot].valid = true;
		LOG_DBG("T-Cal MLS: Degenerate case, using weighted average at %.2fC (cached in slot %d)", (double)temp, slot);
		return 0;
	}

	// Solve for 'a' (the bias at query temperature) and local slope 'c'
	// a = (sum_wb * sum_wdt2 - sum_wdtb * sum_wdt) / det
	// c = (sum_w * sum_wdtb - sum_wdt * sum_wb) / det
	float slope_out[3];
	for (int axis = 0; axis < 3; axis++) {
		double a = (sum_wb[axis] * sum_wdt2 - sum_wdtb[axis] * sum_wdt) / det;
		double c = (sum_w * sum_wdtb[axis] - sum_wdt * sum_wb[axis]) / det;
		bias_out[axis] = (float)a;
		slope_out[axis] = (float)c;
	}

	// Update cache with computed result (select best slot based on distance)
	int slot = sensor_tcal_cache_select_slot(temp);
	mls_cache.slots[slot].temp = temp;
	memcpy(mls_cache.slots[slot].bias, bias_out, sizeof(float) * 3);
	memcpy(mls_cache.slots[slot].slope, slope_out, sizeof(float) * 3);
	mls_cache.slots[slot].valid = true;

	LOG_DBG(
		"T-Cal MLS: Computed bias [%.4f, %.4f, %.4f] at %.2fC using %d points (cached in slot %d)",
		(double)bias_out[0],
		(double)bias_out[1],
		(double)bias_out[2],
		(double)temp,
		point_count,
		slot
	);

	return 0;
}

// =============================================================================
// LUT Incremental Build Functions
// =============================================================================

/**
 * Helper function to compute and store a single LUT entry
 * @param idx LUT index to compute
 * @return true if successfully computed, false on error
 */
static bool sensor_tcal_lut_compute_entry(int idx)
{
	if (idx < 0 || idx >= MLS_LUT_SIZE) {
		return false;
	}

	// Skip if already computed
	if (mls_lut.entries[idx].computed) {
		return true;
	}

	float temp = MLS_LUT_IDX_TO_TEMP(idx);
	float bias[3];

	if (sensor_tcal_mls_lookup(temp, bias) == 0) {
		memcpy(mls_lut.entries[idx].bias, bias, sizeof(float) * 3);
		mls_lut.entries[idx].computed = true;
		mls_lut.computed_count++;
		return true;
	}

	return false;
}

/**
 * Build priority zone of LUT (±3°C around current temperature)
 * This is called at startup and when calibration points change.
 * Returns quickly after building the priority zone.
 * Background build continues via sensor_tcal_build_lut_continue().
 *
 * @param current_temp Current device temperature
 */
static void sensor_tcal_build_lut_priority(float current_temp)
{
	// Check if we have enough points for MLS
	if (retained->tempCalState.count < MLS_MIN_POINTS_FOR_FIT) {
		LOG_INF("T-Cal LUT: Not enough points (%u < %d), LUT disabled",
		        retained->tempCalState.count, MLS_MIN_POINTS_FOR_FIT);
		mls_lut.valid = false;
		mls_lut.build_state = MLS_LUT_BUILD_IDLE;
		return;
	}

	int64_t start_time = k_uptime_get();

	// Reset LUT state for fresh build
	mls_lut.valid = false;
	mls_lut.version = retained->tempCalState.count;
	mls_lut.computed_count = 0;

	// Mark all entries as not computed
	for (int i = 0; i < MLS_LUT_SIZE; i++) {
		mls_lut.entries[i].computed = false;
	}

	// Clear legacy cache to force fresh MLS computation
	for (int i = 0; i < MLS_CACHE_SLOTS; i++) {
		mls_cache.slots[i].valid = false;
	}
	mls_cache.count = 0;

	// Calculate priority zone indices (±3°C around current temp)
	float priority_temp_min = current_temp - MLS_LUT_PRIORITY_RANGE;
	float priority_temp_max = current_temp + MLS_LUT_PRIORITY_RANGE;

	// Clamp to LUT range
	if (priority_temp_min < MLS_LUT_TEMP_MIN) {
		priority_temp_min = MLS_LUT_TEMP_MIN;
	}
	if (priority_temp_max > MLS_LUT_TEMP_MAX) {
		priority_temp_max = MLS_LUT_TEMP_MAX;
	}

	mls_lut.priority_idx_min = (int)MLS_LUT_TEMP_TO_IDX(priority_temp_min);
	mls_lut.priority_idx_max = (int)MLS_LUT_TEMP_TO_IDX(priority_temp_max) + 1;

	// Clamp indices
	if (mls_lut.priority_idx_min < 0) {
		mls_lut.priority_idx_min = 0;
	}
	if (mls_lut.priority_idx_max >= MLS_LUT_SIZE) {
		mls_lut.priority_idx_max = MLS_LUT_SIZE - 1;
	}

	LOG_INF("T-Cal LUT: Building priority zone [%d-%d] (%.1f°C to %.1f°C)",
	        mls_lut.priority_idx_min, mls_lut.priority_idx_max,
	        (double)priority_temp_min, (double)priority_temp_max);

	mls_lut.build_state = MLS_LUT_BUILD_PRIORITY;

	// Build priority zone entries
	int priority_count = 0;
	for (int idx = mls_lut.priority_idx_min; idx <= mls_lut.priority_idx_max; idx++) {
		if (sensor_tcal_lut_compute_entry(idx)) {
			priority_count++;
		}

		// Feed watchdog periodically
		if (priority_count % 20 == 0) {
			watchdog_feed(WDT_CHANNEL_CALIBRATION);
		}
	}

	// Mark LUT as valid once priority zone is complete
	mls_lut.valid = true;

	// Set up for background build of remaining entries
	mls_lut.build_next_idx = 0;
	mls_lut.build_state = MLS_LUT_BUILD_BACKGROUND;

	int64_t elapsed = k_uptime_get() - start_time;
	LOG_INF("T-Cal LUT: Priority zone built (%d entries) in %lld ms, background build started",
	        priority_count, elapsed);
}

/**
 * Continue building LUT entries in background.
 * Called from calibration_thread main loop.
 * Processes a small batch of entries per call to avoid blocking.
 *
 * @return true if build is complete, false if more work remains
 */
static bool sensor_tcal_build_lut_continue(void)
{
	// Check if build is needed
	if (mls_lut.build_state != MLS_LUT_BUILD_BACKGROUND) {
		return true;  // Not in background build state
	}

	// Check version match
	if (mls_lut.version != retained->tempCalState.count) {
		// Points changed, invalidate and stop
		mls_lut.build_state = MLS_LUT_BUILD_IDLE;
		mls_lut.valid = false;
		return true;
	}

	// Process a batch of entries
	int computed_this_batch = 0;
	while (computed_this_batch < MLS_LUT_BATCH_SIZE && mls_lut.build_next_idx < MLS_LUT_SIZE) {
		int idx = mls_lut.build_next_idx;
		mls_lut.build_next_idx++;

		// Skip already computed entries (priority zone)
		if (mls_lut.entries[idx].computed) {
			continue;
		}

		sensor_tcal_lut_compute_entry(idx);
		computed_this_batch++;
	}

	// Check if complete
	if (mls_lut.build_next_idx >= MLS_LUT_SIZE) {
		mls_lut.build_state = MLS_LUT_BUILD_COMPLETE;
		return true;
	}

	// Yield CPU time
	k_msleep(MLS_LUT_BATCH_YIELD_MS);
	return false;
}

// =============================================================================
// LUT Lookup Function - O(1) Linear Interpolation
// =============================================================================
/**
 * Fast O(1) lookup using pre-computed LUT with linear interpolation.
 * For entries not yet computed (during incremental build), falls back to -1.
 *
 * @param temp Query temperature
 * @param bias_out Output: interpolated 3-axis bias
 * @return 0 on success, -1 if required entries not computed
 */
static int sensor_tcal_lut_lookup(float temp, float bias_out[3])
{
	// Check LUT validity and version
	if (!mls_lut.valid || mls_lut.version != retained->tempCalState.count) {
		return -1;  // LUT not available
	}

	// Track if we need to extrapolate (temp outside LUT range)
	bool extrapolate_low = (temp < MLS_LUT_TEMP_MIN);
	bool extrapolate_high = (temp > MLS_LUT_TEMP_MAX);
	float original_temp = temp;

	// For extrapolation, use 4 points with 1°C spacing for robust slope estimation
	// With MLS_LUT_STEP_PER_DEGREE = 2, 1°C = 2 LUT indices
	#define EXTRAP_POINT_SPACING MLS_LUT_STEP_PER_DEGREE  // 1°C spacing
	#define EXTRAP_NUM_POINTS 4

	if (extrapolate_low) {
		// Use first 4 points at 1°C intervals: indices 0, 2, 4, 6
		int indices[EXTRAP_NUM_POINTS];
		for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
			indices[i] = i * EXTRAP_POINT_SPACING;
			if (indices[i] >= MLS_LUT_SIZE) {
				return -1;  // Not enough range for extrapolation
			}
			if (!mls_lut.entries[indices[i]].computed) {
				return -1;  // Required points not computed
			}
		}

		// Linear least squares fit using 4 points
		// slope = Σ(t_i - t_mean)(b_i - b_mean) / Σ(t_i - t_mean)²
		float temps[EXTRAP_NUM_POINTS];
		float t_mean = 0.0f;
		for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
			temps[i] = MLS_LUT_IDX_TO_TEMP(indices[i]);
			t_mean += temps[i];
		}
		t_mean /= EXTRAP_NUM_POINTS;

		// Pre-compute Σ(t_i - t_mean)² (same for all axes)
		float sum_dt_sq = 0.0f;
		for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
			float dt = temps[i] - t_mean;
			sum_dt_sq += dt * dt;
		}

		for (int axis = 0; axis < 3; axis++) {
			float b_mean = 0.0f;
			for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
				b_mean += mls_lut.entries[indices[i]].bias[axis];
			}
			b_mean /= EXTRAP_NUM_POINTS;

			float sum_dt_db = 0.0f;
			for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
				float dt = temps[i] - t_mean;
				float db = mls_lut.entries[indices[i]].bias[axis] - b_mean;
				sum_dt_db += dt * db;
			}

			float slope = (sum_dt_sq > 0.001f) ? sum_dt_db / sum_dt_sq : 0.0f;
			bias_out[axis] = b_mean + slope * (original_temp - t_mean);
		}
		return 0;

	} else if (extrapolate_high) {
		// Use last 4 points at 1°C intervals: indices n-1, n-3, n-5, n-7
		int indices[EXTRAP_NUM_POINTS];
		for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
			indices[i] = (MLS_LUT_SIZE - 1) - i * EXTRAP_POINT_SPACING;
			if (indices[i] < 0) {
				return -1;  // Not enough range for extrapolation
			}
			if (!mls_lut.entries[indices[i]].computed) {
				return -1;  // Required points not computed
			}
		}

		// Linear least squares fit using 4 points
		float temps[EXTRAP_NUM_POINTS];
		float t_mean = 0.0f;
		for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
			temps[i] = MLS_LUT_IDX_TO_TEMP(indices[i]);
			t_mean += temps[i];
		}
		t_mean /= EXTRAP_NUM_POINTS;

		float sum_dt_sq = 0.0f;
		for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
			float dt = temps[i] - t_mean;
			sum_dt_sq += dt * dt;
		}

		for (int axis = 0; axis < 3; axis++) {
			float b_mean = 0.0f;
			for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
				b_mean += mls_lut.entries[indices[i]].bias[axis];
			}
			b_mean /= EXTRAP_NUM_POINTS;

			float sum_dt_db = 0.0f;
			for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
				float dt = temps[i] - t_mean;
				float db = mls_lut.entries[indices[i]].bias[axis] - b_mean;
				sum_dt_db += dt * db;
			}

			float slope = (sum_dt_sq > 0.001f) ? sum_dt_db / sum_dt_sq : 0.0f;
			bias_out[axis] = b_mean + slope * (original_temp - t_mean);
		}
		return 0;
	}

	// Normal interpolation within range
	float fidx = MLS_LUT_TEMP_TO_IDX(temp);

	// Get integer indices for interpolation
	int idx_lo = (int)fidx;
	int idx_hi = idx_lo + 1;

	// Clamp indices to valid range (handle edge cases)
	if (idx_lo < 0) {
		idx_lo = 0;
		idx_hi = 1;
	}
	if (idx_hi >= MLS_LUT_SIZE) {
		idx_hi = MLS_LUT_SIZE - 1;
		idx_lo = MLS_LUT_SIZE - 2;
	}
	if (idx_lo < 0) {
		idx_lo = 0;
	}

	// Check if required entries are computed
	if (!mls_lut.entries[idx_lo].computed || !mls_lut.entries[idx_hi].computed) {
		return -1;  // Required entries not yet computed, caller should use MLS fallback
	}

	// Linear interpolation
	float frac = fidx - (float)idx_lo;
	if (frac < 0.0f) {
		frac = 0.0f;
	} else if (frac > 1.0f) {
		frac = 1.0f;
	}

	const float *bias_lo = mls_lut.entries[idx_lo].bias;
	const float *bias_hi = mls_lut.entries[idx_hi].bias;

#if CONFIG_CMSIS_DSP
	// bias_out = bias_lo + frac * (bias_hi - bias_lo)
	float diff[3];
	arm_sub_f32(bias_hi, bias_lo, diff, 3);
	arm_scale_f32(diff, frac, diff, 3);
	arm_add_f32(bias_lo, diff, bias_out, 3);
#else
	for (int axis = 0; axis < 3; axis++) {
		bias_out[axis] = bias_lo[axis] + frac * (bias_hi[axis] - bias_lo[axis]);
	}
#endif

	return 0;
}

#endif
