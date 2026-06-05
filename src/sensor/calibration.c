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
#if CONFIG_SENSOR_USE_VQF
#include "fusion/vqf/vqf.h"
#endif

#include "calibration.h"

static uint8_t imu_id;
static uint8_t sensor_data[128]; // any use sensor data

static float accelBias[3] = {0}, gyroBias[3] = {0}, magBias[3] = {0}; // offset biases

static float accBAinv[4][3];
static float magBAinv[4][3];

static uint8_t magneto_progress;
static uint8_t last_magneto_progress;
static int64_t magneto_progress_time;

K_MUTEX_DEFINE(calibration_request_lock);
static int requested_calibration;

#if CONFIG_SENSOR_USE_SENS_CALIBRATION
// Parameters for the requested gyro sensitivity calibration, latched by
// sensor_request_calibration_sens() before the calibration thread runs.
static uint8_t sens_cal_axis;
static uint16_t sens_cal_revolutions;
#endif

// Only trust orientation updates when accel magnitude stays reasonably close to 1 g.
// This rejects samples collected during strong linear acceleration while keeping
// normal hand-rotation usable for calibration.
#define MAG_CAL_ACCEL_MAG_MIN_SQ 0.75f
#define MAG_CAL_ACCEL_MAG_MAX_SQ 1.3f
#define CALIBRATION_SENSOR_INIT_WAIT_MS 10000
#define CALIBRATION_SENSOR_INIT_POLL_MS 10

// Minimum samples before attempting trial calibration
#define MAG_CAL_MIN_SAMPLES 64
// Attempt trial calibration every this many new samples (manual cal)
#define MAG_CAL_TRIAL_INTERVAL 80

static int64_t mag_cal_last_status_log;

static double ata[100]; // manual calibration accumulator
static double norm_sum;
static double sample_count;
// Direction range tracking for manual calibration: per-axis min/max of normalized direction
static float dir_min[3];
static float dir_max[3];
// Per-axis min/max center estimator used only for coverage decisions.
// Magneto still receives raw samples so the hard/soft-iron fit is unchanged.
typedef struct {
	float min[3];
	float max[3];
	bool initialized;
} mag_center_estimator_t;

static mag_center_estimator_t manual_center_estimator;
static mag_center_estimator_t online_center_estimator;
// Minimum direction range per axis for accepting manual calibration
// 0.5 ≈ 30° arc on each axis; requires meaningful rotation around at least 2 axes
#define MAG_CAL_MIN_DIR_RANGE 0.5f
// Do not use a provisional min/max center until the raw point cloud spans a
// meaningful range on every axis. Without this, tiny local motion around a
// biased field can look like full centered coverage and produce a huge-gain fit.
#define MAG_CAL_MIN_RAW_AXIS_RANGE 0.5f
// Reject degenerate Magneto fits that turn a tiny point cloud into a sphere by
// applying a very large soft-iron gain. Normal calibrated gains are around 1-3.
#define MAG_CAL_MAX_AXIS_GAIN 8.0f

// Per-quadrant ring buffer for online magnetometer calibration.
// Combines the directional coverage guarantee of quadrant-based sampling
// (8 octants based on sign of x, y, z) with the natural aging of FIFO
// per-quadrant sliding windows. Each octant independently wraps after
// QUADRANT_BUF_SIZE samples — staying in one orientation only updates
// that octant, leaving the other 7 with diverse data.
// 8 × 16 = 128 samples total (matching magcal's proven size),
// ~1.5KB vs 3.3KB for the old 4×80 segment design.
#define QUADRANT_BUF_SIZE 16
#define ONLINE_QUADRANT_COUNT 8

typedef struct {
	float x, y, z;
} quadrant_sample_t;

typedef struct {
	quadrant_sample_t samples[QUADRANT_BUF_SIZE];
	uint8_t head;   // next write position
	uint8_t count;  // valid samples (0..QUADRANT_BUF_SIZE)
	int64_t last_seq; // global accepted-sample sequence of the newest sample in this octant
} quadrant_buf_t;

// Incremental calibration blending (EMA on BAinv elements)
// Base alpha: weight given to new trial calibration. A value of 0.35 means
// 35% new + 65% existing → gradual convergence over ~3 updates.
// Higher when trial diverges significantly from existing (environment change).
#define ONLINE_BLEND_BASE_ALPHA 0.35f
#define ONLINE_BLEND_MIN_ALPHA 0.12f   // floor: very similar calibrations
#define ONLINE_BLEND_MAX_ALPHA 0.70f   // ceiling: significant divergence detected
#define ONLINE_BLEND_SIMILARITY_LOW 0.85f   // below this similarity, increase alpha
#define ONLINE_BLEND_SIMILARITY_HIGH 0.97f  // above this similarity, use min alpha

static quadrant_buf_t quad_buf[ONLINE_QUADRANT_COUNT];
static int64_t online_total_sample_count;
static int64_t online_last_checked_sample_count;
static int64_t online_last_check_time;
static int64_t online_last_sample_time; // rate limiting
// Drop octants that have not been refreshed for too long.
// This is kept separate from the check cadence: stale-history rejection should
// not depend on how often the background thread decides to run Magneto.
#define ONLINE_STALE_QUADRANT_MAX_AGE 320
// Minimum direction change to accept an online sample. The configured value is
// expressed in degrees and converted to the equivalent 1 - cos(theta) threshold.
static float online_last_dir[3];
static float online_last_accel_dir[3]; // accel direction for cross-validation

// Manual calibration direction tracking (same cross-validation logic as online path)
static float manual_last_dir[3];
static float manual_last_accel_dir[3];

#define ONLINE_MIN_DIR_CHANGE_DEG 10.0f
#define ONLINE_MIN_INTERVAL_MS 30  // minimum 30ms between online samples
// Background checks should not run on every calibration-thread pass.
// Tie the minimum check spacing to roughly one fresh fit's worth of accepted
// samples at the maximum online sampling rate.
#define ONLINE_MIN_CHECK_INTERVAL_MS (MAG_CAL_MIN_SAMPLES * ONLINE_MIN_INTERVAL_MS * 2)

// Runtime calibrated norm tracking (exponential moving average)
// Used to assess current calibration quality and decide if online update is needed
static float cal_norm_ema;        // EMA of calibrated mag norm
static float cal_norm_var_ema;    // EMA of squared deviation from mean
static uint32_t cal_norm_count;   // number of norm samples processed
#define CAL_NORM_EMA_ALPHA 0.01f  // smoothing factor (~100 sample window)
// Don't update calibration if current norm CV is below this threshold
#define CAL_NORM_GOOD_CV 0.05f    // 5% = good enough calibration

// Minimum time between online calibration updates (prevents frequent VQF mag ref resets)
#define ONLINE_MIN_UPDATE_INTERVAL_S 6  // 6 seconds cooldown
static int64_t online_last_update_time;

// Suppress online sample collection for N ms after buffer resets (wake-up,
// reboot, environment change, calibration update).  This lets sensor data
// stabilise before collecting, avoiding transient/mixed-environment samples
// that produce poor calibration fits.
#define ONLINE_COLLECTION_SUPPRESS_MS 1500
static int64_t online_collection_suppress_until;

// Minimum sustained VQF magnetic disturbance duration before allowing an online
// calibration update.  Short disturbance bursts are usually transient interference;
// updating calibration during those resets VQF's heading reference for no benefit.
#define ONLINE_VQF_DIST_MIN_DURATION_MS 3000
static int64_t online_vqf_dist_start_time;

// Require at least N successful calibration updates before trusting the
// norm CV gate. Prevents a single early fit from being declared "good enough"
// when the buffer is still filling and directional coverage is incomplete.
#define ONLINE_MIN_UPDATES 3
static int online_update_count;

// Norm-change detection: when the average raw field strength in the buffer
// changes by >40% between updates, the magnetic environment has changed.
// We clear buffers to avoid mixed-data fits and let the next cycle fit
// on consistent data from the new environment.
static float online_last_buf_avg_norm = 0.0f;

static void magneto_online_runtime_reset(void);
static void magneto_online_runtime_load_retained(void);

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
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
static void sensor_calibrate_sens(void);
#endif

// =============================================================================
// Offset-bias collection constants
// =============================================================================
#ifndef BIAS_COLLECT_TEMP_RANGE_THRESHOLD
#define BIAS_COLLECT_TEMP_RANGE_THRESHOLD 1.0f // °C - stop early if temp changes this much
#endif

#ifndef BIAS_COLLECT_MAX_SAMPLE_TIME_MS
#define BIAS_COLLECT_MAX_SAMPLE_TIME_MS 5000 // 5 seconds max
#endif

#ifndef BIAS_COLLECT_MIN_SAMPLE_TIME_MS
#define BIAS_COLLECT_MIN_SAMPLE_TIME_MS 3000 // 3 seconds min
#endif

#ifndef BIAS_COLLECT_TEMP_CHECK_TIME_MS
#define BIAS_COLLECT_TEMP_CHECK_TIME_MS 3000 // 3 seconds - prioritize sampling time over temp stability
#endif

#ifndef BIAS_COLLECT_GYRO_MOTION_THRESHOLD
#define BIAS_COLLECT_GYRO_MOTION_THRESHOLD 1.5f // dps (range method)
#endif

#ifndef BIAS_COLLECT_ACCEL_MOTION_THRESHOLD
#define BIAS_COLLECT_ACCEL_MOTION_THRESHOLD 0.06f // G (range method)
#endif

#if CONFIG_SENSOR_USE_TCAL

#define TEMP_TO_IDX(temp) (int)((((float)temp) - CONFIG_SENSOR_POLY_TEMP_MIN) * CONFIG_SENSOR_POLY_STEPS_PER_DEGREE)
#define IDX_TO_TEMP(idx) (float)(((float)(idx) / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE) + CONFIG_SENSOR_POLY_TEMP_MIN)

// =============================================================================
// Boot Calibration - Runtime D_offset Calculation
// =============================================================================

// Boot calibration constants
#define BOOT_CAL_TIME_WINDOW_START_MS 5000 // 5 seconds after boot
#define BOOT_CAL_TIME_WINDOW_END_MS 30000  // 30 seconds after boot
#define BOOT_CAL_MAX_ATTEMPTS 2            // Maximum retry attempts
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

// T-Cal compensation control (persisted via NVS)
static bool tcal_compensation_enabled = true;

// Temperature direction tracking for hysteresis-aware blending
// Tracks the direction of temperature change at the time of each calibration sample
typedef enum {
	TCAL_DIR_UNKNOWN = 0,
	TCAL_DIR_RISING,
	TCAL_DIR_FALLING,
} tcal_temp_direction_t;

static tcal_temp_direction_t tcal_current_direction = TCAL_DIR_UNKNOWN;
static float tcal_direction_ref_temp = NAN; // Reference temperature for direction detection

// Hysteresis blending EMA factors (new = alpha * measured + (1-alpha) * existing)
// Physically, IMU bias is more stable during warm-up (rising) than during cooling,
// so rising-direction measurements are given higher trust.
#define TCAL_HYSTERESIS_EMA_RISING  0.7f // Rising temp: favor new measurement
#define TCAL_HYSTERESIS_EMA_FALLING 0.3f // Falling temp: preserve warm-up data
#define TCAL_HYSTERESIS_EMA_UNKNOWN 0.5f // Unknown direction: equal blend

// Minimum bias change (dps) required to trigger a flash write.
// Skips NVS/LUT rebuild when successive flushes of the same bucket produce
// negligible change, preventing periodic flash wear during stable rest.
#define TCAL_SAVE_SIGNIFICANCE_THRESHOLD 0.002f

// =============================================================================
// Continuous Accumulator-Based T-Cal Sampling
// =============================================================================
// A single accumulator continuously collects gyro+temperature samples while
// the device is resting. Periodically (every ~20 seconds), the accumulator is
// flushed: the averaged bias is saved to the appropriate temperature bucket.
//
// If temperature drifts more than one bucket width during accumulation, an
// early flush is triggered to avoid cross-bucket contamination.
//
// Benefits over fixed-time one-shot sampling:
// - No temperature gap can be skipped regardless of change rate
// - No temperature change invalidation (each flush is short enough)
// - Points saved at bucket center, eliminating boundary drift issues
// - Low overhead: just sum accumulation, no per-sample bucket lookup

// Flush interval: accumulator is committed every ~25 seconds
#define TCAL_ACCUM_FLUSH_INTERVAL_MS 25000

// Minimum samples required for a valid flush
#define TCAL_ACCUM_MIN_SAMPLES 2000

// Maximum temperature drift within one accumulation window before early flush.
// Slightly larger than bucket width (0.5°C) to allow full bucket coverage
// while preventing cross-bucket contamination.
#define TCAL_ACCUM_TEMP_DRIFT_MAX 0.53f

// Gyro range threshold: if any axis exceeds this during accumulation,
// motion is detected and the accumulator is reset.
#define TCAL_ACCUM_GYRO_RANGE_THRESHOLD 1.5f

static struct {
	double gyro_sum[3];    // Accumulated raw gyro values
	double temp_sum;       // Accumulated temperature values
	int sample_count;      // Number of gyro samples
	int temp_count;        // Number of valid temperature samples
	float min_g[3];        // Gyro min per axis for motion detection
	float max_g[3];        // Gyro max per axis for motion detection
	float temp_min;        // Temperature min during accumulation
	float temp_max;        // Temperature max during accumulation
	bool active;           // Whether accumulator is currently collecting
	int64_t start_time;    // Timestamp when accumulation started
} tcal_accum;

static int64_t tcal_accum_last_commit_time = 0;

// Forward declarations
static void tcal_accum_reset(void);
static void tcal_accum_flush(void);
static void tcal_save_point(int idx, const float bias[3]);

static float last_gyro_tcal_offset[3] = {0.0f, 0.0f, 0.0f};

static void update_tcal_state(void); // Function to refresh T-Cal state

// =============================================================================
// T-Cal Moving Least Squares (MLS) Implementation
// =============================================================================
// MLS provides smooth, continuous bias estimation without discontinuities
// caused by method switching. Uses inverse distance weighting with local
// linear fitting for optimal balance of smoothness and responsiveness.

// MLS Configuration
#define MLS_BANDWIDTH 2.5f       // Temperature bandwidth (°C) - controls locality
#define MLS_MIN_WEIGHT 0.03f     // Minimum weight threshold (~6.5°C distance cutoff)
#define MLS_MAX_POINTS 10        // Maximum points to consider for efficiency
#define MLS_MIN_POINTS_FOR_FIT 4 // Minimum points with significant weight for MLS to be valid
#define MLS_EXTRAP_POINTS 4      // Number of edge points for linear extrapolation (matches LUT)

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
// - At boot, first build entries within ±2°C of current temperature (priority zone)
// - Return quickly to allow other threads to run
// - Continue building remaining entries in small batches during idle time
// - LUT lookup falls back to MLS for entries not yet computed

#define MLS_LUT_STEP_PER_DEGREE 2   // Steps per degree (0.5°C per step)
#define MLS_LUT_STEP_SIZE (1.0f / MLS_LUT_STEP_PER_DEGREE)  // 0.5°C
#define MLS_LUT_TEMP_MIN ((float)CONFIG_SENSOR_POLY_TEMP_MIN)
#define MLS_LUT_TEMP_MAX ((float)CONFIG_SENSOR_POLY_TEMP_MAX)
#define MLS_LUT_SIZE ((int)((CONFIG_SENSOR_POLY_TEMP_MAX - CONFIG_SENSOR_POLY_TEMP_MIN) * MLS_LUT_STEP_PER_DEGREE) + 1)

// Incremental build configuration
#define MLS_LUT_PRIORITY_RANGE 2.0f  // ±2°C around current temp is priority zone
#define MLS_LUT_BATCH_SIZE 8        // Entries to compute per incremental batch
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
	MLS_LUT_BUILD_PRIORITY,     // Building priority zone (±2°C around current temp)
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
static void magneto_reset(void);
static void magneto_online_clear_history(void);
static void magneto_online_reset(void);
static double magneto_online_recent_center(mag_center_estimator_t *center);
static double magneto_online_collect_recent(double ata_out[100], double *norm_sum_out,
                                            float dir_sum_out[3], float *raw_range_out);
static int magneto_online_recent_sample_count(void);
static float magneto_online_recent_dir_bias(void);
static float magneto_online_min_dir_change_threshold(void);
static float magneto_directional_bias(const float ds[3], double count);
static void magneto_center_reset(mag_center_estimator_t *estimator);
static void magneto_center_update(mag_center_estimator_t *estimator, const float m[3]);
static void magneto_center_get(const mag_center_estimator_t *estimator, float center[3]);
static float magneto_center_min_range(const mag_center_estimator_t *estimator);
static bool magneto_center_has_coverage(const mag_center_estimator_t *estimator);
static void magneto_centered_sample(const mag_center_estimator_t *estimator, const float m[3], float centered[3]);
static void magneto_coverage_sample(const mag_center_estimator_t *estimator, const float m[3], float coverage_sample[3]);
static float magneto_norm_sq(const float v[3]);
static bool magneto_normalize_direction(const float v[3], float dir[3]);
static bool magneto_centered_direction(const mag_center_estimator_t *estimator, const float m[3], float dir[3]);
static void magneto_accumulate_direction(float ds[3], const float v[3]);
static void magneto_update_dir_range(const float v[3]);
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
static void sensor_sample_mag_magneto_sample(const float m[3]);

static int sensor_calibration_request(int id);

static void calibration_thread(void);
// Keep background calibration below the sensor loop so trial Magneto solves do
// not preempt FIFO servicing. This makes online/manual calibration a little less
// eager, but avoids sensor-loop timing regressions from background work.
K_THREAD_DEFINE(calibration_thread_id, 4096, calibration_thread, NULL, NULL, NULL, 8, 0, 0);

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

	// Feed raw gyro data into continuous bucket accumulator for auto T-Cal
	// This must happen before bias subtraction so we capture the true raw bias
	// (auto-cal collection continues regardless of compensation enable state)
	if (tcal_auto_calibration_enabled && !isnan(temp)) {
		sensor_tcal_feed_continuous_sample(g, temp);
	}

	// ==========================================================================
	// Unified T-Cal Strategy: LUT -> MLS -> Static Bias
	// D_offset is always applied when valid
	// Skipped when tcal_compensation_enabled is false (uses static bias instead)
	// ==========================================================================

	if (tcal_compensation_enabled && !isnan(temp) && retained->tempCalState.count >= 1) {
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

	// Strategy 2: Final fallback to static bias (when no T-Cal data or compensation disabled)
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

static void magneto_online_runtime_load_retained(void)
{
	magneto_online_reset();
	online_update_count = retained->onlineMagState.update_count;
	online_last_buf_avg_norm = retained->onlineMagState.last_buf_avg_norm;
}

void sensor_calibration_online_mag_retained_save(void)
{
	retained->onlineMagState.update_count = (uint8_t)CLAMP(online_update_count, 0, 255);
	retained->onlineMagState.last_buf_avg_norm = online_last_buf_avg_norm;
}

void sensor_calibration_online_mag_retained_clear(void)
{
	memset(&retained->onlineMagState, 0, sizeof(retained->onlineMagState));
}

void sensor_calibration_online_mag_cold_start(void)
{
	magneto_online_runtime_reset();
	sensor_calibration_online_mag_retained_clear();
}

void sensor_calibration_read(void)
{
	memcpy(sensor_data, retained->sensor_data, sizeof(sensor_data));
	memcpy(accelBias, retained->accelBias, sizeof(accelBias));
	memcpy(gyroBias, retained->gyroBias, sizeof(gyroBias));
	memcpy(magBias, retained->magBias, sizeof(magBias));
	memcpy(magBAinv, retained->magBAinv, sizeof(magBAinv));
	memcpy(accBAinv, retained->accBAinv, sizeof(accBAinv));
	{
		float zero[3] = {0};
		if (v_diff_mag(magBAinv[0], zero) != 0) {
			magneto_online_runtime_load_retained();
			if (online_update_count > 0 || cal_norm_count > 0) {
				LOG_INF("Online mag runtime restored (%d updates, %u norm samples)",
				        online_update_count, cal_norm_count);
			}
		} else {
			magneto_online_runtime_reset();
		}
	}
#if CONFIG_SENSOR_USE_TCAL
	tcal_compensation_enabled = retained->tcal_enabled;
	LOG_INF("T-Cal compensation: %s", tcal_compensation_enabled ? "enabled" : "disabled");
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
	float zero[3] = {0};
	float diagonal[3];
	for (int i = 0; i < 3; i++) {
		diagonal[i] = m_inv[i + 1][i];
	}
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	float max_gain = MAX(MAX(fabsf(diagonal[0]), fabsf(diagonal[1])), fabsf(diagonal[2]));
	if (!v_epsilon(m_inv[0], zero, magnitude * 2.0f)
		|| !v_epsilon(
			diagonal,
			average,
			MAX(magnitude * 0.2f, 0.1f)
		)
		|| max_gain > MAG_CAL_MAX_AXIS_GAIN) // check offset, diagonal spread, and reject huge-gain fits
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
	bool clearing_live_state = (m_inv == NULL || m_inv == magBAinv);
	if (m_inv == NULL) {
		m_inv = magBAinv;
	}
	memset(m_inv, 0, sizeof(magBAinv)); // zeroed matrix will disable magnetometer in fusion
	if (clearing_live_state) {
		magneto_online_runtime_reset();
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
	if (requested_calibration != 0) {
		k_mutex_unlock(&calibration_request_lock);
		LOG_ERR("Sensor calibration is already running");
		return -1;
	}

	sens_cal_axis = axis;
	sens_cal_revolutions = revolutions;
	requested_calibration = 5;
	k_mutex_unlock(&calibration_request_lock);
	return 0;
}
#endif

void sensor_request_calibration_mag(void)
{
	// If already collecting, just check if ready
	if (magneto_progress & 0x80) {
		if (!get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
			set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
		}
		return;
	}

	if (!get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
		set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
	}

	// LED sequence before calibration:
	// 1. Flash LED to let user identify tracker
	LOG_INF("Magnetometer calibration: identify tracker");
	set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_SENSOR);
	k_msleep(2000);  // Wait for pattern to complete

	// 2. Flash twice to indicate calibration is starting
	//    SYS_LED_PATTERN_ONESHOT_PROGRESS: 200ms on + 200ms off, 2 times = 800ms
	set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
	k_msleep(800);

	// Start fresh calibration
	magneto_progress = 0;
	last_magneto_progress = 0;
	magneto_progress_time = 0;
	mag_cal_last_status_log = 0;
	magneto_reset();  // Clear ata buffer and sample count
	magneto_online_reset();  // Clear online accumulator
	magneto_progress |= 1 << 7;  // Set collection active flag
	LOG_INF("Magnetometer calibration started (rotate tracker in all orientations)");
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

#if CONFIG_SENSOR_USE_TCAL
	if (tcal_auto_calibration_enabled && !isnan(avg_temp)) {
		// Auto temperature calibration enabled: save to tcal data points only, don't change gyro bias
		sys_write(MAIN_GYRO_TEMP_ID, &retained->gyroTemp, &avg_temp, sizeof(avg_temp));
		LOG_INF("T-Cal auto-calibration enabled: saving to tcal data only, not updating gyro bias");

		// Update temperature direction tracking for hysteresis-aware blending
		if (!isnan(tcal_direction_ref_temp)) {
			float delta = avg_temp - tcal_direction_ref_temp;
			if (delta > 0.2f) {
				tcal_current_direction = TCAL_DIR_RISING;
			} else if (delta < -0.2f) {
				tcal_current_direction = TCAL_DIR_FALLING;
			}
			// If delta is within ±0.2°C, keep previous direction (noise filter)
		}
		tcal_direction_ref_temp = avg_temp;

		// Check if T-Cal coverage is good - if so, skip saving a redundant point
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
			// 2. Has both upper and lower bounds AND closest is within 1x sampling interval
			if (closest_distance <= sampling_interval) {
				// Very close to existing point - definitely good coverage
				has_good_coverage = true;
				LOG_INF(
					"T-Cal: Excellent coverage at %.2fC (closest: %.2fC away, within sampling interval)",
					(double)avg_temp,
					(double)closest_distance
				);
			} else if (has_lower_bound && has_upper_bound && closest_distance <= sampling_interval * 1.0f) {
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
						(double)(sampling_interval * 1.0f)
					);
				}
			}
		}

		if (has_good_coverage) {
			LOG_INF(
				"T-Cal: Coverage sufficient at %.2fC, skipping point save",
				(double)avg_temp
			);
		}

		if (!has_good_coverage) {
			// Save as new calibration point
			LOG_INF(
				"T-Cal: Saving calibration point at average temp %.2fC (range: %.2fC)",
				(double)avg_temp,
				(double)temp_range
			);
			int idx = TEMP_TO_IDX(avg_temp);
			if (idx >= 0 && idx < TCAL_BUFFER_SIZE) {

				// Hysteresis-aware blending: prefer rising-phase data.
				// Use tcal_current_direction directly — do not infer from temp comparison.
				if (retained->tempCalPoints[idx].temp != 0.0f) {
					float ema_alpha;
					switch (tcal_current_direction) {
					case TCAL_DIR_RISING:  ema_alpha = TCAL_HYSTERESIS_EMA_RISING;  break;
					case TCAL_DIR_FALLING: ema_alpha = TCAL_HYSTERESIS_EMA_FALLING; break;
					default:               ema_alpha = TCAL_HYSTERESIS_EMA_UNKNOWN; break;
					}
					LOG_INF(
						"T-Cal: Blending with existing point (dir: %s, alpha: %.2f)",
						tcal_current_direction == TCAL_DIR_RISING ? "rising" :
						tcal_current_direction == TCAL_DIR_FALLING ? "falling" : "unknown",
						(double)ema_alpha
					);
					for (int axis = 0; axis < 3; axis++) {
						g_bias[axis] = ema_alpha * g_bias[axis] +
						               (1.0f - ema_alpha) * retained->tempCalPoints[idx].bias[axis];
					}
					LOG_INF(
						"T-Cal: Blended bias: %.5f %.5f %.5f",
						(double)g_bias[0], (double)g_bias[1], (double)g_bias[2]
					);
				} else {
					retained->tempCalState.count++; // New slot
				}
				retained->tempCalPoints[idx].temp = avg_temp;
				memcpy(retained->tempCalPoints[idx].bias, g_bias, sizeof(g_bias));
				retained->tempCalState.valid = false; // Invalidate old curve
				// Manual calibration always writes NVS (user-initiated, not periodic)
				update_tcal_state();

			} else {
				LOG_WRN(
					"T-Cal: Temperature %.2fC is outside the configured calibration range. Point not saved.",
					(double)avg_temp
				);
			}
		}
	} else {
		// Auto tcal not enabled or no valid temperature: save gyro bias to NVS only
		sys_write(MAIN_GYRO_BIAS_ID, &retained->gyroBias, gyroBias, sizeof(gyroBias));
		LOG_INF("Saving gyro bias to NVS (auto tcal not enabled)");
	}
#else
	// No tcal support: always save gyro bias to NVS
	sys_write(MAIN_GYRO_BIAS_ID, &retained->gyroBias, gyroBias, sizeof(gyroBias));
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
		magneto_reset();
		if (get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
			set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
		}
		return -1; // magnetometer calibration already exists
	}

	if (!get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
		set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
	}

	float m[3];
	if (sensor_wait_mag(m, K_MSEC(1000))) {
		return -1; // Timeout
	}
	sensor_sample_mag_magneto_sample(m); // 400us

	// Periodic status log every 1 second
	int64_t now = k_uptime_get();
	if (now - mag_cal_last_status_log >= 1000) {
		mag_cal_last_status_log = now;
		LOG_INF("Mag cal: %d samples collected", (int)sample_count);
	}

	if (magneto_progress != 0b11111111) {
		return 1;  // Still collecting - signal caller to use short sleep
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
		if (get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
			set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
		}
		return -1;
	} else {
		LOG_INF("Applying calibration");
		memcpy(magBAinv, m_inv, sizeof(magBAinv));
		magneto_online_runtime_reset();  // Restart online calibration from a clean baseline
#if CONFIG_SENSOR_USE_VQF
		vqf_reset_mag_ref();
		sensor_mag_ref_reset(); // Recompute magRef from new calibration
#endif
		// fusion invalidation not necessary
	}
	sys_write(MAIN_MAG_BIAS_ID, &retained->magBAinv, magBAinv, sizeof(magBAinv));

	LOG_INF("Finished calibration");
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
	sensor_refresh_sensor_ids(); // Refresh reported mag status after calibration
	if (get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
		set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
	}
	return 0;
}

// =============================================================================
// Gyro sensitivity (scale-factor) calibration
// =============================================================================
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
// The user spins the tracker a known number of full revolutions about a single
// axis. We integrate the measured gyro rate over that motion and compare the
// measured angle against the true angle to derive a per-axis scale factor that
// corrects cumulative over- or under-rotation.
#define SENS_CAL_BIAS_SAMPLE_MS   1000   // In-situ bias averaging window
#define SENS_CAL_START_RATE_DPS   30.0f  // Rate that counts as "spin started"
#define SENS_CAL_STOP_RATE_DPS    10.0f  // Rate that counts as "spin stopped"
#define SENS_CAL_STOP_DWELL_MS    1000   // Rate must stay low this long to stop
#define SENS_CAL_START_TIMEOUT_MS 30000  // Give up waiting for the spin to start
#define SENS_CAL_SPIN_TIMEOUT_MS  60000  // Give up waiting for the spin to finish
#define SENS_CAL_MIN_FRACTION     0.85f  // Require near-complete expected angle before stopping
#define SENS_CAL_MIN_SCALE        0.9f   // Reject implausible results (likely wrong turn count)
#define SENS_CAL_MAX_SCALE        1.1f
#define SENS_CAL_WARN_OFF_AXIS_RATIO 0.10f
#define SENS_CAL_MAX_OFF_AXIS_RATIO 0.25f

static void sensor_calibrate_sens(void)
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
		if (fabsf(rate) < SENS_CAL_STOP_RATE_DPS &&
		    fabs(measured) >= (double)(expected_deg * SENS_CAL_MIN_FRACTION)) {
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

	if (!isfinite(scale)) {
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
			"Gyro sensitivity auto-calibration rejected: measured %.2f deg for %.2f deg, scale %.5f outside %.2f..%.2f. Equivalent sens diff over %u rev: %.3f deg. Nothing saved.\n",
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
	sys_write(
		MAIN_GYRO_SENS_ID,
		&retained->gyroSensScale,
		retained->gyroSensScale,
		sizeof(retained->gyroSensScale)
	);

	LOG_INF("Sensitivity calibration: axis %c scale set to %.5f", axis_char, (double)scale);
	if (off_axis_ratio > SENS_CAL_WARN_OFF_AXIS_RATIO) {
		printk(
			"Gyro sensitivity auto-calibration saved: axis %c, scale %.5f, equivalent sens diff over %u rev: %.3f deg, off-axis %.2f. Axis alignment was loose; repeating may improve accuracy.\n",
			axis_char,
			(double)scale,
			(unsigned int)CONFIG_SENSOR_SENS_REV,
			(double)equivalent_diff_deg,
			(double)off_axis_ratio
		);
	} else {
		printk(
			"Gyro sensitivity auto-calibration saved: axis %c, scale %.5f, equivalent sens diff over %u rev: %.3f deg, off-axis %.2f.\n",
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

static void magneto_reset(void)
{
	magneto_progress = 0;
	last_magneto_progress = 0;
	magneto_progress_time = 0;
	memset(ata, 0, sizeof(ata));
	norm_sum = 0;
	sample_count = 0;
	for (int i = 0; i < 3; i++) {
		dir_min[i] = 2.0f;   // start high
		dir_max[i] = -2.0f;  // start low
	}
	magneto_center_reset(&manual_center_estimator);
	memset(manual_last_dir, 0, sizeof(manual_last_dir));
	memset(manual_last_accel_dir, 0, sizeof(manual_last_accel_dir));
}

static void magneto_online_reset(void)
{
	magneto_online_clear_history();
	online_last_sample_time = 0;
	online_last_update_time = 0;
}

static void magneto_online_runtime_reset(void)
{
	magneto_online_reset();
	online_update_count = 0;
	online_last_buf_avg_norm = 0.0f;
	cal_norm_count = 0;
	cal_norm_ema = 0.0f;
	cal_norm_var_ema = 0.0f;
}

static void magneto_online_clear_history(void)
{
	memset(quad_buf, 0, sizeof(quad_buf));
	online_total_sample_count = 0;
	online_last_checked_sample_count = 0;
	online_last_check_time = 0;
	memset(online_last_dir, 0, sizeof(online_last_dir));
	memset(online_last_accel_dir, 0, sizeof(online_last_accel_dir));
	magneto_center_reset(&online_center_estimator);
	// Suppress collection for a few seconds so transient/stale samples from
	// wake-up, reboot, or environment transitions are not mixed into the
	// fresh buffer.
	online_collection_suppress_until = k_uptime_get() + ONLINE_COLLECTION_SUPPRESS_MS;
}

static bool magneto_online_quadrant_is_recent(const quadrant_buf_t *qbuf)
{
	if (qbuf->count == 0) {
		return false;
	}
	return (online_total_sample_count - qbuf->last_seq) <= ONLINE_STALE_QUADRANT_MAX_AGE;
}

static double magneto_online_recent_center(mag_center_estimator_t *center)
{
	magneto_center_reset(center);

	double count = 0;
	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent(&quad_buf[q])) {
			continue;
		}
		for (int i = 0; i < quad_buf[q].count; i++) {
			quadrant_sample_t *s = &quad_buf[q].samples[i];
			float m[3] = {s->x, s->y, s->z};
			magneto_center_update(center, m);
			count++;
		}
	}

	return count;
}

// Collect all valid samples from all 8 quadrant ring buffers.
// Recomputes ATA, norm_sum, centered dir_sum, and raw min/max range from raw samples.
static double magneto_online_collect_recent(double ata_out[100], double *norm_sum_out,
                                            float dir_sum_out[3], float *raw_range_out)
{
	memset(ata_out, 0, sizeof(double) * 100);
	*norm_sum_out = 0;
	memset(dir_sum_out, 0, sizeof(float) * 3);

	mag_center_estimator_t recent_center;
	double recent_sample_count = magneto_online_recent_center(&recent_center);
	if (raw_range_out) {
		*raw_range_out = magneto_center_min_range(&recent_center);
	}

	double fit_sample_count = 0;
	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent(&quad_buf[q])) {
			continue;
		}
		for (int i = 0; i < quad_buf[q].count; i++) {
			quadrant_sample_t *s = &quad_buf[q].samples[i];
			magneto_sample((double)s->x, (double)s->y, (double)s->z, ata_out, norm_sum_out, &fit_sample_count);
			float raw[3] = {s->x, s->y, s->z};
			float coverage_sample[3];
			magneto_coverage_sample(&recent_center, raw, coverage_sample);
			magneto_accumulate_direction(dir_sum_out, coverage_sample);
		}
	}
	return recent_sample_count;
}

static int magneto_online_recent_sample_count(void)
{
	int count = 0;
	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent(&quad_buf[q])) {
			continue;
		}
		count += quad_buf[q].count;
	}
	return count;
}

static float magneto_online_recent_dir_bias(void)
{
	float dir_sum_recent[3] = {0};
	mag_center_estimator_t recent_center;
	double recent_sample_count = magneto_online_recent_center(&recent_center);

	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent(&quad_buf[q])) {
			continue;
		}
		for (int i = 0; i < quad_buf[q].count; i++) {
			quadrant_sample_t *s = &quad_buf[q].samples[i];
			float raw[3] = {s->x, s->y, s->z};
			float coverage_sample[3];
			magneto_coverage_sample(&recent_center, raw, coverage_sample);
			magneto_accumulate_direction(dir_sum_recent, coverage_sample);
		}
	}

	return magneto_directional_bias(dir_sum_recent, recent_sample_count);
}

static float magneto_online_min_dir_change_threshold(void)
{
	static bool initialized = false;
	static float threshold = 0.0f;

	if (!initialized) {
		const float deg_to_rad = 0.01745329251994329577f;
		threshold = 1.0f - cosf(ONLINE_MIN_DIR_CHANGE_DEG * deg_to_rad);
		initialized = true;
	}

	return threshold;
}

/**
 * Compute directional bias of accumulated mag samples.
 * Returns |sum(m/|m|)| / N, where 0=perfect sphere coverage, 1=all same direction.
 */
static float magneto_directional_bias(const float ds[3], double count)
{
	if (count < 2) {
		return 1.0f;
	}
	float inv_n = 1.0f / (float)count;
	float cx = ds[0] * inv_n;
	float cy = ds[1] * inv_n;
	float cz = ds[2] * inv_n;
	return sqrtf(cx * cx + cy * cy + cz * cz);
}

/**
 * Update a min/max hard-iron center estimate for coverage calculations.
 */
static void magneto_center_reset(mag_center_estimator_t *estimator)
{
	memset(estimator, 0, sizeof(*estimator));
}

static void magneto_center_update(mag_center_estimator_t *estimator, const float m[3])
{
	if (!estimator->initialized) {
		memcpy(estimator->min, m, sizeof(estimator->min));
		memcpy(estimator->max, m, sizeof(estimator->max));
		estimator->initialized = true;
		return;
	}

	for (int i = 0; i < 3; i++) {
		if (m[i] < estimator->min[i]) { estimator->min[i] = m[i]; }
		if (m[i] > estimator->max[i]) { estimator->max[i] = m[i]; }
	}
}

static void magneto_center_get(const mag_center_estimator_t *estimator, float center[3])
{
	if (!estimator->initialized) {
		memset(center, 0, sizeof(float) * 3);
		return;
	}

	for (int i = 0; i < 3; i++) {
		center[i] = (estimator->min[i] + estimator->max[i]) * 0.5f;
	}
}

static float magneto_center_min_range(const mag_center_estimator_t *estimator)
{
	if (!estimator->initialized) {
		return 0.0f;
	}

	float min_range = estimator->max[0] - estimator->min[0];
	for (int i = 1; i < 3; i++) {
		float range = estimator->max[i] - estimator->min[i];
		if (range < min_range) {
			min_range = range;
		}
	}
	return min_range;
}

static bool magneto_center_has_coverage(const mag_center_estimator_t *estimator)
{
	return magneto_center_min_range(estimator) >= MAG_CAL_MIN_RAW_AXIS_RANGE;
}

static void magneto_centered_sample(const mag_center_estimator_t *estimator, const float m[3], float centered[3])
{
	float center[3];
	magneto_center_get(estimator, center);
	for (int i = 0; i < 3; i++) {
		centered[i] = m[i] - center[i];
	}
}

static void magneto_coverage_sample(const mag_center_estimator_t *estimator, const float m[3], float coverage_sample[3])
{
	if (magneto_center_has_coverage(estimator)) {
		magneto_centered_sample(estimator, m, coverage_sample);
	} else {
		memcpy(coverage_sample, m, sizeof(float) * 3);
	}
}

static float magneto_norm_sq(const float v[3])
{
	return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

static bool magneto_normalize_direction(const float v[3], float dir[3])
{
	float norm_sq = magneto_norm_sq(v);
	if (norm_sq < 1e-8f) {
		return false;
	}
#if CONFIG_CMSIS_DSP
	float norm;
	arm_sqrt_f32(norm_sq, &norm);
	float inv_norm = 1.0f / norm;
#else
	float inv_norm = 1.0f / sqrtf(norm_sq);
#endif
	dir[0] = v[0] * inv_norm;
	dir[1] = v[1] * inv_norm;
	dir[2] = v[2] * inv_norm;
	return true;
}

static bool magneto_centered_direction(const mag_center_estimator_t *estimator, const float m[3], float dir[3])
{
	float centered[3];
	if (magneto_center_has_coverage(estimator)) {
		magneto_centered_sample(estimator, m, centered);
		if (magneto_normalize_direction(centered, dir)) {
			return true;
		}
	}

	// First sample, or a sample very near the provisional center: keep the old
	// raw-direction fallback so calibration can bootstrap.
	return magneto_normalize_direction(m, dir);
}

/**
 * Accumulate a normalized direction for diversity tracking.
 */
static void magneto_accumulate_direction(float ds[3], const float v[3])
{
	float dir[3];
	if (!magneto_normalize_direction(v, dir)) {
		return;
	}
	ds[0] += dir[0];
	ds[1] += dir[1];
	ds[2] += dir[2];
}

/**
 * Update direction range tracking (min/max per axis of normalized direction).
 * Used only by manual calibration to ensure sufficient multi-axis rotation.
 */
static void magneto_update_dir_range(const float v[3])
{
	float d[3];
	if (!magneto_normalize_direction(v, d)) {
		return;
	}
	for (int i = 0; i < 3; i++) {
		if (d[i] < dir_min[i]) { dir_min[i] = d[i]; }
		if (d[i] > dir_max[i]) { dir_max[i] = d[i]; }
	}
}

/**
 * Get minimum direction range across all three axes.
 * Range = max - min of normalized direction per axis.
 * Higher = more diverse coverage.
 */
static float magneto_min_dir_range(void)
{
	float min_range = 2.0f;
	for (int i = 0; i < 3; i++) {
		float range = dir_max[i] - dir_min[i];
		if (range < min_range) {
			min_range = range;
		}
	}
	return min_range;
}

/**
 * Check if a Magneto calibration result passes quality checks.
 * Performs silent validation (no warning logs) since this is called speculatively.
 * If m_inv_out is non-NULL and check passes, the computed calibration is stored there.
 * Returns true if quality is acceptable.
 */
static bool magneto_quality_check(double *ata_buf, double norm_sum_val, double sample_count_val,
                                  float m_inv_out[][3])
{
	if (sample_count_val < MAG_CAL_MIN_SAMPLES) {
		return false;
	}

	// Run trial calibration
	float m_inv[4][3];
	magneto_current_calibration(m_inv, ata_buf, norm_sum_val, sample_count_val);

	// Silent validation: check bias < 1 and diagonals within 20%
	// (same as sensor_calibration_validate_mag but without LOG_WRN or clearing)
	float zero[3] = {0};
	float diagonal[3];
	for (int i = 0; i < 3; i++) {
		diagonal[i] = m_inv[i + 1][i];
	}
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	float max_gain = MAX(MAX(fabsf(diagonal[0]), fabsf(diagonal[1])), fabsf(diagonal[2]));
	float hm = (float)(norm_sum_val / sample_count_val);
	if (!v_epsilon(m_inv[0], zero, hm * 2.0f)
	    || !v_epsilon(diagonal, average, MAX(magnitude * 0.2f, 0.1f))
	    || max_gain > MAG_CAL_MAX_AXIS_GAIN) {
		return false;
	}

	if (m_inv_out) {
		memcpy(m_inv_out, m_inv, sizeof(m_inv));
	}
	return true;
}

/**
 * Compute similarity between two BAinv calibration matrices.
 * Uses normalized Frobenius norm: similarity = 1.0 - ||candidate - existing|| / ||existing||.
 * Returns 1.0 for identical calibrations, approaching 0 for very different ones.
 * The offset row (row 0) and soft-iron rows (1-3) contribute equally to the norm.
 */
static float magneto_BAinv_similarity(float existing[4][3], float candidate[4][3])
{
	float diff_norm_sq = 0;
	float existing_norm_sq = 0;

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 3; c++) {
			float d = candidate[r][c] - existing[r][c];
			diff_norm_sq += d * d;
			existing_norm_sq += existing[r][c] * existing[r][c];
		}
	}

	if (existing_norm_sq < 1e-12f) {
		// Existing calibration is near-zero (identity): treat as low similarity
		return 0.0f;
	}

	float similarity = 1.0f - sqrtf(diff_norm_sq / existing_norm_sq);
	if (similarity < 0.0f) { similarity = 0.0f; }
	if (similarity > 1.0f) { similarity = 1.0f; }
	return similarity;
}

/**
 * Blend two BAinv calibrations using exponential moving average (EMA).
 * blended = (1 - alpha) * existing + alpha * candidate
 * Alpha is computed from similarity: more similar → lower alpha (conservative),
 * more different → higher alpha (adaptive to environmental change).
 *
 * The blended result is validated with the same structural checks as
 * magneto_quality_check. If blending produces an invalid result, the
 * candidate is used directly (fallback to full replacement).
 *
 * Returns true if the blend (or fallback) is valid, false if both are invalid.
 */
static bool magneto_blend_BAinv(float out[4][3], float existing[4][3],
                                float candidate[4][3])
{
	float similarity = magneto_BAinv_similarity(existing, candidate);

	// Compute adaptive blending weight
	float alpha;
	if (similarity >= ONLINE_BLEND_SIMILARITY_HIGH) {
		// Very similar: candidate is just a minor refinement → low alpha
		alpha = ONLINE_BLEND_MIN_ALPHA;
	} else if (similarity <= ONLINE_BLEND_SIMILARITY_LOW) {
		// Significant divergence (possible environment change) → high alpha
		alpha = ONLINE_BLEND_MAX_ALPHA;
	} else {
		// Linear interpolation between low and high thresholds
		float t = (similarity - ONLINE_BLEND_SIMILARITY_LOW)
		        / (ONLINE_BLEND_SIMILARITY_HIGH - ONLINE_BLEND_SIMILARITY_LOW);
		alpha = ONLINE_BLEND_MAX_ALPHA
		      + t * (ONLINE_BLEND_MIN_ALPHA - ONLINE_BLEND_MAX_ALPHA);
	}

	// Blend
	float blended[4][3];
	float one_minus_alpha = 1.0f - alpha;
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 3; c++) {
			blended[r][c] = one_minus_alpha * existing[r][c]
			              + alpha * candidate[r][c];
		}
	}

	// Validate blended result
	float zero[3] = {0};
	float diagonal[3];
	for (int i = 0; i < 3; i++) {
		diagonal[i] = blended[i + 1][i];
	}
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	float max_gain = MAX(MAX(fabsf(diagonal[0]), fabsf(diagonal[1])), fabsf(diagonal[2]));
	if (v_epsilon(blended[0], zero, 1)
	    && v_epsilon(diagonal, average, MAX(magnitude * 0.2f, 0.1f))
	    && max_gain <= MAG_CAL_MAX_AXIS_GAIN) {
		// Blended result is valid
		memcpy(out, blended, sizeof(blended));
		return true;
	}

	// Fallback: use candidate directly if blend is invalid.
	// Candidate was already validated by magneto_quality_check before this call,
	// so validate with its own diagonal here for consistency.
	{
		float c_diag[3];
		for (int i = 0; i < 3; i++) {
			c_diag[i] = candidate[i + 1][i];
		}
		float c_avg = v_avg(c_diag);
		float c_avg_arr[3] = {c_avg, c_avg, c_avg};
		float c_max_gain = MAX(MAX(fabsf(c_diag[0]), fabsf(c_diag[1])), fabsf(c_diag[2]));
		if (v_epsilon(candidate[0], zero, 1)
		    && v_epsilon(c_diag, c_avg_arr, MAX(c_avg * 0.2f, 0.1f))
		    && c_max_gain <= MAG_CAL_MAX_AXIS_GAIN) {
			memcpy(out, candidate, sizeof(float) * 4 * 3);
			return true;
		}
	}

	// Both blend and fallback invalid — should not happen since candidate was
	// already validated by magneto_quality_check before calling this function
	return false;
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
	if (!isnan(current_temp) && current_temp > -20.0f && current_temp < 60.0f) {
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
		if (!isnan(current_temp) && current_temp > -20.0f && current_temp < 60.0f) {
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
// Timeout for waiting for new pose (30 seconds in milliseconds)
#define CALIB_POSE_TIMEOUT_MS 30000
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

// Collect magnetometer sample for manual calibration.
// Accumulates into ATA and periodically runs trial calibration for quality check.
static void sensor_sample_mag_magneto_sample(const float m[3])
{
	// Direction diversity gate: accept sample if either mag direction OR
	// accelerometer (gravity) direction has changed since last accepted.
	// Identical logic to the online path — breaks direction lock-in during
	// magnetic interference by cross-validating with physical orientation.
	//
	// Gate accelerometer by magnitude as well: skip the direction check
	// entirely when the device is under strong linear acceleration, falling
	// back to mag-only for that sample.
	float accel_mag_sq = aBuf[0] * aBuf[0] + aBuf[1] * aBuf[1] + aBuf[2] * aBuf[2];
	bool accel_trustworthy = (accel_mag_sq >= MAG_CAL_ACCEL_MAG_MIN_SQ
	                       && accel_mag_sq <= MAG_CAL_ACCEL_MAG_MAX_SQ);

	float raw_mag[3] = {m[0], m[1], m[2]};
	float cur_mag_dir[3];
	if (magneto_norm_sq(raw_mag) < 1e-8f) {
		return;
	}
	magneto_center_update(&manual_center_estimator, raw_mag);
	if (!magneto_centered_direction(&manual_center_estimator, raw_mag, cur_mag_dir)) {
		return;
	}

	// Normalize accelerometer (if trustworthy)
	float cur_accel_dir[3] = {0};
	if (accel_trustworthy) {
		float accel_inv = 1.0f / sqrtf(accel_mag_sq);
		cur_accel_dir[0] = aBuf[0] * accel_inv;
		cur_accel_dir[1] = aBuf[1] * accel_inv;
		cur_accel_dir[2] = aBuf[2] * accel_inv;
	}

	if (sample_count > 0) {
		float min_change = magneto_online_min_dir_change_threshold();

		// Mag direction change
		float mag_dot = cur_mag_dir[0] * manual_last_dir[0]
		              + cur_mag_dir[1] * manual_last_dir[1]
		              + cur_mag_dir[2] * manual_last_dir[2];
		bool mag_changed = (1.0f - mag_dot >= min_change);

		// Accel direction change (only if accel is trustworthy)
		bool accel_changed = false;
		if (accel_trustworthy) {
			float accel_dot = cur_accel_dir[0] * manual_last_accel_dir[0]
			                + cur_accel_dir[1] * manual_last_accel_dir[1]
			                + cur_accel_dir[2] * manual_last_accel_dir[2];
			accel_changed = (1.0f - accel_dot >= min_change);
		}

		// Accept only if at least one direction source shows movement
		if (!mag_changed && !accel_changed) {
			return; // redundant sample
		}
	}

	// Update last accepted directions
	manual_last_dir[0] = cur_mag_dir[0];
	manual_last_dir[1] = cur_mag_dir[1];
	manual_last_dir[2] = cur_mag_dir[2];
	if (accel_trustworthy) {
		manual_last_accel_dir[0] = cur_accel_dir[0];
		manual_last_accel_dir[1] = cur_accel_dir[1];
		manual_last_accel_dir[2] = cur_accel_dir[2];
	}

	// Accept sample - add to Magneto accumulator
	magneto_sample(m[0], m[1], m[2], ata, &norm_sum, &sample_count); // 400us
	float coverage_mag[3];
	magneto_coverage_sample(&manual_center_estimator, raw_mag, coverage_mag);
	magneto_update_dir_range(coverage_mag);

	// Attempt trial calibration every MAG_CAL_TRIAL_INTERVAL samples
	if (sample_count >= MAG_CAL_MIN_SAMPLES &&
	    (int)sample_count % MAG_CAL_TRIAL_INTERVAL < 1) {
		float min_range = magneto_min_dir_range();
		float raw_range = magneto_center_min_range(&manual_center_estimator);
		LOG_INF("Mag cal check: %d samples, min_range=%.2f (need %.2f), raw_range=%.3f (need %.3f)",
		        (int)sample_count, (double)min_range, (double)MAG_CAL_MIN_DIR_RANGE,
		        (double)raw_range, (double)MAG_CAL_MIN_RAW_AXIS_RANGE);

		// Require minimum directional coverage before attempting calibration
		if (min_range < MAG_CAL_MIN_DIR_RANGE || raw_range < MAG_CAL_MIN_RAW_AXIS_RANGE) {
			LOG_INF("Mag cal: need more rotation, keep turning");
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
			return;
		}

		if (magneto_quality_check(ata, norm_sum, sample_count, NULL)) {
			magneto_progress |= 0b01111111;
			LOG_INF("Mag cal ready: %d samples, min_range=%.2f",
			        (int)sample_count, (double)min_range);
			set_led(SYS_LED_PATTERN_FLASH, SYS_LED_PRIORITY_SENSOR);
		} else {
			LOG_INF("Mag cal: not ready yet, keep rotating (%d samples)",
			        (int)sample_count);
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
		}
	}
}

// Phase 2: Background online magnetometer calibration
// Called from sensor loop for each new raw mag sample during normal operation.
// Gated by: VQF disturbance detection, accel magnitude, time interval, and direction change.
void sensor_calibration_online_mag_sample(const float m[3])
{
	// Don't accumulate during manual calibration
	if (magneto_progress & 0x80) {
		return;
	}

	int64_t now = k_uptime_get();

	// Track VQF disturbance duration (before any sample gates) so the
	// background check function knows how long disturbance has persisted.
#if CONFIG_SENSOR_USE_VQF
	if (vqf_get_mag_dist_detected()) {
		if (online_vqf_dist_start_time == 0) {
			online_vqf_dist_start_time = now;
		}
	} else {
		online_vqf_dist_start_time = 0;
	}
#endif

	// Suppress collection after buffer resets (wake-up, reboot, environment
	// change, calibration update) to let sensor data stabilise.
	if (now < online_collection_suppress_until) {
		return;
	}

	// Reject if VQF detects magnetic disturbance (only when we have an existing
	// calibration — VQF only receives mag data when calibrated, so mag_dist_detected
	// is meaningless without calibration).
	// Exception 1: if current calibration quality is bad (norm CV > 6%), the disturbance
	// detection itself may be unreliable due to the bad calibration, so skip the gate.
	// Exception 2: if VQF has been reporting disturbance continuously for a long time,
	// the "disturbance" is likely a calibration drift or environment change rather than
	// transient interference.  Allow samples through to enable recalibration.
	// Without this, a deadlock occurs: VQF reports disturbance → gate blocks samples →
	// cal_norm_count stops updating (guarded by !magDistDetected in sensor.c) → CV
	// stays frozen at a low value → gate never opens → no recalibration possible.
#if CONFIG_SENSOR_USE_VQF
	{
		float zero[3] = {0};
		bool has_cal = (v_diff_mag(magBAinv[0], zero) != 0);
		float current_cv = sensor_calibration_get_mag_quality();
		if (has_cal && current_cv < 0.06f && vqf_get_mag_dist_detected()) {
			// Sustained disturbance override: if disturbance has persisted for
			// more than 5 seconds, allow samples through.
			bool sustained = (online_vqf_dist_start_time > 0 &&
			                  (now - online_vqf_dist_start_time) > 5000);
			if (!sustained) {
				return;
			}
		}
	}
#endif

	// Rate limit: minimum interval between samples
	if (now - online_last_sample_time < ONLINE_MIN_INTERVAL_MS) {
		return;
	}

	// Gate by accel magnitude: reject samples under strong linear acceleration
	float accel_mag_sq = aBuf[0] * aBuf[0] + aBuf[1] * aBuf[1] + aBuf[2] * aBuf[2];
	if (accel_mag_sq < MAG_CAL_ACCEL_MAG_MIN_SQ || accel_mag_sq > MAG_CAL_ACCEL_MAG_MAX_SQ) {
		return;
	}

	// Direction diversity gate: accept sample if either mag direction OR
	// accelerometer (gravity) direction has changed since last accepted sample.
	// Pure magnetometer-based direction check can suffer from "direction lock-in"
	// during strong magnetic interference: the mag reading points to a distorted
	// but stable direction while the tracker physically rotates.  The accelerometer
	// cross-check breaks this deadlock — if the device has physically moved
	// (accel direction changed), accept the sample regardless of mag direction.
	float raw_mag[3] = {m[0], m[1], m[2]};
	float cur_dir[3];
	if (magneto_norm_sq(raw_mag) < 1e-8f) {
		return;
	}
	magneto_center_update(&online_center_estimator, raw_mag);
	if (!magneto_centered_direction(&online_center_estimator, raw_mag, cur_dir)) {
		return;
	}

	// Normalize accelerometer to get gravity direction
	// aBuf magnitude already validated (~1g) by the accel gate above
	float accel_norm = sqrtf(accel_mag_sq);
	float accel_inv = 1.0f / accel_norm;
	float cur_accel_dir[3] = {aBuf[0] * accel_inv, aBuf[1] * accel_inv, aBuf[2] * accel_inv};

	if (online_total_sample_count > 0) {
		float mag_dot = cur_dir[0] * online_last_dir[0]
		              + cur_dir[1] * online_last_dir[1]
		              + cur_dir[2] * online_last_dir[2];
		float accel_dot = cur_accel_dir[0] * online_last_accel_dir[0]
		                + cur_accel_dir[1] * online_last_accel_dir[1]
		                + cur_accel_dir[2] * online_last_accel_dir[2];

		float min_change = magneto_online_min_dir_change_threshold();
		bool mag_changed = (1.0f - mag_dot >= min_change);
		bool accel_changed = (1.0f - accel_dot >= min_change);

		if (!mag_changed && !accel_changed) {
			return; // neither mag nor accel direction changed enough
		}
	}

	online_last_sample_time = now;
	online_last_dir[0] = cur_dir[0];
	online_last_dir[1] = cur_dir[1];
	online_last_dir[2] = cur_dir[2];
	online_last_accel_dir[0] = cur_accel_dir[0];
	online_last_accel_dir[1] = cur_accel_dir[1];
	online_last_accel_dir[2] = cur_accel_dir[2];

	float route_mag[3];
	magneto_coverage_sample(&online_center_estimator, raw_mag, route_mag);
	if (magneto_norm_sq(route_mag) < 1e-8f) {
		memcpy(route_mag, raw_mag, sizeof(route_mag));
	}

	// Route sample to its octant based on sign relative to the min/max center.
	// This guarantees each octant independently rolls its ring buffer,
	// preventing a single orientation from evicting diverse data in other octants.
	int octant = 0;
	if (route_mag[0] < 0) octant |= 1;
	if (route_mag[1] < 0) octant |= 2;
	if (route_mag[2] < 0) octant |= 4;

	quadrant_buf_t *qbuf = &quad_buf[octant];
	online_total_sample_count++;
	qbuf->last_seq = online_total_sample_count;
	qbuf->samples[qbuf->head].x = m[0];
	qbuf->samples[qbuf->head].y = m[1];
	qbuf->samples[qbuf->head].z = m[2];
	qbuf->head = (qbuf->head + 1) % QUADRANT_BUF_SIZE;
	if (qbuf->count < QUADRANT_BUF_SIZE) {
		qbuf->count++;
	}
}

static bool sensor_calibration_online_mag_check(void)
{
	int recent_sample_count_now = magneto_online_recent_sample_count();
	int64_t now = k_uptime_get();

	if (recent_sample_count_now < MAG_CAL_MIN_SAMPLES) {
		return false;
	}
	if (online_total_sample_count == online_last_checked_sample_count) {
		return false;
	}
	if (online_last_check_time > 0 &&
	    (now - online_last_check_time) < ONLINE_MIN_CHECK_INTERVAL_MS) {
		return false;
	}

	online_last_checked_sample_count = online_total_sample_count;
	online_last_check_time = now;

	float zero[3] = {0};
	bool has_existing = (v_diff_mag(magBAinv[0], zero) != 0);
	float current_cv = has_existing ? sensor_calibration_get_mag_quality() : 1.0f;

	// When VQF has a reliable calibration (CV < 4%) and is NOT experiencing
	// sustained magnetic disturbance, skip the calibration check entirely.
	// Updating calibration resets VQF's mag reference (vqf_reset_mag_ref),
	// causing ~6s of heading instability.  Only attempt a recalibration
	// when VQF has been detecting disturbance for >3 seconds, indicating
	// the current calibration is genuinely insufficient.
#if CONFIG_SENSOR_USE_VQF
	if (has_existing && current_cv < 0.04f) {
		if (online_vqf_dist_start_time == 0) {
			return false; // VQF not disturbed — current cal is fine
		}
		int64_t dist_duration_ms = now - online_vqf_dist_start_time;
		if (dist_duration_ms < ONLINE_VQF_DIST_MIN_DURATION_MS) {
			return false; // transient disturbance — wait
		}
	}
#endif

	// If the current calibration is already good enough AND we've had enough
	// updates to trust that assessment, skip the heavy Magneto fit.
	//
	// Two-tier convergence:
	//   Tier 1 (strict): CV is good AND directional coverage is adequate
	//     → require low dir_bias (sphere sampled evenly)
	//   Tier 2 (relaxed): CV is excellent AND we've done many updates
	//     → trust the fit regardless of dir_bias (directional fluctuations
	//       during normal rotation are just sampling noise, not real problems)
	//
	// Exception: skip convergence checks when VQF is experiencing sustained
	// magnetic disturbance.  The CV value is frozen during disturbance (norm
	// tracking gated by !magDistDetected in sensor.c), so a low frozen CV
	// does NOT mean the calibration is still good — the environment may have
	// changed.  If the code reached here past the CV < 4% disturbance gate
	// above, the disturbance is sustained and recalibration should proceed.
#if CONFIG_SENSOR_USE_VQF
	bool vqf_sustained_dist = (online_vqf_dist_start_time > 0 &&
	                           (now - online_vqf_dist_start_time) > ONLINE_VQF_DIST_MIN_DURATION_MS);
#else
	bool vqf_sustained_dist = false;
#endif
	if (has_existing && current_cv < CAL_NORM_GOOD_CV && online_update_count >= ONLINE_MIN_UPDATES
	    && !vqf_sustained_dist) {
		// Tier 2: Excellent fit + sufficient history — lock it in.
		// CV < 0.035 and 3+ updates mean the calibration has reliably
		// converged.  Further updates would only add noise.
		if (current_cv < 0.035f && online_update_count >= 3) {
			LOG_INF("Online mag cal: converged (cv=%.3f, %d updates)",
			        (double)current_cv, online_update_count);
			return false;
		}

		float dir_bias_check = magneto_online_recent_dir_bias();

		// Tier 1: Good fit with adequate directional coverage
		if (dir_bias_check < 0.10f) {
			LOG_INF("Online mag cal: skipping (cv=%.3f < %.3f, dir_bias=%.3f, %d updates)",
			        (double)current_cv, (double)CAL_NORM_GOOD_CV,
			        (double)dir_bias_check, online_update_count);
			return false;
		}
		// Directional bias still too high: buffer samples are clustered.
		// Fall through to run calibration even though CV looks good.
		LOG_INF("Online mag cal: CV ok but dir_bias=%.3f >= 0.10, continuing",
		        (double)dir_bias_check);
	}

	double ata_recent[100];
	double recent_norm_sum;
	float recent_dir_sum[3];
	float recent_raw_range;
	double recent_sample_count = magneto_online_collect_recent(ata_recent, &recent_norm_sum,
	                                                           recent_dir_sum, &recent_raw_range);
	if (recent_sample_count < MAG_CAL_MIN_SAMPLES) {
		return false;
	}
	if (recent_raw_range < MAG_CAL_MIN_RAW_AXIS_RANGE) {
		LOG_INF("Online mag cal: need more rotation (raw_range=%.3f < %.3f, %d recent samples)",
		        (double)recent_raw_range, (double)MAG_CAL_MIN_RAW_AXIS_RANGE,
		        (int)recent_sample_count);
		return false;
	}

	// Detect magnetic environment changes by comparing the buffer's
	// average raw field strength against the last update's reference.
	// When the norm changes by >25% (e.g., moving between a desk and
	// a high-interference area), clear buffers to prevent mixed-data
	// fits.  Direction is preserved — only scale changes.
	float buf_avg_norm = (float)(recent_norm_sum / recent_sample_count);

	if (has_existing && online_update_count > 0 && online_last_buf_avg_norm > 0.0f) {
		float norm_ratio = buf_avg_norm / online_last_buf_avg_norm;
		if (norm_ratio > 1.25f || norm_ratio < 0.80f) {
			LOG_WRN("Online mag cal: env change detected (buf norm %.3f -> %.3f, ratio %.2f), "
			        "resetting buffers",
			        (double)online_last_buf_avg_norm, (double)buf_avg_norm, (double)norm_ratio);
			magneto_online_clear_history();
			online_update_count = 0;
			online_last_buf_avg_norm = 0.0f;
			return false;
		}
	}

	float dbias = magneto_directional_bias(recent_dir_sum, recent_sample_count);
	LOG_INF("Online mag cal: coverage raw_range=%.3f, dir_bias=%.3f, n=%d",
	        (double)recent_raw_range, (double)dbias, (int)recent_sample_count);

	// Quality check: directional diversity + validation + compute calibration
	float m_inv[4][3];
	if (!magneto_quality_check(ata_recent, recent_norm_sum, recent_sample_count, m_inv)) {
		LOG_INF("Online mag cal: check failed (%d recent samples, dir_bias=%.3f)",
		        (int)recent_sample_count, (double)dbias);
		return false;
	}

	if (has_existing) {
		// Enforce minimum cooldown between updates to avoid frequent VQF mag ref resets.
		// Each update resets VQF's heading reference, causing ~6s of re-establishment.
		if (online_last_update_time > 0 &&
		    (now - online_last_update_time) < (ONLINE_MIN_UPDATE_INTERVAL_S * 1000LL)) {
			return false;
		}

		// Blend trial calibration with existing using EMA.
		// Blending weight is similarity-adaptive: more similar → conservative,
		// more divergent → faster adaptation (possible environment change).
		float blended[4][3];
		if (!magneto_blend_BAinv(blended, magBAinv, m_inv)) {
			LOG_WRN("Online mag cal: blend validation failed, skipping update");
			return false;
		}

		float similarity = magneto_BAinv_similarity(magBAinv, m_inv);

		// Reject candidate if similarity is below threshold — the data
		// is too inconsistent for a meaningful fit.  This guards against
		// mixed-data fits (e.g. when the tracker moves between magnetic
		// environments and the quadrant buffer holds samples from both old
		// and new locations).
		// Rather than blindly trusting a poor fit, let the buffer age out
		// stale samples; the next cycle will fit a consistent dataset with
		// much higher similarity.
		//
		// Exception: during the first ONLINE_MIN_UPDATES cycles we accept
		// even low-sim fits to establish an initial baseline (especially
		// important when booting with a stale NVS calibration).
		if (similarity < 0.85f) {
			if (online_update_count < ONLINE_MIN_UPDATES) {
				LOG_INF("Online mag cal: low sim=%.3f accepted (early bootstrap #%d)",
				        (double)similarity, online_update_count + 1);
			} else if (current_cv < CAL_NORM_GOOD_CV) {
				LOG_WRN("Online mag cal: rejecting candidate (sim=%.3f < 0.85, "
				        "current cv=%.3f is good — possible mixed data)",
				        (double)similarity, (double)current_cv);
				return false;
			} else {
				LOG_WRN("Online mag cal: rejecting candidate (sim=%.3f < 0.85, "
				        "current cv=%.3f — incomplete/dirty buffer?)",
				        (double)similarity, (double)current_cv);
				return false;
			}
		}

		LOG_INF("Online mag cal: blended (#%d, %d samples, dir_bias=%.3f, cur_cv=%.3f, sim=%.3f)",
		        online_update_count + 1,
		        (int)recent_sample_count, (double)dbias, (double)current_cv,
		        (double)similarity);
		memcpy(magBAinv, blended, sizeof(magBAinv));
		memcpy(m_inv, blended, sizeof(m_inv)); // for logging below

		// Start the next cycle from a clean buffer. Keeping pre-update samples
		// around can immediately re-mix old field conditions into the next fit.
		magneto_online_clear_history();
		online_last_update_time = now;
		online_update_count++;
		online_last_buf_avg_norm = buf_avg_norm;

		// Reset VQF mag reference so it re-establishes with the refined calibration
#if CONFIG_SENSOR_USE_VQF
		vqf_reset_mag_ref();
		sensor_mag_ref_reset();
#endif
	} else {
		LOG_INF("Online mag cal: first calibration (%d recent samples, dir_bias=%.3f)",
		        (int)recent_sample_count, (double)dbias);

		// First calibration: use candidate directly
		memcpy(magBAinv, m_inv, sizeof(magBAinv));
		magneto_online_clear_history();
		online_last_update_time = now;
		online_update_count = 1;
		online_last_buf_avg_norm = buf_avg_norm;

		// Reset VQF mag reference so it re-establishes with the new calibration
#if CONFIG_SENSOR_USE_VQF
		vqf_reset_mag_ref();
		sensor_mag_ref_reset();
#endif
	}

	// Reset norm tracking after calibration change
	cal_norm_count = 0;
	cal_norm_ema = 0;
	cal_norm_var_ema = 0;

	sys_write(MAIN_MAG_BIAS_ID, &retained->magBAinv, magBAinv, sizeof(magBAinv));
	sensor_refresh_sensor_ids();

	LOG_INF("Online mag cal applied:");
	for (int i = 0; i < 3; i++) {
		LOG_INF("%.5f %.5f %.5f %.5f",
			(double)m_inv[0][i], (double)m_inv[1][i],
			(double)m_inv[2][i], (double)m_inv[3][i]);
	}

	return true;
}

int sensor_calibration_online_mag_status(float *dir_bias)
{
	if (dir_bias) {
		*dir_bias = magneto_online_recent_dir_bias();
	}
	return magneto_online_recent_sample_count();
}

// Feed calibrated mag norm for runtime quality tracking.
// Called from sensor.c after applying BAinv calibration.
void sensor_calibration_track_mag_norm(float cal_norm)
{
	if (cal_norm < 1e-6f) {
		return;
	}
	if (cal_norm_count == 0) {
		cal_norm_ema = cal_norm;
		cal_norm_var_ema = 0;
	} else {
		float diff = cal_norm - cal_norm_ema;
		cal_norm_ema += CAL_NORM_EMA_ALPHA * diff;
		cal_norm_var_ema += CAL_NORM_EMA_ALPHA * (diff * diff - cal_norm_var_ema);
	}
	cal_norm_count++;
}

// Get current calibration quality: returns norm CV (std/mean).
// Lower is better. Returns 1.0 if insufficient data.
float sensor_calibration_get_mag_quality(void)
{
	if (cal_norm_count < 100 || cal_norm_ema < 1e-6f) {
		return 1.0f;
	}
	float std = sqrtf(cal_norm_var_ema);
	return std / cal_norm_ema;
}

static int sensor_calibration_request(int id)
{
	int result;

	k_mutex_lock(&calibration_request_lock, K_FOREVER);
	switch (id) {
	case -1:
		requested_calibration = 0;
		result = 0;
		break;
	case 0:
		result = requested_calibration;
		break;
	default:
		if (requested_calibration != 0) {
			LOG_ERR("Sensor calibration is already running");
			result = -1;
		} else {
			requested_calibration = id;
			result = 0;
		}
		break;
	}
	k_mutex_unlock(&calibration_request_lock);
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
		if (!sensor_ready &&
		    k_uptime_get() - init_wait_start >= CALIBRATION_SENSOR_INIT_WAIT_MS) {
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

		// Phase 2: Background online magnetometer calibration check
		if (requested == 0) {
			sensor_calibration_online_mag_check();
		}

		/* Feed watchdog at end of each loop iteration */
		watchdog_feed(WDT_CHANNEL_CALIBRATION);

		if (requested < 0) {
			k_msleep(5);
		} else if (requested > 0) {
			k_msleep(20);  // Mag cal in progress - short sleep for fast sampling
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
	set_status(SYS_STATUS_CALIBRATION_RUNNING, enabled);
	if (!enabled) {
		tcal_accum_reset();
	}
	LOG_INF("T-Cal Auto-calibration %s", enabled ? "enabled" : "disabled");
}

// Get auto-calibration enabled status
bool sensor_tcal_get_auto_calibration(void)
{
	return tcal_auto_calibration_enabled;
}

// =============================================================================
// Continuous Accumulator Implementation
// =============================================================================

static void tcal_accum_reset(void)
{
	memset(&tcal_accum, 0, sizeof(tcal_accum));
	tcal_accum.temp_min = INFINITY;
	tcal_accum.temp_max = -INFINITY;
}

/**
 * Save a calibration point with hysteresis-aware blending.
 * Saves at the bucket center temperature to prevent boundary drift.
 */
static void tcal_save_point(int idx, const float bias[3])
{
	if (idx < 0 || idx >= TCAL_BUFFER_SIZE) {
		LOG_WRN("T-Cal: Index %d out of range, skipping", idx);
		return;
	}

	// Use the bucket center temperature as the point temperature
	float bucket_temp = IDX_TO_TEMP(idx) + (0.5f / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE);

	// Update temperature direction tracking
	if (!isnan(tcal_direction_ref_temp)) {
		float delta = bucket_temp - tcal_direction_ref_temp;
		if (delta > 0.2f) {
			tcal_current_direction = TCAL_DIR_RISING;
		} else if (delta < -0.2f) {
			tcal_current_direction = TCAL_DIR_FALLING;
		}
	}
	tcal_direction_ref_temp = bucket_temp;

	float final_bias[3];
	memcpy(final_bias, bias, sizeof(final_bias));

	// Hysteresis-aware blending when overwriting existing point.
	// Use tcal_current_direction directly — inferring direction from bucket center
	// temperatures is unreliable since same-bucket overwrites always have equal temps.
	bool is_new_point = (retained->tempCalPoints[idx].temp == 0.0f);
	if (!is_new_point) {
		// Select EMA alpha based on approach direction, preferring rising-phase data
		float ema_alpha;
		switch (tcal_current_direction) {
		case TCAL_DIR_RISING:  ema_alpha = TCAL_HYSTERESIS_EMA_RISING;  break;
		case TCAL_DIR_FALLING: ema_alpha = TCAL_HYSTERESIS_EMA_FALLING; break;
		default:               ema_alpha = TCAL_HYSTERESIS_EMA_UNKNOWN; break;
		}
		LOG_INF("T-Cal: Blending at idx %d (dir: %s, alpha: %.2f)",
		        idx,
		        tcal_current_direction == TCAL_DIR_RISING ? "rising" :
		        tcal_current_direction == TCAL_DIR_FALLING ? "falling" : "unknown",
		        (double)ema_alpha);
		for (int axis = 0; axis < 3; axis++) {
			final_bias[axis] = ema_alpha * final_bias[axis] +
			                   (1.0f - ema_alpha) * retained->tempCalPoints[idx].bias[axis];
		}
		LOG_INF("T-Cal: Blended bias: %.5f %.5f %.5f",
		        (double)final_bias[0], (double)final_bias[1], (double)final_bias[2]);
	} else {
		retained->tempCalState.count++;
	}

	// Check if the change is significant enough to warrant a flash write.
	// New points always write; existing points only write if bias changed meaningfully.
	float max_delta = 0.0f;
	if (!is_new_point) {
		for (int axis = 0; axis < 3; axis++) {
			float d = fabsf(final_bias[axis] - retained->tempCalPoints[idx].bias[axis]);
			if (d > max_delta) {
				max_delta = d;
			}
		}
	}

	retained->tempCalPoints[idx].temp = bucket_temp;
	memcpy(retained->tempCalPoints[idx].bias, final_bias, sizeof(float) * 3);
	retained->tempCalState.valid = false;

	LOG_INF("T-Cal: Committed point at idx %d (%.2fC): [%.5f, %.5f, %.5f] (delta: %.4f dps)",
	        idx, (double)bucket_temp,
	        (double)final_bias[0], (double)final_bias[1], (double)final_bias[2],
	        (double)max_delta);

	if (is_new_point || max_delta >= TCAL_SAVE_SIGNIFICANCE_THRESHOLD) {
		// Significant change or first write: persist to NVS and rebuild LUT
		update_tcal_state();
	} else {
		// Minor update: refresh LUT cache so runtime lookup sees the new value
		// without paying the cost of a full NVS write cycle
		LOG_DBG("T-Cal: Change below threshold (%.4f < %.4f), skipping NVS write",
		        (double)max_delta, (double)TCAL_SAVE_SIGNIFICANCE_THRESHOLD);
		sensor_tcal_cache_invalidate();
	}
}

/**
 * Flush the accumulator: compute average bias/temperature, save to the
 * appropriate temperature bucket.
 */
static void tcal_accum_flush(void)
{
	if (!tcal_accum.active || tcal_accum.sample_count < TCAL_ACCUM_MIN_SAMPLES) {
		return;
	}

	// Compute averages
	float avg_bias[3];
	for (int axis = 0; axis < 3; axis++) {
		avg_bias[axis] = (float)(tcal_accum.gyro_sum[axis] / tcal_accum.sample_count);
	}

	float avg_temp = (tcal_accum.temp_count > 0)
	                 ? (float)(tcal_accum.temp_sum / tcal_accum.temp_count)
	                 : NAN;

	if (isnan(avg_temp)) {
		LOG_WRN("T-Cal: No valid temperature samples in accumulator, discarding");
		tcal_accum_reset();
		return;
	}

	// Map average temperature to bucket index
	int idx = TEMP_TO_IDX(avg_temp);
	if (idx < 0 || idx >= TCAL_BUFFER_SIZE) {
		LOG_WRN("T-Cal: Average temperature %.2fC outside calibration range, discarding",
		        (double)avg_temp);
		tcal_accum_reset();
		return;
	}

	LOG_INF("T-Cal: Flushing accumulator: %d samples, avg temp %.2fC (range %.2fC), bucket idx %d",
	        tcal_accum.sample_count, (double)avg_temp,
	        (double)(tcal_accum.temp_max - tcal_accum.temp_min), idx);

	// Update gyroTemp in retained
	sys_write(MAIN_GYRO_TEMP_ID, &retained->gyroTemp, &avg_temp, sizeof(avg_temp));

	// Save to the bucket
	tcal_save_point(idx, avg_bias);
	tcal_accum_last_commit_time = k_uptime_get();

	// Reset accumulator for next collection period
	tcal_accum_reset();
}

/**
 * Feed a gyro sample into the continuous accumulator.
 * Called from sensor_calibration_process_gyro for every raw gyro sample.
 *
 * The accumulator collects data continuously. Periodically it is flushed
 * (by time or by temperature drift) to save the averaged result.
 *
 * @param g Raw gyro reading (before bias subtraction)
 * @param temp Current IMU temperature
 */
void sensor_tcal_feed_continuous_sample(const float g[3], float temp)
{
	if (!tcal_auto_calibration_enabled) {
		return;
	}

	// Validate temperature
	if (isnan(temp) || temp < (float)CONFIG_SENSOR_POLY_TEMP_MIN ||
	    temp > (float)CONFIG_SENSOR_POLY_TEMP_MAX) {
		return;
	}

	// Initialize accumulator on first sample
	if (!tcal_accum.active) {
		tcal_accum_reset();
		tcal_accum.active = true;
		tcal_accum.start_time = k_uptime_get();
		tcal_accum.min_g[0] = g[0]; tcal_accum.min_g[1] = g[1]; tcal_accum.min_g[2] = g[2];
		tcal_accum.max_g[0] = g[0]; tcal_accum.max_g[1] = g[1]; tcal_accum.max_g[2] = g[2];
		tcal_accum.temp_min = temp;
		tcal_accum.temp_max = temp;
	}

	// Motion detection: check gyro range
	for (int j = 0; j < 3; j++) {
		if (g[j] < tcal_accum.min_g[j]) tcal_accum.min_g[j] = g[j];
		if (g[j] > tcal_accum.max_g[j]) tcal_accum.max_g[j] = g[j];
		if (tcal_accum.max_g[j] - tcal_accum.min_g[j] > TCAL_ACCUM_GYRO_RANGE_THRESHOLD) {
			// Motion detected — discard accumulated data
			LOG_DBG("T-Cal: Motion in accumulator, axis %d (range: %.3f dps), resetting",
			        j, (double)(tcal_accum.max_g[j] - tcal_accum.min_g[j]));
			tcal_accum_reset();
			return;
		}
	}

	// Accumulate gyro
	tcal_accum.gyro_sum[0] += (double)g[0];
	tcal_accum.gyro_sum[1] += (double)g[1];
	tcal_accum.gyro_sum[2] += (double)g[2];
	tcal_accum.sample_count++;

	// Accumulate temperature
	if (temp < tcal_accum.temp_min) tcal_accum.temp_min = temp;
	if (temp > tcal_accum.temp_max) tcal_accum.temp_max = temp;
	tcal_accum.temp_sum += (double)temp;
	tcal_accum.temp_count++;

	// Check flush conditions
	int64_t elapsed = k_uptime_get() - tcal_accum.start_time;
	float temp_drift = tcal_accum.temp_max - tcal_accum.temp_min;

	// Condition 1: Temperature drifted beyond one bucket width — flush early
	// to avoid cross-bucket contamination, then start a new accumulation window
	if (temp_drift > TCAL_ACCUM_TEMP_DRIFT_MAX &&
	    tcal_accum.sample_count >= TCAL_ACCUM_MIN_SAMPLES) {
		LOG_INF("T-Cal: Temperature drift %.2fC exceeded threshold, early flush",
		        (double)temp_drift);
		tcal_accum_flush();
		return;
	}

	// Condition 2: Flush interval reached
	if (elapsed >= TCAL_ACCUM_FLUSH_INTERVAL_MS &&
	    tcal_accum.sample_count >= TCAL_ACCUM_MIN_SAMPLES) {
		tcal_accum_flush();
		return;
	}
}

/**
 * Called when motion is detected — flush if enough data, then reset.
 */
void sensor_tcal_continuous_motion_detected(void)
{
	if (tcal_accum.active) {
		if (tcal_accum.sample_count >= TCAL_ACCUM_MIN_SAMPLES) {
			tcal_accum_flush();
		} else {
			tcal_accum_reset();
		}
	}
}

// Check and request auto calibration if conditions are met.
// With continuous accumulator sampling, this is only used as a fallback
// to trigger initial calibration when no T-Cal data exists at all.
void sensor_tcal_check_auto_calibration(float current_temp)
{
	static int64_t last_calibration_time = 0;

	int64_t now = k_uptime_get();

	if (!tcal_auto_calibration_enabled) {
		return;
	}

	// Continuous accumulator handles the normal case.
	// This fallback only triggers initial manual calibration when there
	// are zero temperature calibration points (device first use).
	if (retained->tempCalState.count > 0) {
		return;
	}

	const int64_t calibration_cooldown_ms = BIAS_COLLECT_MAX_SAMPLE_TIME_MS + 10000;
	if ((now - last_calibration_time) < calibration_cooldown_ms) {
		return;
	}

	if (isnan(current_temp) || current_temp < -20.0f || current_temp > 60.0f) {
		return;
	}

	LOG_INF("T-Cal Auto: No calibration data exists, requesting initial calibration at %.2fC",
	        (double)current_temp);

	int request_result = sensor_calibration_request(1);
	if (request_result == 0) {
		last_calibration_time = now;
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
	if (isnan(current_temp) || current_temp < -20.0f || current_temp > 60.0f) {
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
	if (isnan(current_temp) || current_temp < -20.0f || current_temp > 60.0f) {
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

// Enable/disable T-Cal compensation (persisted via NVS)
void sensor_tcal_set_enabled(bool enabled)
{
	if (tcal_compensation_enabled == enabled) {
		LOG_INF("T-Cal compensation already %s", enabled ? "enabled" : "disabled");
		return;
	}
	tcal_compensation_enabled = enabled;
	bool val = enabled;
	sys_write(TCAL_ENABLED_ID, &retained->tcal_enabled, &val, sizeof(val));
	LOG_INF("T-Cal compensation %s (persisted)", enabled ? "enabled" : "disabled");
}

bool sensor_tcal_get_enabled(void)
{
	return tcal_compensation_enabled;
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
	if (isnan(current_temp) || current_temp < -20.0f || current_temp > 60.0f) {
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

	// Online top-k selection: maintain only the best MLS_MAX_POINTS by weight
	// This avoids large stack allocations while scanning all buffer entries
	WeightedPoint points[MLS_MAX_POINTS];
	int point_count = 0;
	int total_valid = 0;  // total points passing weight filter

	float bandwidth_sq = MLS_BANDWIDTH * MLS_BANDWIDTH;
	float min_selected_weight = 0.0f;  // track minimum weight in selected set
	int min_selected_idx = 0;

	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (retained->tempCalPoints[i].temp == 0.0f) {
			continue; // Skip empty slots
		}

		float point_temp = retained->tempCalPoints[i].temp;
		float d = point_temp - temp;
		float d_sq = d * d;

		// Cauchy-like weight: w = 1 / (1 + (d/h)²)
		float weight = 1.0f / (1.0f + d_sq / bandwidth_sq);

		// Skip points with negligible weight
		if (weight < MLS_MIN_WEIGHT) {
			continue;
		}

		total_valid++;

		if (point_count < MLS_MAX_POINTS) {
			// Still filling the selection buffer
			points[point_count].temp = point_temp;
			points[point_count].weight = weight;
			memcpy(points[point_count].bias, retained->tempCalPoints[i].bias, sizeof(float) * 3);
			point_count++;

			// Update minimum tracking when buffer is full
			if (point_count == MLS_MAX_POINTS) {
				min_selected_weight = points[0].weight;
				min_selected_idx = 0;
				for (int j = 1; j < MLS_MAX_POINTS; j++) {
					if (points[j].weight < min_selected_weight) {
						min_selected_weight = points[j].weight;
						min_selected_idx = j;
					}
				}
			}
		} else if (weight > min_selected_weight) {
			// Replace the weakest point in our selection
			points[min_selected_idx].temp = point_temp;
			points[min_selected_idx].weight = weight;
			memcpy(points[min_selected_idx].bias, retained->tempCalPoints[i].bias, sizeof(float) * 3);

			// Find new minimum
			min_selected_weight = points[0].weight;
			min_selected_idx = 0;
			for (int j = 1; j < MLS_MAX_POINTS; j++) {
				if (points[j].weight < min_selected_weight) {
					min_selected_weight = points[j].weight;
					min_selected_idx = j;
				}
			}
		}
	}

	if (point_count == 0) {
		// No points with sufficient weight - temperature is far outside calibrated range
		// Use linear extrapolation from edge points (similar to LUT extrapolation logic)

		// Collect all valid points with their temperatures
		typedef struct {
			float temp;
			float bias[3];
		} TempPoint;
		TempPoint edge_points[MLS_EXTRAP_POINTS];
		int edge_count = 0;

		// Determine if we're extrapolating low or high
		// First scan to find data range
		float data_min_temp = 1000.0f;
		float data_max_temp = -1000.0f;

		for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
			if (retained->tempCalPoints[i].temp == 0.0f) {
				continue;
			}
			float pt = retained->tempCalPoints[i].temp;
			if (pt < data_min_temp) data_min_temp = pt;
			if (pt > data_max_temp) data_max_temp = pt;
		}

		if (data_min_temp > 999.0f) {
			// No calibration data at all
			LOG_ERR("T-Cal MLS: No calibration data available");
			return -1;
		}

		bool extrapolate_low = (temp < data_min_temp);

		if (extrapolate_low) {
			// Collect lowest temperature points for low extrapolation
			// Sort by temperature ascending, take first MLS_EXTRAP_POINTS
			for (int k = 0; k < MLS_EXTRAP_POINTS && k < retained->tempCalState.count; k++) {
				float lowest_temp = 1000.0f;
				int lowest_idx = -1;

				for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
					if (retained->tempCalPoints[i].temp == 0.0f) continue;
					float pt = retained->tempCalPoints[i].temp;

					// Check if already selected
					bool already_selected = false;
					for (int j = 0; j < edge_count; j++) {
						if (fabsf(edge_points[j].temp - pt) < 0.01f) {
							already_selected = true;
							break;
						}
					}
					if (already_selected) continue;

					if (pt < lowest_temp) {
						lowest_temp = pt;
						lowest_idx = i;
					}
				}

				if (lowest_idx >= 0) {
					edge_points[edge_count].temp = retained->tempCalPoints[lowest_idx].temp;
					memcpy(edge_points[edge_count].bias, retained->tempCalPoints[lowest_idx].bias, sizeof(float) * 3);
					edge_count++;
				}
			}
		} else {
			// Collect highest temperature points for high extrapolation
			// Sort by temperature descending, take first MLS_EXTRAP_POINTS
			for (int k = 0; k < MLS_EXTRAP_POINTS && k < retained->tempCalState.count; k++) {
				float highest_temp = -1000.0f;
				int highest_idx = -1;

				for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
					if (retained->tempCalPoints[i].temp == 0.0f) continue;
					float pt = retained->tempCalPoints[i].temp;

					// Check if already selected
					bool already_selected = false;
					for (int j = 0; j < edge_count; j++) {
						if (fabsf(edge_points[j].temp - pt) < 0.01f) {
							already_selected = true;
							break;
						}
					}
					if (already_selected) continue;

					if (pt > highest_temp) {
						highest_temp = pt;
						highest_idx = i;
					}
				}

				if (highest_idx >= 0) {
					edge_points[edge_count].temp = retained->tempCalPoints[highest_idx].temp;
					memcpy(edge_points[edge_count].bias, retained->tempCalPoints[highest_idx].bias, sizeof(float) * 3);
					edge_count++;
				}
			}
		}

		if (edge_count == 0) {
			LOG_ERR("T-Cal MLS: No edge points found for extrapolation");
			return -1;
		}

		// Single point: constant extrapolation
		if (edge_count == 1) {
			memcpy(bias_out, edge_points[0].bias, sizeof(float) * 3);

			int slot = sensor_tcal_cache_select_slot(temp);
			mls_cache.slots[slot].temp = temp;
			memcpy(mls_cache.slots[slot].bias, bias_out, sizeof(float) * 3);
			memset(mls_cache.slots[slot].slope, 0, sizeof(mls_cache.slots[slot].slope));
			mls_cache.slots[slot].valid = true;

			LOG_DBG("T-Cal MLS: Single-point extrapolation at %.2fC", (double)temp);
			return 0;
		}

		// Multiple points: linear least squares fit (same as LUT extrapolation)
		float t_mean = 0.0f;
		for (int i = 0; i < edge_count; i++) {
			t_mean += edge_points[i].temp;
		}
		t_mean /= edge_count;

		float sum_dt_sq = 0.0f;
		for (int i = 0; i < edge_count; i++) {
			float dt = edge_points[i].temp - t_mean;
			sum_dt_sq += dt * dt;
		}

		float slope[3] = {0.0f, 0.0f, 0.0f};

		for (int axis = 0; axis < 3; axis++) {
			float b_mean = 0.0f;
			for (int i = 0; i < edge_count; i++) {
				b_mean += edge_points[i].bias[axis];
			}
			b_mean /= edge_count;

			float sum_dt_db = 0.0f;
			for (int i = 0; i < edge_count; i++) {
				float dt = edge_points[i].temp - t_mean;
				float db = edge_points[i].bias[axis] - b_mean;
				sum_dt_db += dt * db;
			}

			slope[axis] = (sum_dt_sq > 0.001f) ? sum_dt_db / sum_dt_sq : 0.0f;
			bias_out[axis] = b_mean + slope[axis] * (temp - t_mean);
		}

		// Cache the extrapolated result with slope for smooth interpolation
		int slot = sensor_tcal_cache_select_slot(temp);
		mls_cache.slots[slot].temp = temp;
		memcpy(mls_cache.slots[slot].bias, bias_out, sizeof(float) * 3);
		memcpy(mls_cache.slots[slot].slope, slope, sizeof(float) * 3);
		mls_cache.slots[slot].valid = true;

		LOG_DBG("T-Cal MLS: Linear extrapolation %s range (%.2fC, %d pts)",
			extrapolate_low ? "below" : "above", (double)temp, edge_count);
		return 0;
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
