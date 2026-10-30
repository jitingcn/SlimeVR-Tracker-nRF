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
#include "sensor/sensor.h"
#include "util.h"

#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>

#if CONFIG_CMSIS_DSP
#include <arm_math.h>
#endif

#include "tcal_mls_lut.h"

#if CONFIG_SENSOR_USE_TCAL

LOG_MODULE_REGISTER(cal_tcal_mls_lut, LOG_LEVEL_INF);

static K_MUTEX_DEFINE(tcal_mutex);
static uint32_t model_generation;
static float build_temperature;
static bool build_temperature_valid;

void sensor_tcal_lock(void)
{
	k_mutex_lock(&tcal_mutex, K_FOREVER);
}

void sensor_tcal_unlock(void)
{
	k_mutex_unlock(&tcal_mutex);
}

uint32_t sensor_tcal_model_generation(void)
{
	sensor_tcal_lock();
	uint32_t generation = model_generation;
	sensor_tcal_unlock();
	return generation;
}

// =============================================================================
// T-Cal Moving Least Squares (MLS) Implementation
// =============================================================================
// MLS provides smooth, continuous bias estimation without discontinuities
// caused by method switching. Uses inverse distance weighting with local
// linear fitting for optimal balance of smoothness and responsiveness.

// MLS Configuration
#define MLS_MAX_POINTS 10        // Maximum points to consider for efficiency

// =============================================================================
// Exact MLS Memoization
// =============================================================================
// Cache only the result at the exact requested temperature and model generation.
// Reusing a nearby fit would make canonical grid values depend on query history.

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
// - Schedule entries within ±2°C of current temperature first (priority zone)
// - Scheduling does no fitting on the caller's gyro path
// - Continue building remaining entries in small batches during idle time
// - Canonical lookup computes at most two missing grid entries on demand

#define MLS_LUT_STEP_SIZE (1.0f / MLS_LUT_STEP_PER_DEGREE)  // 0.5°C
#define MLS_LUT_TEMP_MIN ((float)CONFIG_SENSOR_POLY_TEMP_MIN)
#define MLS_LUT_TEMP_MAX ((float)CONFIG_SENSOR_POLY_TEMP_MAX)

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

static struct {
	MlsLutEntry entries[MLS_LUT_SIZE]; // Pre-computed bias values
	uint32_t version;                   // Model generation represented by the LUT
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

static bool sensor_tcal_lut_compute_entry(int idx);

// =============================================================================
// Exact-temperature MLS cache (shared by direct queries and grid construction)
// =============================================================================

#define MLS_CACHE_SLOTS 5

typedef struct {
	float temp;        // Temperature at which cache was computed
	float bias[3];     // Cached bias values at temp
	bool valid;        // Cache validity flag
} MlsCacheSlot;

static struct {
	MlsCacheSlot slots[MLS_CACHE_SLOTS]; // Cache slots covering temperature range
	uint32_t generation;                // Model generation represented by cached values
} mls_cache = {
	.slots = {{0}},
	.generation = 0
};

/**
 * Select the best cache slot to use for a new entry at the given temperature.
 * Strategy:
 * 1. If an invalid slot exists, use it
 * 2. Find the slot with the largest distance from the query temperature
 *    (this preserves nearby temperatures likely to be queried again)
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
void sensor_tcal_cache_invalidate(void)
{
	sensor_tcal_lock();
	// Invalidate exact-temperature cache slots
	for (int i = 0; i < MLS_CACHE_SLOTS; i++) {
		mls_cache.slots[i].valid = false;
	}
	// Invalidate LUT and stop any incremental build in progress
	mls_lut.version = model_generation;
	mls_lut.valid = false;
	mls_lut.build_state = MLS_LUT_BUILD_IDLE;
	mls_lut.computed_count = 0;
	// Mark all entries as not computed
	for (int i = 0; i < MLS_LUT_SIZE; i++) {
		mls_lut.entries[i].computed = false;
	}
	LOG_DBG("T-Cal cache/LUT invalidated, incremental build stopped");
	sensor_tcal_unlock();
}

static void sensor_tcal_schedule_lut_locked(float current_temp);

void sensor_tcal_model_changed(float current_temp)
{
	sensor_tcal_lock();
	model_generation++;
	sensor_tcal_cache_invalidate();
	if (v_finite(&current_temp, 1)) {
		sensor_tcal_schedule_lut_locked(current_temp);
	} else if (build_temperature_valid) {
		sensor_tcal_schedule_lut_locked(build_temperature);
	}
	sensor_tcal_unlock();
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
static int sensor_tcal_mls_lookup_locked(float temp, float bias_out[3])
{
	if (!v_finite(&temp, 1)) {
		return -1;
	}
	// Check if we have any calibration data
	if (retained->tempCalState.count < 1) {
		LOG_ERR("T-Cal MLS: No calibration data available");
		return -1;
	}

	// Check multi-slot cache:
	// Content replacement must invalidate even when the point count is unchanged.
	if (mls_cache.generation != model_generation) {
		// Invalidate all slots
		for (int i = 0; i < MLS_CACHE_SLOTS; i++) {
			mls_cache.slots[i].valid = false;
		}
		mls_cache.generation = model_generation;
	}

	// Search for a matching cache slot
	for (int i = 0; i < MLS_CACHE_SLOTS; i++) {
		if (mls_cache.slots[i].valid && mls_cache.slots[i].temp == temp) {
			memcpy(bias_out, mls_cache.slots[i].bias, sizeof(float) * 3);
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
		/* Outside weighted support: clamp to nearest calibrated point (no unbounded slope). */
		float best_dist = INFINITY;
		int best_idx = -1;
		for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
			if (retained->tempCalPoints[i].temp == 0.0f) {
				continue;
			}
			float d = fabsf(retained->tempCalPoints[i].temp - temp);
			if (d < best_dist) {
				best_dist = d;
				best_idx = i;
			}
		}
		if (best_idx < 0) {
			LOG_ERR("T-Cal MLS: No calibration data available");
			return -1;
		}
		memcpy(bias_out, retained->tempCalPoints[best_idx].bias, sizeof(float) * 3);
		int slot = sensor_tcal_cache_select_slot(temp);
		mls_cache.slots[slot].temp = temp;
		memcpy(mls_cache.slots[slot].bias, bias_out, sizeof(float) * 3);
		mls_cache.slots[slot].valid = true;
		LOG_DBG(
			"T-Cal MLS: Clamped to nearest point %.2fC (query %.2fC)",
			(double)retained->tempCalPoints[best_idx].temp,
			(double)temp
		);
		return 0;
	}

	// If only one point has significant weight, just return it
	if (point_count == 1) {
		memcpy(bias_out, points[0].bias, sizeof(float) * 3);

		// Cache the exact query result
		int slot = sensor_tcal_cache_select_slot(temp);
		mls_cache.slots[slot].temp = temp;
		memcpy(mls_cache.slots[slot].bias, bias_out, sizeof(float) * 3);
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
		mls_cache.slots[slot].valid = true;
		LOG_DBG("T-Cal MLS: Degenerate case, using weighted average at %.2fC (cached in slot %d)", (double)temp, slot);
		return 0;
	}

	// Solve for 'a', the bias at this exact query temperature.
	for (int axis = 0; axis < 3; axis++) {
		double a = (sum_wb[axis] * sum_wdt2 - sum_wdtb[axis] * sum_wdt) / det;
		bias_out[axis] = (float)a;
	}

	// Update cache with computed result (select best slot based on distance)
	int slot = sensor_tcal_cache_select_slot(temp);
	mls_cache.slots[slot].temp = temp;
	memcpy(mls_cache.slots[slot].bias, bias_out, sizeof(float) * 3);
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

int sensor_tcal_mls_lookup(float temp, float bias_out[3])
{
	sensor_tcal_lock();
	int result = sensor_tcal_mls_lookup_locked(temp, bias_out);
	sensor_tcal_unlock();
	return result;
}

// =============================================================================
// LUT Incremental Build Functions
// =============================================================================

/**
 * Compute and publish one LUT entry while holding the model lock.
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

	if (sensor_tcal_mls_lookup_locked(temp, bias) == 0) {
		memcpy(mls_lut.entries[idx].bias, bias, sizeof(float) * 3);
		mls_lut.entries[idx].computed = true;
		mls_lut.computed_count++;
		if (mls_lut.computed_count == MLS_LUT_SIZE) {
			mls_lut.valid = true;
		}
		return true;
	}

	return false;
}

/* Model lock held. Schedule the current ±2C priority zone without fitting.
 * The caller has already invalidated this generation's cache. */
static void sensor_tcal_schedule_lut_locked(float current_temp)
{
	build_temperature = current_temp;
	build_temperature_valid = true;
	if (retained->tempCalState.count < MLS_MIN_POINTS_FOR_FIT) {
		return;
	}

	mls_lut.version = model_generation;
	// Clamp before index conversion, including finite temperatures far outside the grid.
	float center = fmaxf(MLS_LUT_TEMP_MIN, fminf(current_temp, MLS_LUT_TEMP_MAX));
	float low = fmaxf(MLS_LUT_TEMP_MIN, center - MLS_LUT_PRIORITY_RANGE);
	float high = fminf(MLS_LUT_TEMP_MAX, center + MLS_LUT_PRIORITY_RANGE);
	mls_lut.priority_idx_min = (int)MLS_LUT_TEMP_TO_IDX(low);
	mls_lut.priority_idx_max = (int)MLS_LUT_TEMP_TO_IDX(high) + 1;
	if (mls_lut.priority_idx_max >= MLS_LUT_SIZE) {
		mls_lut.priority_idx_max = MLS_LUT_SIZE - 1;
	}
	mls_lut.build_next_idx = mls_lut.priority_idx_min;
	mls_lut.build_state = MLS_LUT_BUILD_PRIORITY;
}

/**
 * Explicitly rebuild priority entries, then the rest of the LUT. No fitting
 * here; callers may request a rebuild directly from the gyro collector.
 */
void sensor_tcal_build_lut_priority(float current_temp)
{
	if (!v_finite(&current_temp, 1)) {
		return;
	}
	sensor_tcal_lock();
	sensor_tcal_cache_invalidate();
	sensor_tcal_schedule_lut_locked(current_temp);
	sensor_tcal_unlock();
}

/**
 * Run a bounded batch from the calibration worker, never with an outer
 * model lock held. Each fit and its publication share one lock acquisition;
 * mutation between entries restarts the build without publishing stale data.
 */
bool sensor_tcal_build_lut_continue(void)
{
	for (int work = 0; work < MLS_LUT_BATCH_SIZE; work++) {
		sensor_tcal_lock();
		if (mls_lut.build_state != MLS_LUT_BUILD_PRIORITY &&
		    mls_lut.build_state != MLS_LUT_BUILD_BACKGROUND) {
			sensor_tcal_unlock();
			return true;
		}
		if (mls_lut.version != model_generation) {
			sensor_tcal_cache_invalidate();
			sensor_tcal_unlock();
			return true;
		}

		if (mls_lut.build_state == MLS_LUT_BUILD_BACKGROUND) {
			while (mls_lut.build_next_idx < MLS_LUT_SIZE &&
			       mls_lut.entries[mls_lut.build_next_idx].computed) {
				mls_lut.build_next_idx++;
			}
		}
		if (mls_lut.build_next_idx >= MLS_LUT_SIZE) {
			mls_lut.build_state = MLS_LUT_BUILD_COMPLETE;
			sensor_tcal_unlock();
			return true;
		}

		if (!sensor_tcal_lut_compute_entry(mls_lut.build_next_idx)) {
			sensor_tcal_cache_invalidate();
			sensor_tcal_unlock();
			return true;
		}
		mls_lut.build_next_idx++;
		if (mls_lut.build_state == MLS_LUT_BUILD_PRIORITY &&
		    mls_lut.build_next_idx > mls_lut.priority_idx_max) {
			mls_lut.valid = true;
			mls_lut.build_state = MLS_LUT_BUILD_BACKGROUND;
			mls_lut.build_next_idx = 0;
		}
		if (mls_lut.computed_count == MLS_LUT_SIZE) {
			mls_lut.build_state = MLS_LUT_BUILD_COMPLETE;
			sensor_tcal_unlock();
			return true;
		}
		sensor_tcal_unlock();
	}

	k_msleep(MLS_LUT_BATCH_YIELD_MS);
	return false;
}

// =============================================================================
// Canonical Grid Lookup
// =============================================================================
static int sensor_tcal_grid_lookup_locked(float temp, float bias_out[3])
{
	if (!v_finite(&temp, 1) || mls_lut.version != model_generation) {
		return -1;  // LUT not available
	}

	/* Outside configured LUT range: clamp to nearest edge entry (no slope run-away). */
	if (temp < MLS_LUT_TEMP_MIN) {
		if (!sensor_tcal_lut_compute_entry(0)) {
			return -1;
		}
		memcpy(bias_out, mls_lut.entries[0].bias, sizeof(float) * 3);
		return 0;
	}
	if (temp > MLS_LUT_TEMP_MAX) {
		int hi = MLS_LUT_SIZE - 1;
		if (!sensor_tcal_lut_compute_entry(hi)) {
			return -1;
		}
		memcpy(bias_out, mls_lut.entries[hi].bias, sizeof(float) * 3);
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

	if (!sensor_tcal_lut_compute_entry(idx_lo) || !sensor_tcal_lut_compute_entry(idx_hi)) {
		return -1;
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

int sensor_tcal_curve_lookup(float temp, float bias_out[3])
{
	sensor_tcal_lock();
	int result = sensor_tcal_grid_lookup_locked(temp, bias_out);
	sensor_tcal_unlock();
	return result;
}

MlsLutBuildState sensor_tcal_lut_get_build_state(void)
{
	sensor_tcal_lock();
	MlsLutBuildState state = mls_lut.build_state;
	sensor_tcal_unlock();
	return state;
}

bool sensor_tcal_lut_is_valid(void)
{
	sensor_tcal_lock();
	bool valid = mls_lut.valid && mls_lut.version == model_generation;
	sensor_tcal_unlock();
	return valid;
}

int sensor_tcal_lut_get_computed_count(void)
{
	sensor_tcal_lock();
	int count = mls_lut.computed_count;
	sensor_tcal_unlock();
	return count;
}


#endif /* CONFIG_SENSOR_USE_TCAL */
