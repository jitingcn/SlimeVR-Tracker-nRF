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
#include "util.h"

#include <math.h>
#include <string.h>

#if CONFIG_CMSIS_DSP
#include <arm_math.h>
#endif

#include "sensor/magneto/magneto1_4.h"

#include "magneto.h"

/**
 * Update a min/max hard-iron center estimate for coverage calculations.
 */
void magneto_center_reset(mag_center_estimator_t *estimator)
{
	memset(estimator, 0, sizeof(*estimator));
}

void magneto_center_update(mag_center_estimator_t *estimator, const float m[3])
{
	if (!estimator->initialized) {
		memcpy(estimator->min, m, sizeof(estimator->min));
		memcpy(estimator->max, m, sizeof(estimator->max));
		estimator->initialized = true;
		return;
	}

	for (int i = 0; i < 3; i++) {
		if (m[i] < estimator->min[i]) {
			estimator->min[i] = m[i];
		}
		if (m[i] > estimator->max[i]) {
			estimator->max[i] = m[i];
		}
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

float magneto_center_min_range(const mag_center_estimator_t *estimator)
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

void magneto_coverage_sample(const mag_center_estimator_t *estimator, const float m[3], float coverage_sample[3])
{
	if (magneto_center_has_coverage(estimator)) {
		magneto_centered_sample(estimator, m, coverage_sample);
	} else {
		memcpy(coverage_sample, m, sizeof(float) * 3);
	}
}

float magneto_norm_sq(const float v[3])
{
	return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

bool magneto_normalize_direction(const float v[3], float dir[3])
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

bool magneto_centered_direction(const mag_center_estimator_t *estimator, const float m[3], float dir[3])
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

bool mag_bainv_structurally_ok(const float m_inv[4][3], float bias_limit)
{
	float zero[3] = {0};
	float diagonal[3];

	for (int i = 0; i < 3; i++) {
		diagonal[i] = m_inv[i + 1][i];
	}

	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	float max_gain = MAX(MAX(fabsf(diagonal[0]), fabsf(diagonal[1])), fabsf(diagonal[2]));
	float limit = bias_limit > 0.0f ? bias_limit : MAX(fabsf(magnitude) * 2.0f, 0.1f);

	return v_epsilon(m_inv[0], zero, limit)
		&& v_epsilon(diagonal, average, MAX(fabsf(magnitude) * 0.2f, 0.1f))
		&& max_gain <= MAG_CAL_MAX_AXIS_GAIN;
}

/**
 * Check if a Magneto calibration result passes quality checks.
 * Performs silent validation (no warning logs) since this is called speculatively.
 * If m_inv_out is non-NULL and check passes, the computed calibration is stored there.
 * Returns true if quality is acceptable.
 */
bool magneto_quality_check(double *ata_buf, double norm_sum_val, double sample_count_val, float m_inv_out[][3])
{
	if (sample_count_val < MAG_CAL_MIN_SAMPLES) {
		return false;
	}

	// Run trial calibration
	float m_inv[4][3];
	magneto_current_calibration(m_inv, ata_buf, norm_sum_val, sample_count_val);

	float hm = (float)(norm_sum_val / sample_count_val);
	if (!mag_bainv_structurally_ok(m_inv, hm * 2.0f)) {
		return false;
	}

	if (m_inv_out) {
		memcpy(m_inv_out, m_inv, sizeof(m_inv));
	}
	return true;
}
