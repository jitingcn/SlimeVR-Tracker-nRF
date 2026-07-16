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
#ifndef SLIMENRF_CAL_MAG_COMMON_H
#define SLIMENRF_CAL_MAG_COMMON_H

#include <stdbool.h>
#include <stdint.h>

/* Shared magnetometer calibration constants (manual + online paths). */
#define MAG_CAL_ACCEL_MAG_MIN_SQ 0.75f
#define MAG_CAL_ACCEL_MAG_MAX_SQ 1.3f
#define MAG_CAL_MIN_SAMPLES 192
#define MAG_CAL_TRIAL_INTERVAL (MAG_CAL_MIN_SAMPLES / 2)
#define MAG_CAL_MIN_DIR_RANGE 0.5f
#define MAG_CAL_MIN_RAW_AXIS_RANGE 0.5f
#define MAG_CAL_MAX_AXIS_GAIN 8.0f

typedef struct {
	float min[3];
	float max[3];
	bool initialized;
} mag_center_estimator_t;

/* magBAinv: calibration.c; aBuf: cal_sample.c; magneto_progress: cal_mag.c */
extern float magBAinv[4][3];
extern float aBuf[3];
extern uint8_t magneto_progress;

void magneto_center_reset(mag_center_estimator_t *estimator);
void magneto_center_update(mag_center_estimator_t *estimator, const float m[3]);
float magneto_center_min_range(const mag_center_estimator_t *estimator);
void magneto_coverage_sample(const mag_center_estimator_t *estimator, const float m[3],
			     float coverage_sample[3]);
float magneto_norm_sq(const float v[3]);
bool magneto_normalize_direction(const float v[3], float dir[3]);
bool magneto_centered_direction(const mag_center_estimator_t *estimator, const float m[3],
				float dir[3]);
bool magneto_quality_check(double *ata_buf, double norm_sum_val, double sample_count_val,
			   float m_inv_out[][3]);

#endif /* SLIMENRF_CAL_MAG_COMMON_H */
