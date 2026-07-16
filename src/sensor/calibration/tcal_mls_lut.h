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
#ifndef SLIMENRF_CAL_TCAL_MLS_LUT_H
#define SLIMENRF_CAL_TCAL_MLS_LUT_H

#include <stdbool.h>

#if CONFIG_SENSOR_USE_TCAL

/* MLS parameters also needed by quality assessment / status in calibration.c */
#define MLS_BANDWIDTH 2.5f
#define MLS_MIN_WEIGHT 0.03f
#define MLS_MIN_POINTS_FOR_FIT 4

#define MLS_LUT_STEP_PER_DEGREE 2
#define MLS_LUT_SIZE ((int)((CONFIG_SENSOR_POLY_TEMP_MAX - CONFIG_SENSOR_POLY_TEMP_MIN) * MLS_LUT_STEP_PER_DEGREE) + 1)

typedef enum {
	MLS_LUT_BUILD_IDLE,
	MLS_LUT_BUILD_PRIORITY,
	MLS_LUT_BUILD_BACKGROUND,
	MLS_LUT_BUILD_COMPLETE
} MlsLutBuildState;

int sensor_tcal_mls_lookup(float temp, float bias_out[3]);
int sensor_tcal_lut_lookup(float temp, float bias_out[3]);
void sensor_tcal_build_lut_priority(float current_temp);
bool sensor_tcal_build_lut_continue(void);
void sensor_tcal_cache_invalidate(void);

MlsLutBuildState sensor_tcal_lut_get_build_state(void);
bool sensor_tcal_lut_is_valid(void);
int sensor_tcal_lut_get_computed_count(void);

#endif /* CONFIG_SENSOR_USE_TCAL */

#endif /* SLIMENRF_CAL_TCAL_MLS_LUT_H */
