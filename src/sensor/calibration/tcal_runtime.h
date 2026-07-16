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
#ifndef SLIMENRF_CAL_TCAL_RUNTIME_H
#define SLIMENRF_CAL_TCAL_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

#if CONFIG_SENSOR_USE_TCAL

#include "bias_collect.h"
#include "cal_sample.h"

#define TEMP_TO_IDX(temp) (int)((((float)temp) - CONFIG_SENSOR_POLY_TEMP_MIN) * CONFIG_SENSOR_POLY_STEPS_PER_DEGREE)
#define IDX_TO_TEMP(idx) (float)(((float)(idx) / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE) + CONFIG_SENSOR_POLY_TEMP_MIN)

#define BOOT_CAL_TIME_WINDOW_START_MS 5000
#define BOOT_CAL_TIME_WINDOW_END_MS 30000
#define BOOT_CAL_MAX_ATTEMPTS 2
#define BOOT_CAL_MIN_CURVE_POINTS 4

#define RUNTIME_CAL_REST_TIME_MS 8000
#define RUNTIME_CAL_COOLDOWN_MS 60000
#define RUNTIME_CAL_MIN_UPTIME_MS 60000
#define RUNTIME_CAL_TEMP_CHANGE_MIN 1.0f
#define RUNTIME_CAL_SAMPLE_TIME_MS 3000
#define RUNTIME_CAL_FAILURE_COOLDOWN_MS 30000

#define TCAL_HYSTERESIS_EMA_RISING 0.7f
#define TCAL_HYSTERESIS_EMA_FALLING 0.3f
#define TCAL_HYSTERESIS_EMA_UNKNOWN 0.5f

typedef enum {
	TCAL_DIR_UNKNOWN = 0,
	TCAL_DIR_RISING,
	TCAL_DIR_FALLING,
} tcal_temp_direction_t;

/* Shared with calibration.c IMU-cal / clear / status paths */
extern tcal_temp_direction_t tcal_current_direction;
extern float tcal_direction_ref_temp;
extern float runtime_cal_last_temp;

void update_tcal_state(void);
void tcal_accum_reset(void);
void sensor_tcal_runtime_init_from_retained(void);

/* calibration_thread entry points (were file-local) */
int sensor_perform_boot_calibration(void);
int sensor_perform_runtime_calibration(void);

/*
 * Public T-Cal runtime APIs remain declared in calibration.h and are defined
 * in tcal_runtime.c.
 */

/* Request latch owned by calibration.c */
int sensor_calibration_request(int id);

#endif /* CONFIG_SENSOR_USE_TCAL */

#endif /* SLIMENRF_CAL_TCAL_RUNTIME_H */
