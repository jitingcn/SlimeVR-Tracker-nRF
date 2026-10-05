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
#include <stddef.h>

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

/* Rising preferred; falling still accepted but weakly updates the shared slot. */
#define TCAL_HYSTERESIS_EMA_RISING 0.7f
#define TCAL_HYSTERESIS_EMA_FALLING 0.15f
#define TCAL_HYSTERESIS_EMA_UNKNOWN 0.5f

/* Continuous-bucket write gates (quasi-steady only; does not change apply path). */
#define TCAL_ACCUM_ACCEL_MOTION_THRESHOLD BIAS_COLLECT_ACCEL_MOTION_THRESHOLD
#define TCAL_WRITE_DTDT_MAX_C_PER_S 0.04f /* ~2.4 °C/min; faster → discard flush */
/* Accel peak–peak check every N gyro samples (accel ~100Hz vs gyro 1.6kHz). */
#define TCAL_ACCUM_ACCEL_PEEK_DIV 16

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
/* Only the sensor owner may reset/mutate directly; other threads advance a
 * reset generation. Owner consumption never erases publication invalidation. */
void tcal_accum_reset(void);
void tcal_accum_request_reset(void);
void tcal_accum_apply_reset(void);
#if CONFIG_SENSOR_TCAL_HEATED
/* Sensor owner, request lock held; finish is normal completion only.
 * Confirmed bin exits stage once; start-band dwell breaks the initial hold.
 * These APIs stage RAM points and never publish the live model. Explicit
 * user stop publishes already staged points, discarding its unfinished bin. */
void sensor_tcal_heated_accum_feed(const float g[3], float temp);
void sensor_tcal_heated_accum_finish(void);
#endif
void sensor_tcal_runtime_init_from_retained(void);
/* Worker command/process own allocation and publication; argv includes "tcal". */
void sensor_tcal_backup_command(size_t argc, char **argv, uint32_t input_generation);
void sensor_tcal_backup_process(void);
bool sensor_tcal_backup_active(void);
/* IRQ-safe sink: 0 ordinary editor, 1 consumed, 2 consumed and worker wake. */
int sensor_tcal_backup_input_byte(uint8_t byte);
uint32_t sensor_tcal_backup_input_generation(void);
/* IRQ-safe cancellation, retired/freed by the worker. */
void sensor_tcal_backup_input_lost(void);
/* Caller holds the T-Cal lock across point mutation and publication. */
void sensor_tcal_refresh_model(void);
uint32_t sensor_tcal_reference_generation(void);
bool sensor_tcal_take_bias_reset(void);
void sensor_tcal_clear_doffset(void);
void sensor_tcal_mark_measured_bias(void);
/* Sensor owner holds the T-Cal lock; receipt follows real gyro subtraction.
 * Reset-all cancels the deferred user receipt independently of request CLEAR. */
void sensor_tcal_feedback_applied(uint32_t reference_generation, bool offset_applied);
void sensor_tcal_feedback_cancel(void);

/* calibration_thread entry points (were file-local) */
int sensor_perform_boot_calibration(void);
int sensor_perform_runtime_calibration(void);

/*
 * Public T-Cal runtime APIs remain declared in calibration.h and are defined
 * in tcal_runtime.c.
 */

#endif /* CONFIG_SENSOR_USE_TCAL */

#endif /* SLIMENRF_CAL_TCAL_RUNTIME_H */
