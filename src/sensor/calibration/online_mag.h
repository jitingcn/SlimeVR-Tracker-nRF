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
#ifndef SLIMENRF_CAL_ONLINE_MAG_H
#define SLIMENRF_CAL_ONLINE_MAG_H

#include <stdbool.h>
#include <stdint.h>

enum online_mag_phase {
	TRAINING,
	FREEZE_REQUESTED,
	FROZEN,
	VALIDATION_READY,
	VALIDATING,
	PROBATION,
	CONFIRMATION_READY
};
enum online_mag_outcome {
	ONLINE_MAG_NONE,
	ONLINE_MAG_UNCHANGED,
	ONLINE_MAG_ENVIRONMENT,
	ONLINE_MAG_UPDATED,
	ONLINE_MAG_REJECTED
};
enum online_mag_rejection {
	ONLINE_MAG_REJECT_NONE,
	ONLINE_MAG_REJECT_FIT,
	ONLINE_MAG_REJECT_RADIAL,
	ONLINE_MAG_REJECT_DIP,
	ONLINE_MAG_REJECT_COVERAGE,
	ONLINE_MAG_REJECT_TIMEOUT,
	ONLINE_MAG_REJECT_CANCELLED,
	ONLINE_MAG_REJECT_MATRIX,
	ONLINE_MAG_REJECT_SAMPLE,
	ONLINE_MAG_REJECT_OVERFLOW,
	ONLINE_MAG_REJECT_NO_BENEFIT
};
struct online_mag_diagnostics {
	float old_rms, new_rms;
	float worst_cell_rms, max_radial_error;
	float old_dip_sd, new_dip_sd, dip_delta;
	uint32_t phase_age_ms;
	int fit_errno;
	uint16_t radial_count, dip_count;
	uint8_t phase, outcome, rejection, radial_cells, dip_cells;
	uint8_t radial_poles, dip_poles, score_phase;
	uint8_t last_gate;
	bool has_model, trial, score_valid;
};
/* Scalar-only coherent snapshot. score_phase identifies the evaluated window;
 * score_valid requires at least one cell with four samples. Immediate radial
 * rejection can report max_radial_error without a scored holdout. Pole bits are
 * +X,-X,+Y,-Y,+Z,-Z. has_model/trial describe the live, not retained, matrix. */
void sensor_calibration_online_mag_diagnostics(struct online_mag_diagnostics *out);
/* Background check invoked from calibration_thread. */
bool sensor_calibration_online_mag_check(void);
/* Runtime-only lifecycle logging, off at boot; unaffected by calibration resets. */
bool sensor_calibration_get_online_mag_debug(void);
void sensor_calibration_set_online_mag_debug(bool enabled);

/* Runtime state helpers used by calibration.c glue (read / mag request). */
void magneto_online_reset(void);
void magneto_online_runtime_reset(void);
void magneto_online_runtime_configure(bool enabled);
void magneto_online_runtime_load_retained(void);
float magneto_online_min_dir_change_threshold(void);
int cal_online_mag_update_count(void);
uint32_t cal_online_mag_norm_count(void);

void magneto_online_snapshot_BAinv(float out[4][3]);
void magneto_online_replace_BAinv_and_reset(const float replacement[4][3]);
/* Sensor-only, consumed after sample service and before applying/fusing mag.
 * norm > 0 supplies validated calibrated-domain norm/dip; zero reacquires.
 * Bootstrap/manual replacement also emits this notification on warm startup. */
bool magneto_online_take_mag_ref(float *norm, float *dip);

/*
 * Public online-mag APIs remain declared in calibration.h and are defined in
 * online_mag.c:
 *   sensor_calibration_online_mag_sample
 *   sensor_calibration_online_mag_status
 *   sensor_calibration_track_mag_norm
 *   sensor_calibration_get_mag_quality
 *   sensor_calibration_set/get_online_mag_enabled
 *   sensor_calibration_online_mag_retained_*
 *   sensor_calibration_online_mag_cold_start
 */

#endif /* SLIMENRF_CAL_ONLINE_MAG_H */
