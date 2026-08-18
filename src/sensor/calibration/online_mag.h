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

/* Background check invoked from calibration_thread. */
bool sensor_calibration_online_mag_check(void);

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
