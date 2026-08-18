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
#ifndef SLIMENRF_CAL_BIAS_COLLECT_H
#define SLIMENRF_CAL_BIAS_COLLECT_H

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
#define BIAS_COLLECT_GYRO_MOTION_THRESHOLD 10.0f // dps (windowed-mean range method)
#endif

#ifndef BIAS_COLLECT_GYRO_MOTION_WINDOW_MS
#define BIAS_COLLECT_GYRO_MOTION_WINDOW_MS 250 // ms - smooth raw gyro noise before range check
#endif

#ifndef BIAS_COLLECT_ACCEL_MOTION_THRESHOLD
#define BIAS_COLLECT_ACCEL_MOTION_THRESHOLD 0.06f // G (range method)
#endif

int sensor_offsetBias_internal(
	float *dest1,
	float *dest2,
	float *avg_temp,
	float *temp_range,
	int max_sample_time_ms,
	int min_sample_time_ms
);
int sensor_offsetBias(float *dest1, float *dest2, float *avg_temp, float *temp_range);

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
/* Rest detector used by 6-side capture in calibration.c */
int isAccRest(float *acc, float *pre_acc, float threshold, int *t, int restdelta);
#endif

#endif /* SLIMENRF_CAL_BIAS_COLLECT_H */
