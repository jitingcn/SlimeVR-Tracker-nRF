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
#ifndef SLIMENRF_CAL_SAMPLE_H
#define SLIMENRF_CAL_SAMPLE_H

#include <zephyr/kernel.h>

#include <stdbool.h>
#include <stdint.h>

enum sensor_calibration_sample_channel {
	CAL_SAMPLE_ACCEL = 1 << 0,
	CAL_SAMPLE_GYRO = 1 << 1,
	CAL_SAMPLE_MAG = 1 << 2,
};

/* Calibration-thread owner only, with no pending sample wait. Each boundary
 * discards all queued vectors, including an in-flight prior-session publish.
 * Only selected channels enqueue; the live accel peek always remains available. */
void sensor_calibration_samples_begin(uint8_t channels);
void sensor_calibration_samples_end(void);

void sensor_sample_accel(const float a[3]);
int sensor_wait_accel(float a[3], k_timeout_t timeout);
#if CONFIG_SENSOR_USE_ACCEL_CALIBRATION
/* Publication time, not a hardware acquisition timestamp. */
struct sensor_accel_observation {
	float v[3];
	uint32_t sequence;
	uint32_t observed_ms;
};
/* Wait for a queued sample, then return the newest published observation. */
int sensor_wait_accel_observation(struct sensor_accel_observation *observation, k_timeout_t timeout);
/* Discard only pending accel vectors, without changing session admission. */
void sensor_discard_pending_accel(void);
#endif
/* Non-blocking: copy last accel sample if any has been published. */
bool sensor_peek_accel(float a[3]);
#if CONFIG_SENSOR_TCAL_HEATED
bool sensor_peek_accel_fresh(float a[3], int64_t max_age_ms);
#endif
/* One raw FIFO vector represents dt seconds, independent of delivery time.
 * Positive finite dt is required; dt == 0 is a driver-reported acquisition
 * discontinuity and must reject collection, even without a following vector.
 * Sequence advances for every admitted publish, including discontinuities and
 * queue overflow, so losing a marker also exposes loss without stretching dt.
 * Both sequence and queued samples restart at each owner session boundary. */
struct sensor_gyro_sample {
	float v[3];
	float dt;
	uint32_t sequence;
};
void sensor_sample_gyro(const float g[3], float dt);
int sensor_wait_gyro(struct sensor_gyro_sample *sample, k_timeout_t timeout);
#if CONFIG_SENSOR_USE_MAGNETOMETER
void sensor_sample_mag(const float m[3]);
int sensor_wait_mag(float m[3], k_timeout_t timeout);
#endif
bool wait_for_motion(bool motion, int samples);

#endif /* SLIMENRF_CAL_SAMPLE_H */
