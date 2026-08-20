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
#include "system/watchdog.h"
#include "util.h"

#include <string.h>

#include "cal_sample.h"

LOG_MODULE_REGISTER(cal_sample, LOG_LEVEL_INF);

float aBuf[3] = {0};
static uint64_t accel_sample = 0;
static K_SEM_DEFINE(accel_sem, 0, 64);

void sensor_sample_accel(const float a[3])
{
	memcpy(aBuf, a, sizeof(aBuf));
	accel_sample++;
	k_sem_give(&accel_sem);
}

int sensor_wait_accel(float a[3], k_timeout_t timeout)
{
	if (k_sem_take(&accel_sem, timeout) != 0) {
		LOG_ERR("Accelerometer wait timed out");
		return -1;
	}
	memcpy(a, aBuf, sizeof(aBuf));
	return 0;
}

bool sensor_peek_accel(float a[3])
{
	if (accel_sample == 0 || a == NULL) {
		return false;
	}
	memcpy(a, aBuf, sizeof(aBuf));
	return true;
}

static float gBuf[3] = {0};
/* Counting semaphore: one give per published sample. The limit covers the
 * largest possible FIFO batch (buffer / packet size) so a burst while the
 * consumer is busy is never dropped, and the consumer can never lose a
 * sample to a poll/vs-yield race the way the old counter spin could. */
static K_SEM_DEFINE(gyro_sem, 0, 64);

void sensor_sample_gyro(const float g[3])
{
	memcpy(gBuf, g, sizeof(gBuf));
	k_sem_give(&gyro_sem);
}

int sensor_wait_gyro(float g[3], k_timeout_t timeout)
{
	if (k_sem_take(&gyro_sem, timeout) != 0) {
		LOG_ERR("Gyroscope wait timed out");
		return -1;
	}
	memcpy(g, gBuf, sizeof(gBuf));
	return 0;
}

static float mBuf[3] = {0};
static K_SEM_DEFINE(mag_sem, 0, 64);

void sensor_sample_mag(const float m[3])
{
	memcpy(mBuf, m, sizeof(mBuf));
	k_sem_give(&mag_sem);
}

int sensor_wait_mag(float m[3], k_timeout_t timeout)
{
	if (k_sem_take(&mag_sem, timeout) != 0) {
		LOG_ERR("Magnetometer wait timed out");
		return -1;
	}
	memcpy(m, mBuf, sizeof(mBuf));
	return 0;
}

bool wait_for_motion(bool motion, int samples)
{
	uint8_t counts = 0;
	float a[3], last_a[3];
	if (sensor_wait_accel(last_a, K_MSEC(1000))) {
		return false;
	}
	LOG_INF("Accelerometer: %.5f %.5f %.5f", (double)last_a[0], (double)last_a[1], (double)last_a[2]);
	for (int i = 0; i < samples + counts; i++) {
		k_msleep(500);
		/* Feed watchdog during long wait periods */
		watchdog_feed(WDT_CHANNEL_CALIBRATION);
		if (sensor_wait_accel(a, K_MSEC(1000))) {
			return false;
		}
		LOG_INF("Accelerometer: %.5f %.5f %.5f", (double)a[0], (double)a[1], (double)a[2]);
		if (v_epsilon(a, last_a, 0.1) != motion) {
			LOG_INF("No motion detected");
			counts++;
			if (counts == 2) {
				return true;
			}
		} else {
			counts = 0;
		}
		memcpy(last_a, a, sizeof(a));
	}
	LOG_INF("Motion detected");
	return false;
}
