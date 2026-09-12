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

#include <errno.h>
#include <string.h>
#include <zephyr/sys/atomic.h>

#include "cal_sample.h"

LOG_MODULE_REGISTER(cal_sample, LOG_LEVEL_INF);

/* A FIFO batch can contain more samples than a calibration consumer can use
 * immediately. Keep a bounded queue of actual vectors so a publish cannot
 * claim a sample that has already been overwritten by the next publish. When
 * full, discard the oldest vector and retain the newest sample. */
#define CAL_SAMPLE_QUEUE_CAPACITY 64

struct cal_sample_vector {
	float v[3];
};

K_MSGQ_DEFINE(accel_sample_queue, sizeof(struct cal_sample_vector), CAL_SAMPLE_QUEUE_CAPACITY, 4);
K_MSGQ_DEFINE(gyro_sample_queue, sizeof(struct cal_sample_vector), CAL_SAMPLE_QUEUE_CAPACITY, 4);
K_MSGQ_DEFINE(mag_sample_queue, sizeof(struct cal_sample_vector), CAL_SAMPLE_QUEUE_CAPACITY, 4);

static float latest_accel[3] = {0};
/* Serialize admission, nonblocking queue publication and session purge. The
 * atomic state gives idle gyro/mag a lock-free exit and identifies publishers
 * preempted before the lock across an end/begin boundary. */
static struct k_spinlock sample_lock;
static atomic_t sample_session;
#define CAL_SAMPLE_CHANNEL_MASK (CAL_SAMPLE_ACCEL | CAL_SAMPLE_GYRO | CAL_SAMPLE_MAG)
static bool accel_snapshot_valid;

static void publish_sample_locked(struct k_msgq *queue, const float v[3])
{
	float discarded[3];

	if (k_msgq_put(queue, v, K_NO_WAIT) == -ENOMSG) {
		/* Explicit overflow policy: discard the oldest queued vector and keep
		 * the newest one. This is loss, never a duplicate or fabricated sample. */
		(void)k_msgq_get(queue, discarded, K_NO_WAIT);
		(void)k_msgq_put(queue, v, K_NO_WAIT);
	}
}

static void publish_sample(struct k_msgq *queue, const float v[3], uint8_t channel)
{
	atomic_val_t session = atomic_get(&sample_session);
	if (!(session & channel)) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&sample_lock);
	if (session == atomic_get(&sample_session)) {
		publish_sample_locked(queue, v);
	}
	k_spin_unlock(&sample_lock, key);
}

static int wait_sample(struct k_msgq *queue, float v[3], k_timeout_t timeout, const char *name)
{
	if (k_msgq_get(queue, v, timeout) != 0) {
		LOG_ERR("%s wait timed out", name);
		return -1;
	}
	return 0;
}

static void samples_set_channels(uint8_t channels)
{
	k_spinlock_key_t key = k_spin_lock(&sample_lock);
	/* Advance the generation even when restarting the same channels. Unsigned
	 * arithmetic also makes generation wrap defined. No producer can enqueue
	 * between the purge and publication of the new admission state. */
	atomic_val_t next = (atomic_val_t)(
		((unsigned long)atomic_get(&sample_session) + CAL_SAMPLE_CHANNEL_MASK + 1)
		& ~(unsigned long)CAL_SAMPLE_CHANNEL_MASK
	);
	k_msgq_purge(&accel_sample_queue);
	k_msgq_purge(&gyro_sample_queue);
	k_msgq_purge(&mag_sample_queue);
	atomic_set(&sample_session, next | (channels & CAL_SAMPLE_CHANNEL_MASK));
	k_spin_unlock(&sample_lock, key);
}

void sensor_calibration_samples_begin(uint8_t channels)
{
	samples_set_channels(channels);
}

void sensor_calibration_samples_end(void)
{
	samples_set_channels(0);
}

void sensor_sample_accel(const float a[3])
{
	atomic_val_t session = atomic_get(&sample_session);
	k_spinlock_key_t key = k_spin_lock(&sample_lock);
	memcpy(latest_accel, a, sizeof(latest_accel));
	accel_snapshot_valid = true;
	if ((session & CAL_SAMPLE_ACCEL) && session == atomic_get(&sample_session)) {
		publish_sample_locked(&accel_sample_queue, a);
	}
	k_spin_unlock(&sample_lock, key);
}

int sensor_wait_accel(float a[3], k_timeout_t timeout)
{
	return wait_sample(&accel_sample_queue, a, timeout, "Accelerometer");
}

bool sensor_peek_accel(float a[3])
{
	if (a == NULL) {
		return false;
	}
	k_spinlock_key_t key = k_spin_lock(&sample_lock);
	bool valid = accel_snapshot_valid;
	if (valid) {
		memcpy(a, latest_accel, sizeof(latest_accel));
	}
	k_spin_unlock(&sample_lock, key);
	return valid;
}

void sensor_sample_gyro(const float g[3])
{
	publish_sample(&gyro_sample_queue, g, CAL_SAMPLE_GYRO);
}

int sensor_wait_gyro(float g[3], k_timeout_t timeout)
{
	return wait_sample(&gyro_sample_queue, g, timeout, "Gyroscope");
}

void sensor_sample_mag(const float m[3])
{
	publish_sample(&mag_sample_queue, m, CAL_SAMPLE_MAG);
}

int sensor_wait_mag(float m[3], k_timeout_t timeout)
{
	return wait_sample(&mag_sample_queue, m, timeout, "Magnetometer");
}

static int wait_latest_accel(float a[3], k_timeout_t timeout)
{
	if (wait_sample(&accel_sample_queue, a, timeout, "Accelerometer") != 0) {
		return -1;
	}
	/* Motion detection samples at its own 500 ms cadence. Discard the
	 * intermediate backlog so it never classifies half-second-old motion. */
	float newer[3];
	while (k_msgq_get(&accel_sample_queue, newer, K_NO_WAIT) == 0) {
		memcpy(a, newer, sizeof(newer));
	}
	return 0;
}

bool wait_for_motion(bool motion, int samples)
{
	uint8_t counts = 0;
	float a[3], last_a[3];
	if (wait_latest_accel(last_a, K_MSEC(1000))) {
		return false;
	}
	LOG_INF("Accelerometer: %.5f %.5f %.5f", (double)last_a[0], (double)last_a[1], (double)last_a[2]);
	for (int i = 0; i < samples + counts; i++) {
		k_msleep(500);
		/* Feed watchdog during long wait periods */
		watchdog_feed(WDT_CHANNEL_CALIBRATION);
		if (wait_latest_accel(a, K_MSEC(1000))) {
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
