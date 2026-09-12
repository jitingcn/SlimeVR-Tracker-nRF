#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>
#include <errno.h>
#include <math.h>
#include <string.h>

#include "sensor/calibration/cal_sample.h"

/* Extracted production code uses real Zephyr queues, atomics, spinlocks and
 * scheduling. Only logging/watchdog and the elementary motion predicate are
 * leaves. Wrappers count actual queue traffic or pause BEFORE acquiring a lock. */
static void ignore_log(const char *format, ...) { ARG_UNUSED(format); }
#define LOG_INF(...) ignore_log(__VA_ARGS__)
#define LOG_ERR(...) ignore_log(__VA_ARGS__)
#define WDT_CHANNEL_CALIBRATION 0
static void watchdog_feed(int channel) { ARG_UNUSED(channel); }
static bool v_epsilon(const float *a, const float *b, float epsilon)
{
	float x = a[0] - b[0], y = a[1] - b[1], z = a[2] - b[2];
	return x * x + y * y + z * z < epsilon * epsilon;
}

static atomic_t put_calls;
static struct k_thread producer_thread, consumer_thread;
K_THREAD_STACK_DEFINE(producer_stack, 2048);
K_THREAD_STACK_DEFINE(consumer_stack, 2048);
K_SEM_DEFINE(before_lock, 0, 1);
K_SEM_DEFINE(release_lock, 0, 1);
K_SEM_DEFINE(consumer_ready, 0, 1);
K_SEM_DEFINE(consumer_done, 0, 1);
static bool pause_producer;
static uint8_t producer_channel;
static int consumer_result;
static float consumer_value[3];

static k_spinlock_key_t observed_spin_lock(struct k_spinlock *lock)
{
	if (pause_producer && k_current_get() == &producer_thread) {
		pause_producer = false;
		k_sem_give(&before_lock);
		k_sem_take(&release_lock, K_FOREVER);
	}
	return k_spin_lock(lock);
}

static int observed_msgq_put(struct k_msgq *queue, const void *data, k_timeout_t timeout)
{
	atomic_inc(&put_calls);
	return k_msgq_put(queue, data, timeout);
}

#define k_spin_lock observed_spin_lock
#define k_msgq_put observed_msgq_put
#include "cal_sample.inc"
#undef k_msgq_put
#undef k_spin_lock

static void publish(uint8_t channel, float value)
{
	float v[] = {value, value + 1, value + 2};
	if (channel == CAL_SAMPLE_ACCEL) {
		sensor_sample_accel(v);
	} else if (channel == CAL_SAMPLE_GYRO) {
		sensor_sample_gyro(v);
	} else {
		sensor_sample_mag(v);
	}
}

static int consume(uint8_t channel, float value[3], k_timeout_t timeout)
{
	if (channel == CAL_SAMPLE_ACCEL) {
		return sensor_wait_accel(value, timeout);
	} else if (channel == CAL_SAMPLE_GYRO) {
		return sensor_wait_gyro(value, timeout);
	}
	return sensor_wait_mag(value, timeout);
}

static void expect_vector(uint8_t channel, float expected)
{
	float value[3];
	zassert_equal(consume(channel, value, K_NO_WAIT), 0);
	for (int axis = 0; axis < 3; axis++) {
		zassert_equal(value[axis], expected + axis, "FIFO reordered or tore a vector");
	}
}

static void before_test(void *fixture)
{
	ARG_UNUSED(fixture);
	pause_producer = false;
	sensor_calibration_samples_end();
	atomic_clear(&put_calls);
}

ZTEST(cal_sample, test_idle_has_no_queue_traffic_but_live_accel)
{
	float value[3];
	for (int i = 0; i < 100; i++) {
		publish(CAL_SAMPLE_ACCEL, i);
		publish(CAL_SAMPLE_GYRO, i);
		publish(CAL_SAMPLE_MAG, i);
	}
	zassert_equal(atomic_get(&put_calls), 0, "Idle sampling still maintains unused queues");
	zassert_true(sensor_peek_accel(value));
	zassert_equal(value[0], 99);
	zassert_equal(value[1], 100);
	zassert_equal(value[2], 101);
	zassert_not_equal(sensor_wait_accel(value, K_NO_WAIT), 0);
	zassert_not_equal(sensor_wait_gyro(value, K_NO_WAIT), 0);
	zassert_not_equal(sensor_wait_mag(value, K_NO_WAIT), 0);
}

ZTEST(cal_sample, test_selected_fifo_overflow_and_end_freshness)
{
	float value[3];
	sensor_calibration_samples_begin(CAL_SAMPLE_ACCEL | CAL_SAMPLE_GYRO);
	for (int i = 0; i < 70; i++) {
		publish(CAL_SAMPLE_ACCEL, i);
		publish(CAL_SAMPLE_GYRO, i + 100);
		publish(CAL_SAMPLE_MAG, i + 200);
	}
	/* Capacity is 64; real vector FIFO drops oldest, never repeats newest. */
	for (int i = 6; i < 70; i++) {
		expect_vector(CAL_SAMPLE_ACCEL, i);
		expect_vector(CAL_SAMPLE_GYRO, i + 100);
	}
	zassert_not_equal(sensor_wait_mag(value, K_NO_WAIT), 0, "Unselected channel admitted");
	publish(CAL_SAMPLE_ACCEL, 80);
	sensor_calibration_samples_end();
	atomic_clear(&put_calls);
	publish(CAL_SAMPLE_ACCEL, 90);
	publish(CAL_SAMPLE_GYRO, 90);
	publish(CAL_SAMPLE_MAG, 90);
	zassert_equal(atomic_get(&put_calls), 0, "Session exit leaked admission");
	zassert_true(sensor_peek_accel(value));
	zassert_equal(value[0], 90);
	sensor_calibration_samples_begin(CAL_SAMPLE_ACCEL);
	zassert_not_equal(sensor_wait_accel(value, K_NO_WAIT), 0, "Session started with stale backlog");
	publish(CAL_SAMPLE_ACCEL, 91);
	expect_vector(CAL_SAMPLE_ACCEL, 91);
}

static void producer_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	publish(producer_channel, 42);
}

ZTEST(cal_sample, test_inflight_publish_cannot_cross_same_channel_restart)
{
	const uint8_t channels[] = {CAL_SAMPLE_ACCEL, CAL_SAMPLE_GYRO, CAL_SAMPLE_MAG};
	for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
		producer_channel = channels[i];
		sensor_calibration_samples_begin(producer_channel);
		pause_producer = true;
		k_thread_create(&producer_thread, producer_stack, K_THREAD_STACK_SIZEOF(producer_stack),
			producer_entry, NULL, NULL, NULL, K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
		int paused = k_sem_take(&before_lock, K_MSEC(100));
		if (paused == 0) {
			/* The publisher has read admission but has not acquired the lock. */
			sensor_calibration_samples_end();
			sensor_calibration_samples_begin(producer_channel);
			k_sem_give(&release_lock);
		}
		int joined = k_thread_join(&producer_thread, K_MSEC(100));
		k_thread_abort(&producer_thread);
		zassert_equal(paused, 0, "Publisher did not reach the session-switch boundary");
		zassert_equal(joined, 0, "In-flight publisher did not return");
		float value[3];
		zassert_not_equal(consume(producer_channel, value, K_NO_WAIT), 0,
			"Prior-session vector escaped into the restarted FIFO");
		publish(producer_channel, 100);
		expect_vector(producer_channel, 100);
	}
}

static void consumer_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	k_sem_give(&consumer_ready);
	consumer_result = sensor_wait_gyro(consumer_value, K_MSEC(500));
	/* Acquiring the production snapshot lock also detects an illegal handoff
	 * before the producer releases its outer sample lock. */
	float accel[3];
	(void)sensor_peek_accel(accel);
	k_sem_give(&consumer_done);
}

ZTEST(cal_sample, test_blocked_higher_priority_consumer_handoff)
{
	sensor_calibration_samples_begin(CAL_SAMPLE_GYRO);
	k_thread_create(&consumer_thread, consumer_stack, K_THREAD_STACK_SIZEOF(consumer_stack),
		consumer_entry, NULL, NULL, NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	zassert_equal(k_sem_take(&consumer_ready, K_MSEC(100)), 0);
	/* Let the consumer block in the real message queue before publication. */
	k_msleep(1);
	producer_channel = CAL_SAMPLE_GYRO;
	k_thread_create(&producer_thread, producer_stack, K_THREAD_STACK_SIZEOF(producer_stack),
		producer_entry, NULL, NULL, NULL, K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
	int completed = k_sem_take(&consumer_done, K_MSEC(500));
	int joined = k_thread_join(&consumer_thread, K_MSEC(100));
	int producer_joined = k_thread_join(&producer_thread, K_MSEC(100));
	k_thread_abort(&producer_thread);
	k_thread_abort(&consumer_thread);
	zassert_equal(completed, 0, "Queue handoff deadlocked or failed to wake consumer");
	zassert_equal(joined, 0);
	zassert_equal(producer_joined, 0);
	zassert_equal(consumer_result, 0);
	zassert_equal(consumer_value[0], 42);
	zassert_equal(consumer_value[1], 43);
	zassert_equal(consumer_value[2], 44);
}

ZTEST_SUITE(cal_sample, NULL, NULL, before_test, NULL, NULL);
