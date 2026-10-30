#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>
#include <errno.h>
#include <math.h>
#include <string.h>

#include "sensor/calibration/cal_sample.h"
#include "sensor/calibration/bias_collect.h"
#include "sensor/imu/LSM6DSV.h"
#include "sensor/imu/LSM6DSO.h"

/* Extracted production code uses real Zephyr queues, atomics, spinlocks and
 * scheduling. Only logging/watchdog and the elementary motion predicate are
 * leaves. Wrappers count actual queue traffic or pause BEFORE acquiring a lock. */
static void ignore_log(const char *format, ...) { ARG_UNUSED(format); }
#define LOG_INF(...) ignore_log(__VA_ARGS__)
#define LOG_ERR(...) ignore_log(__VA_ARGS__)
#define LOG_WRN(...) ignore_log(__VA_ARGS__)
#define LOG_DBG(...) ignore_log(__VA_ARGS__)
#define WDT_CHANNEL_CALIBRATION 0
static void watchdog_feed(int channel) { ARG_UNUSED(channel); }
static bool v_epsilon(const float *a, const float *b, float epsilon)
{
	float x = a[0] - b[0], y = a[1] - b[1], z = a[2] - b[2];
	return x * x + y * y + z * z < epsilon * epsilon;
}

/* Driver producer leaves: only SSI I/O is modeled. Resync/read/decode, sensor
 * notification, queue publication and both consumers remain production code. */
static uint8_t fifo_status[2];
static int fifo_status_error, fifo_data_error;
static unsigned fifo_resync_writes;
static uint8_t lsm_unknown_tag_count;
static uint8_t chip_who_am_i = 0x70;
float accel_sensitivity = 1.0f, gyro_sensitivity = 1.0f;

float sensor_get_gyro_odr(void) { return 400.0f; }
float sensor_get_accel_odr(void) { return 400.0f; }

int ssi_reg_write_byte(enum sensor_interface_dev dev, uint8_t reg, uint8_t value)
{
	zassert_equal(dev, SENSOR_INTERFACE_DEV_IMU);
	zassert_equal(reg, LSM6DSV_FIFO_CTRL4);
	zassert_true(value == 0 || value == 6);
	fifo_resync_writes++;
	return 0;
}

int ssi_burst_read(enum sensor_interface_dev dev, uint8_t reg, uint8_t *buf, uint32_t size)
{
	zassert_equal(dev, SENSOR_INTERFACE_DEV_IMU);
	if (reg == LSM6DSV_FIFO_STATUS1 || reg == LSM6DSO_FIFO_STATUS1) {
		zassert_equal(size, 2);
		if (fifo_status_error) {
			return fifo_status_error;
		}
		memcpy(buf, fifo_status, size);
		return 0;
	}
	zassert_true(reg == LSM6DSV_FIFO_DATA_OUT_TAG || reg == LSM6DSO_FIFO_DATA_OUT_TAG);
	if (fifo_data_error) {
		return fifo_data_error;
	}
	memset(buf, 0, size);
	return 0;
}

int ssi_burst_read_interval(enum sensor_interface_dev dev, uint8_t reg, uint8_t *buf,
	uint32_t size, uint32_t interval)
{
	zassert_equal(interval, 7);
	return ssi_burst_read(dev, reg, buf, size);
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
static bool producer_discontinuity;
static uint8_t producer_channel;
static int consumer_result;
static struct sensor_gyro_sample consumer_sample;

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
		sensor_sample_gyro(v, 0.0025f);
	} else {
		sensor_sample_mag(v);
	}
}

static int consume(uint8_t channel, float value[3], k_timeout_t timeout)
{
	if (channel == CAL_SAMPLE_ACCEL) {
		return sensor_wait_accel(value, timeout);
	} else if (channel == CAL_SAMPLE_GYRO) {
		struct sensor_gyro_sample sample;
		int err = sensor_wait_gyro(&sample, timeout);
		if (!err) {
			memcpy(value, sample.v, sizeof(sample.v));
			zassert_equal(sample.dt, 0.0025f);
		}
		return err;
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
	producer_discontinuity = false;
	sensor_calibration_samples_end();
	atomic_clear(&put_calls);
	memset(fifo_status, 0, sizeof(fifo_status));
	fifo_status_error = fifo_data_error = 0;
	fifo_resync_writes = lsm_unknown_tag_count = 0;
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
	struct sensor_gyro_sample gyro;
	zassert_not_equal(sensor_wait_gyro(&gyro, K_NO_WAIT), 0);
	zassert_not_equal(sensor_wait_mag(value, K_NO_WAIT), 0);
}

ZTEST(cal_sample, test_pose_observation_and_capture_purge)
{
	sensor_calibration_samples_begin(CAL_SAMPLE_ACCEL);
	publish(CAL_SAMPLE_ACCEL, 1);
	k_msleep(5);
	uint32_t before_publish = k_uptime_get_32();
	publish(CAL_SAMPLE_ACCEL, 2);
	uint32_t after_publish = k_uptime_get_32();
	k_msleep(5);
	struct sensor_accel_observation observation;
	zassert_equal(sensor_wait_accel_observation(&observation, K_NO_WAIT), 0);
	zassert_equal(observation.v[0], 2);
	zassert_true((uint32_t)(observation.observed_ms - before_publish) <=
	             (uint32_t)(after_publish - before_publish));
	uint32_t previous_sequence = observation.sequence;
	zassert_not_equal(sensor_wait_accel_observation(&observation, K_NO_WAIT), 0);
	publish(CAL_SAMPLE_ACCEL, 3);
	publish(CAL_SAMPLE_ACCEL, 4);
	sensor_discard_pending_accel();
	zassert_not_equal(sensor_wait_accel_observation(&observation, K_NO_WAIT), 0);
	float value[3];
	zassert_not_equal(sensor_wait_accel(value, K_NO_WAIT), 0);
	publish(CAL_SAMPLE_ACCEL, 5);
	zassert_equal(sensor_wait_accel_observation(&observation, K_NO_WAIT), 0);
	zassert_equal(observation.v[0], 5);
	zassert_true(observation.sequence - previous_sequence == 3);
	struct accel_rest_state rest = {0};
	zassert_false(isAccRest(&observation, k_uptime_get_32(), 100, .02f, &rest));
	zassert_true(rest.initialized);
	publish(CAL_SAMPLE_ACCEL, 6);
	k_msleep(120);
	zassert_equal(sensor_wait_accel_observation(&observation, K_NO_WAIT), 0);
	zassert_false(isAccRest(&observation, k_uptime_get_32(), 100, .02f, &rest));
	zassert_false(rest.initialized, "A stale queued observation was treated as newly acquired");
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
	if (producer_discontinuity) {
		sensor_gyro_report_discontinuity();
	} else {
		publish(producer_channel, 42);
	}
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
	consumer_result = sensor_wait_gyro(&consumer_sample, K_MSEC(500));
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
	zassert_equal(consumer_sample.v[0], 42);
	zassert_equal(consumer_sample.v[1], 43);
	zassert_equal(consumer_sample.v[2], 44);
	zassert_equal(consumer_sample.dt, 0.0025f);
	zassert_equal(consumer_sample.sequence, 1);
}

ZTEST(cal_sample, test_gyro_intervals_sequences_and_overflow_gaps)
{
	sensor_calibration_samples_begin(CAL_SAMPLE_GYRO);
	float vector[3] = {1, 2, 3};
	sensor_sample_gyro(vector, 0.00125f);
	struct sensor_gyro_sample sample;
	zassert_equal(sensor_wait_gyro(&sample, K_NO_WAIT), 0);
	zassert_equal(sample.sequence, 1);
	zassert_equal(sample.dt, 0.00125f);
	for (int i = 0; i < 70; i++) {
		vector[0] = i;
		sensor_sample_gyro(vector, 0.0025f);
	}
	zassert_equal(sensor_wait_gyro(&sample, K_NO_WAIT), 0);
	zassert_equal(sample.sequence, 8, "Overflow must expose all six missing intervals");
	zassert_equal(sample.v[0], 6);
	zassert_equal(sample.dt, 0.0025f, "Do not stretch the retained interval across loss");
	for (uint32_t sequence = 9; sequence <= 71; sequence++) {
		zassert_equal(sensor_wait_gyro(&sample, K_NO_WAIT), 0);
		zassert_equal(sample.sequence, sequence);
		zassert_equal(sample.v[0], sequence - 2);
		zassert_equal(sample.dt, 0.0025f);
	}
	/* A same-channel restart purges metadata with its vector and resets sequence. */
	sensor_sample_gyro(vector, 0.5f);
	sensor_calibration_samples_begin(CAL_SAMPLE_GYRO);
	zassert_not_equal(sensor_wait_gyro(&sample, K_NO_WAIT), 0);
	sensor_sample_gyro(vector, 0.003f);
	zassert_equal(sensor_wait_gyro(&sample, K_NO_WAIT), 0);
	zassert_equal(sample.sequence, 1);
	zassert_equal(sample.dt, 0.003f);
}

static void begin_contiguous_measurement(struct sensitivity_stream *stream)
{
	sensor_calibration_samples_begin(CAL_SAMPLE_GYRO);
	*stream = (struct sensitivity_stream){0};
	const float rate[3] = {360, 0, 0};
	sensor_sample_gyro(rate, 0.0025f);
	struct sensor_gyro_sample sample;
	zassert_equal(sensitivity_wait_sample(&sample, stream, K_NO_WAIT), 0);
	zassert_equal(sample.sequence, 1);
}

ZTEST(cal_sample, test_driver_acquisition_loss_reaches_auto_consumer)
{
	uint8_t bytes[7];
	struct sensitivity_stream stream;
	struct sensor_gyro_sample sample;
	for (int empty = 0; empty < 2; empty++) {
		begin_contiguous_measurement(&stream);
		fifo_status[0] = empty ? 0 : 1;
		fifo_status[1] = LSM6DSV_FIFO_OVR_LATCHED;
		unsigned prior = fifo_resync_writes;
		zassert_equal(lsm_fifo_read(bytes, sizeof(bytes)), 0);
		zassert_equal(fifo_resync_writes, prior + 2);
		/* The real acquisition event rejects collection immediately, even
		 * without a next vector whose sequence could reveal the break. */
		zassert_equal(sensitivity_wait_sample(&sample, &stream, K_NO_WAIT), -EINVAL);
		zassert_equal(sample.dt, 0.0f);
		zassert_equal(sample.sequence, 2);
	}
	begin_contiguous_measurement(&stream);
	fifo_status[0] = 1;
	fifo_status[1] = 0;
	fifo_data_error = -EIO;
	zassert_equal(lsm_fifo_read(bytes, sizeof(bytes)), 0);
	zassert_equal(sensitivity_wait_sample(&sample, &stream, K_NO_WAIT), -EINVAL);
	fifo_data_error = 0;

	/* DSO overrun is real loss, whereas FULL alone is not. */
	const uint8_t overruns[] = {LSM6DSO_FIFO_OVR_LATCHED, LSM6DSO_FIFO_OVR};
	for (size_t i = 0; i < ARRAY_SIZE(overruns); i++) {
		begin_contiguous_measurement(&stream);
		fifo_status[0] = 1;
		fifo_status[1] = overruns[i];
		zassert_equal(lsm6dso_fifo_read(bytes, sizeof(bytes)), 1);
		zassert_equal(sensitivity_wait_sample(&sample, &stream, K_NO_WAIT), -EINVAL);
	}
}

ZTEST(cal_sample, test_driver_flush_is_not_averaged_by_bias_consumer)
{
	sensor_calibration_samples_begin(CAL_SAMPLE_ACCEL | CAL_SAMPLE_GYRO);
	const float acceleration[3] = {0, 0, 1};
	const float rate[3] = {2, -3, 0.5f};
	sensor_sample_accel(acceleration);
	sensor_sample_gyro(rate, 0.0025f);
	fifo_status[0] = 1;
	fifo_status[1] = LSM6DSV_FIFO_OVR_LATCHED;
	uint8_t bytes[7];
	zassert_equal(lsm_fifo_read(bytes, sizeof(bytes)), 0);
	float gyro_bias[3] = {10, 11, 12};
	zassert_equal(sensor_offsetBias_internal(gyro_bias, NULL, NULL, 1000, 500), -2);
	for (int i = 0; i < 3; i++) {
		zassert_equal(gyro_bias[i], 10 + i);
	}
}

ZTEST(cal_sample, test_benign_fifo_records_do_not_report_lost_gyro)
{
	struct sensitivity_stream stream;
	struct sensor_gyro_sample sample;
	begin_contiguous_measurement(&stream);
	unsigned prior = atomic_get(&put_calls);
	/* Status I/O failure has not consumed records; empty, full, and capacity
	 * clamping alone prove no loss. Neither may fabricate a loss marker. */
	uint8_t bytes[7];
	fifo_status_error = -EIO;
	zassert_equal(lsm_fifo_read(bytes, sizeof(bytes)), 0);
	fifo_status_error = 0;
	fifo_status[0] = fifo_status[1] = 0;
	zassert_equal(lsm_fifo_read(bytes, sizeof(bytes)), 0);
	fifo_status[0] = 3;
	fifo_status[1] = LSM6DSV_FIFO_FULL_IA;
	zassert_equal(lsm_fifo_read(bytes, sizeof(bytes)), 1);
	fifo_status[1] = LSM6DSO_FIFO_FULL;
	zassert_equal(lsm6dso_fifo_read(bytes, sizeof(bytes)), 1);

	const uint8_t tags[] = {
		LSM6DSV_TAG_FIFO_EMPTY, LSM6DSV_TAG_TEMP, LSM6DSV_TAG_TIMESTAMP,
		LSM6DSV_TAG_CFG_CHANGE, LSM6DSV_TAG_ACCEL_NC, LSM6DSV_TAG_GYRO_NC,
	};
	for (size_t i = 0; i < ARRAY_SIZE(tags); i++) {
		uint8_t packet[7] = {tags[i] << 3};
		float a[3] = {0}, g[3] = {0};
		(void)lsm_fifo_process(0, packet, a, g);
	}
	zassert_equal(atomic_get(&put_calls), prior);
	zassert_equal(fifo_resync_writes, 0);
	/* A genuine zero-rate vector has a positive interval, not a loss marker. */
	const float zero_rate[3] = {0};
	sensor_sample_gyro(zero_rate, 0.0025f);
	zassert_equal(sensitivity_wait_sample(&sample, &stream, K_NO_WAIT), 0);
	zassert_equal(sample.sequence, 2);
	zassert_equal(sample.dt, 0.0025f);
	sensor_calibration_samples_end();
	prior = atomic_get(&put_calls);
	sensor_gyro_report_discontinuity();
	zassert_equal(atomic_get(&put_calls), prior, "Idle discontinuity leaked queue admission");
}

ZTEST(cal_sample, test_inflight_acquisition_loss_cannot_cross_restart)
{
	sensor_calibration_samples_begin(CAL_SAMPLE_GYRO);
	pause_producer = producer_discontinuity = true;
	k_thread_create(&producer_thread, producer_stack, K_THREAD_STACK_SIZEOF(producer_stack),
		producer_entry, NULL, NULL, NULL, K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
	int paused = k_sem_take(&before_lock, K_MSEC(100));
	if (paused == 0) {
		sensor_calibration_samples_end();
		sensor_calibration_samples_begin(CAL_SAMPLE_GYRO);
		k_sem_give(&release_lock);
	}
	int joined = k_thread_join(&producer_thread, K_MSEC(100));
	k_thread_abort(&producer_thread);
	zassert_equal(paused, 0);
	zassert_equal(joined, 0);
	struct sensor_gyro_sample sample;
	zassert_not_equal(sensor_wait_gyro(&sample, K_NO_WAIT), 0,
		"Prior-session acquisition loss escaped into a new measurement");
	struct sensitivity_stream stream = {0};
	const float zero[3] = {0};
	sensor_sample_gyro(zero, 0.0025f);
	zassert_equal(sensitivity_wait_sample(&sample, &stream, K_NO_WAIT), 0);
	zassert_equal(sample.sequence, 1);
}

ZTEST_SUITE(cal_sample, NULL, NULL, before_test, NULL, NULL);
