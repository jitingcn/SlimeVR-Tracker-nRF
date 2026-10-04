#include "globals.h"
#include "system/system.h"
#include "sensor/calibration/imu_calibration.h"
#include <zephyr/kernel.h>
#include "../concurrency/zephyr/sys/atomic.h"

#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "event_probe.h"
void tracker_events_sensor_invalidate(uint8_t reason) { (void)reason; }
static struct host_retained retained_storage;
struct host_retained *retained = &retained_storage;
void sensor_tcal_clear_doffset(void)
{
	retained->bootCalState.doffset_valid = false;
	memset(retained->bootCalState.doffset, 0, sizeof(retained->bootCalState.doffset));
}
static K_MUTEX_DEFINE(sys_storage_lock);
static bool key_exists[8];
static float stored[8][12];
static unsigned writes, clears, errors;
static int clear_error;
static bool nvs_init = true;
static unsigned warm_dirty_count;
static uint32_t warm_tcal_armed;
static unsigned warm_tcal_receipt_count;
static int fs;
static const float zero[3];
static const float gyro[3] = {4.0f, -5.0f, 6.0f};
static const float matrix[4][3] = {{0.1f, 0.0f, 0.0f}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
static pthread_mutex_t rendezvous = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t condition = PTHREAD_COND_INITIALIZER;
static bool hold_write, write_entered, clear_waiting, release_write;
static atomic_t main_suspended;
static atomic_t output_ready;
static bool main_ok = true;
static int sensor_thread_id;
static unsigned resumes;
#define WDT_CHANNEL_SENSOR 0
#define BUILD_TIMESTAMP 1234
#define LOG_INF(...) ((void)0)
#define printk(...) ((void)0)

static void watchdog_resume(int channel) { (void)channel; }
static void k_thread_resume(int *thread) { (void)thread; resumes++; }

/* Semantic LED leaves. Submission is observed per semantic so the fixture can
 * assert the reset-all receipt without reimplementing LED policy. */
static unsigned led_results[LED_SEMANTIC_COUNT];
static unsigned led_requests[LED_SEMANTIC_COUNT];
static uint32_t led_identity;

uint32_t led_request_id(void) { return ++led_identity; }
uint32_t led_event_id(void) { return ++led_identity; }
struct led_token led_begin(enum led_owner owner, uint32_t request_id)
{
	return (struct led_token){ .owner = owner, .session = ++led_identity, .request_id = request_id };
}
enum led_admission led_state(struct led_token token, uint32_t revision, enum led_semantic semantic)
{
	(void)token;
	(void)revision;
	(void)semantic;
	return LED_ADMITTED;
}
enum led_admission led_result(struct led_token token, uint32_t event_id, enum led_semantic semantic)
{
	(void)token;
	(void)event_id;
	led_results[semantic]++;
	return LED_ADMITTED;
}
enum led_admission led_request_event(enum led_owner owner, uint32_t request_id, uint32_t event_id,
				     enum led_semantic semantic)
{
	(void)owner;
	(void)request_id;
	(void)event_id;
	led_requests[semantic]++;
	return LED_ADMITTED;
}

/* Request-generation guard from the request owner (calibration.c, not built
 * here); reset-all must invalidate the collectors accepted before it. */
static uint32_t led_generation = 1;
static unsigned led_invalidations;

uint32_t sensor_calibration_current_generation(void) { return led_generation; }
bool sensor_calibration_generation_valid(uint32_t generation)
{
	return generation == 0 || generation == led_generation;
}
void sensor_calibration_invalidate_requests(void)
{
	led_generation++;
	led_invalidations++;
}
/* Accepted reset admission clears one request kind atomically; this fixture has
 * no live collector, so the leaf only needs to accept the call. */
void sensor_calibration_invalidate_kind(int kind)
{
	(void)kind;
}

/* Warm storage leaves: BMI storage wrapping and guarded writes call these. */
void sys_warm_transaction_begin(void)
{
}

void sys_warm_transaction_mark(uint16_t id, const void *data, size_t size)
{
	(void)id;
	(void)data;
	(void)size;
}

void sys_warm_transaction_end(bool schedule)
{
	(void)schedule;
}

int sys_flush_warm(void)
{
	return 0;
}

/* Reset-all under the storage lock also cancels a pending online-mag
 * configuration write; that owner lives in online_mag.c, not built here. */
void sensor_calibration_online_mag_cancel_pending(void)
{
}

void host_mutex_waiting(struct k_mutex *mutex)
{
	(void)mutex;
	assert(pthread_mutex_lock(&rendezvous) == 0);
	clear_waiting = true;
	assert(pthread_cond_broadcast(&condition) == 0);
	assert(pthread_mutex_unlock(&rendezvous) == 0);
}

int sys_write(uint16_t id, void *ptr, const void *data, size_t len)
{
	k_mutex_lock(&sys_storage_lock, K_FOREVER);
	memcpy(ptr, data, len);
	memcpy(stored[id], data, len);
	key_exists[id] = true;
	writes++;
	k_mutex_unlock(&sys_storage_lock);
	/* Pause after the storage lock is free but before persistence completes:
	 * reset-all must serialize with the transaction, not just an NVS write. */
	assert(pthread_mutex_lock(&rendezvous) == 0);
	if (hold_write) {
		write_entered = true;
		assert(pthread_cond_broadcast(&condition) == 0);
		while (!release_write) {
			assert(pthread_cond_wait(&condition, &rendezvous) == 0);
		}
	}
	assert(pthread_mutex_unlock(&rendezvous) == 0);
	return 0;
}

void retained_update(void) {}
void host_log_error(const char *format, ...) { (void)format; errors++; }
void host_log_warning(const char *format, ...) { (void)format; }
static bool sys_nvs_init(void) { nvs_init = true; return true; }
static int nvs_clear(int *storage)
{
	(void)storage;
	/* During the actual system clear, neither a command nor a frame may
	 * resurrect the canceled transaction. Exercise both against the real owner. */
	int err = sensor_calibration_reset_imu();
	assert(err == -EBUSY || err == -ESHUTDOWN);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_UNCHANGED);
	if (clear_error) {
		return clear_error;
	}
	memset(key_exists, 0, sizeof(key_exists));
	memset(stored, 0, sizeof(stored));
	clears++;
	return 0;
}

#include "clear_resume.inc"

static void assert_no_old_writes(void)
{
	unsigned before = writes;
	sensor_calibration_persist_pending();
	(void)sensor_calibration_apply_pending();
	sensor_calibration_persist_pending();
	assert(writes == before);
	assert(!key_exists[MAIN_GYRO_BIAS_ID]);
	assert(!key_exists[MAIN_ACC_6_BIAS_ID]);
	assert(retained->gyroBias[0] == 0.0f);
	assert(retained->accBAinv[0][0] == 0.0f);
}

static void test_clear_queued_bias(void)
{
	led_results[LED_CANCELLED] = 0;
	led_results[LED_SUCCESS] = 0;
	led_results[LED_APPLIED_NOT_SAVED] = 0;
	struct led_token token = led_begin(LED_OWNER_IMU, led_request_id());
	assert(sensor_calibration_commit_bias(zero, gyro, true, 51, token, 0, 0) == 0);
	assert(event_count == 0);
	assert(sensor_calibration_reset_imu() == -EBUSY);
	/* Confirmation is owned by the console; this call is already authorized. */
	assert(sys_clear() == 0);
	assert(clears == 1);
	assert_no_old_writes();
	assert(event_count == 1);
	assert_event(0, 51, CAL_EVENT_END, CAL_OUTCOME_CANCELLED, CAL_PHASE_APPLY_PENDING, CAL_REASON_RESET);
	/* Reset-all releases the abandoned candidate's own receipt, never a green. */
	assert(led_results[LED_CANCELLED] == 1);
	assert(led_results[LED_SUCCESS] == 0 && led_results[LED_APPLIED_NOT_SAVED] == 0);
	/* It also invalidates every collector accepted before the barrier. */
	assert(led_invalidations >= 1);
	assert(sensor_calibration_commit_bias(zero, zero, true, 0, (struct led_token){0}, 0, 0) == 0);
}

static void test_clear_applied_matrix(void)
{
	assert(sensor_calibration_commit_accel(matrix, 52, (struct led_token){0}, false, 0) == 0);
	assert(event_count == 0);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_FRAME_CHANGED);
	assert(event_count == 1);
	assert_event(0, 52, CAL_EVENT_END, CAL_OUTCOME_SUCCESS, CAL_PHASE_APPLIED, CAL_REASON_NONE);
	sensor_calibration_fusion_applied();
	assert(writes == 0); // Finished, but existing worker has not persisted yet
	assert(sys_clear() == 0);
	assert_no_old_writes();
	assert(event_count == 1); /* Already-applied success cannot become cancellation. */
	/* Reset-all never replaces live coefficients mid-frame. */
	sensor_imu_calibration_t view;
	sensor_calibration_snapshot(&view);
	assert(view.accel_matrix[0][0] == 0.1f);
	assert(sensor_calibration_reset_accel() == 0);
}

static void *persist_thread(void *unused)
{
	(void)unused;
	sensor_calibration_persist_pending();
	return NULL;
}

static void *clear_thread(void *unused)
{
	(void)unused;
	assert(sys_clear() == 0);
	return NULL;
}

static void test_clear_waits_for_persisting_transaction(void)
{
	assert(sensor_calibration_commit_bias(zero, gyro, true, 0, (struct led_token){0}, 0, 0) == 0);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_BIAS_CHANGED);
	sensor_calibration_fusion_applied();
	hold_write = true;
	pthread_t writer, clearer;
	assert(pthread_create(&writer, NULL, persist_thread, NULL) == 0);
	assert(pthread_mutex_lock(&rendezvous) == 0);
	while (!write_entered) {
		assert(pthread_cond_wait(&condition, &rendezvous) == 0);
	}
	assert(pthread_mutex_unlock(&rendezvous) == 0);
	assert(pthread_create(&clearer, NULL, clear_thread, NULL) == 0);
	assert(pthread_mutex_lock(&rendezvous) == 0);
	while (!clear_waiting) {
		assert(pthread_cond_wait(&condition, &rendezvous) == 0);
	}
	/* The clear is blocked behind the immutable flash transaction. */
	assert(clears == 0);
	release_write = true;
	assert(pthread_cond_broadcast(&condition) == 0);
	assert(pthread_mutex_unlock(&rendezvous) == 0);
	assert(pthread_join(writer, NULL) == 0);
	assert(pthread_join(clearer, NULL) == 0);
	assert(writes == 1 && clears == 1);
	assert_no_old_writes();
}

static void test_failed_clear_reports_error_and_releases_barrier(void)
{
	clear_error = -EIO;
	retained->gyroBias[0] = 8.0f;
	assert(sensor_calibration_commit_bias(zero, gyro, true, 0, (struct led_token){0}, 0, 0) == 0);
	assert(sys_clear() == -EIO);
	assert(errors == 1 && clears == 0);
	assert(retained->gyroBias[0] == 8.0f);
	sensor_calibration_persist_pending();
	assert(writes == 0);
	assert(sensor_calibration_reset_imu() == 0);
}

static void test_unavailable_consumer_and_recovery(void)
{
	sensor_calibration_set_consumer_ready(false);
	for (unsigned i = 0; i < 100; i++) {
		assert(sensor_calibration_reset_imu() == -EAGAIN);
		assert(sensor_calibration_reset_accel() == -EAGAIN);
		sensor_calibration_persist_pending();
	}
	assert(writes == 0);
	atomic_set(&main_suspended, true);
	main_ok = false; // actual resume after failed init cannot open admission
	main_imu_resume();
	assert(sensor_calibration_reset_imu() == -EAGAIN);
	atomic_set(&main_suspended, true);
	main_ok = true;
	main_imu_resume();
	assert(sensor_calibration_commit_bias(zero, gyro, true, 0, (struct led_token){0}, 0, 0) == 0);
	/* Accepted before consumer loss: preserve until successful recovery/drain,
	 * but report unavailable rather than indefinitely returning busy. */
	sensor_calibration_set_consumer_ready(false);
	assert(sensor_calibration_reset_imu() == -EAGAIN);
	sensor_calibration_persist_pending();
	assert(writes == 0);
	sensor_calibration_set_consumer_ready(true);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_BIAS_CHANGED);
	sensor_calibration_fusion_applied();
	sensor_calibration_persist_pending();
	assert(stored[MAIN_GYRO_BIAS_ID][0] == 4.0f);
	assert(sensor_calibration_reset_imu() == 0);
}

static void test_bmi_resume_cannot_reopen_terminal_gate(void)
{
	assert(sensor_calibration_commit_bias(zero, gyro, true, 0, (struct led_token){0}, 0, 0) == 0);
	sensor_calibration_prepare_power_down();
	assert(retained->gyroBias[0] == 4.0f);
	assert(retained->fusion_id == 0);
	atomic_set(&main_suspended, true);
	main_imu_resume(); // real caller used by late BMI cleanup
	assert(resumes == 1);
	assert(sensor_calibration_reset_imu() == -ESHUTDOWN);
	assert(sensor_calibration_reset_accel() == -ESHUTDOWN);
	assert(sys_clear() == 0); // reset barrier must not reopen the terminal gate
	assert_no_old_writes();
	assert(sensor_calibration_reset_imu() == -ESHUTDOWN);
}

static void isolated(void (*test)(void))
{
	pid_t child = fork();
	assert(child >= 0);
	if (child == 0) {
		alarm(10); // fail a lock-order regression rather than hang the host suite
		test();
		_exit(0);
	}
	int status;
	assert(waitpid(child, &status, 0) == child);
	assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);
}

int main(void)
{
	sensor_calibration_identity_accel(retained->accBAinv);
	sensor_calibration_imu_load();
	/* Coefficients can load even when discovery has failed. */
	assert(sensor_calibration_reset_imu() == -EAGAIN);
	sensor_calibration_set_consumer_ready(true);
	isolated(test_clear_queued_bias);
	isolated(test_clear_applied_matrix);
	isolated(test_clear_waits_for_persisting_transaction);
	isolated(test_failed_clear_reports_error_and_releases_barrier);
	isolated(test_unavailable_consumer_and_recovery);
	isolated(test_bmi_resume_cannot_reopen_terminal_gate);
	puts("Actual reset-all/resume and calibration lifecycle regressions passed");
	return 0;
}
