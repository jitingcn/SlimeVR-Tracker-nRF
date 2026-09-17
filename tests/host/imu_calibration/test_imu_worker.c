#include "globals.h"
#include "system/system.h"
#include "sensor/calibration/imu_calibration.h"
#include "sensor/calibration/bias_collect.h"
#include <zephyr/kernel.h>
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>
#include "event_probe.h"

#define IS_ENABLED(value) 0
#define LOG_INF(...) ((void)0)
#define SYS_LED_PATTERN_LONG 1
#define SYS_LED_PATTERN_OFF 2
#define SYS_LED_PATTERN_ON 3
#define SYS_LED_PATTERN_ONESHOT_COMPLETE 4
#define SYS_LED_PRIORITY_SENSOR 1

static struct host_retained retained_storage;
struct host_retained *retained = &retained_storage;
static uint16_t requested_operation = 61;
static unsigned token_reads;
static int sample_result;
static bool still = true;
static bool invalid_candidate;
static bool preempt_on_submit;
static bool preempt_armed;
static bool preempted;
static int storage_result;

int sys_write(uint16_t id, void *ptr, const void *data, size_t len)
{
	(void)id;
	memcpy(ptr, data, len);
	return storage_result;
}
void retained_update(void) {}
void host_log_error(const char *format, ...) { (void)format; }
void host_log_warning(const char *format, ...) { (void)format; }
static void set_led(int pattern, int priority) { (void)pattern; (void)priority; }
static void k_msleep(int ms) { (void)ms; }
static bool wait_for_motion(bool motion, int samples) { (void)motion; (void)samples; return still; }
uint16_t sensor_calibration_current_operation(void) { token_reads++; return requested_operation; }
int sensor_offsetBias(float *a, float *g, float *temp, float *range)
{
	(void)a;
	(void)temp;
	(void)range;
	g[0] = invalid_candidate ? NAN : 2.0f;
	preempt_armed = preempt_on_submit && !sample_result && !invalid_candidate;
	return sample_result;
}
void host_spin_unlocked(void)
{
	if (!preempt_armed) {
		return;
	}
	preempt_armed = false;
	preempted = true;
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_BIAS_CHANGED);
}

#include "imu_worker.inc"

static void test_preempted_apply(void)
{
	preempt_on_submit = true;
	sensor_calibrate_imu();
	assert(preempted);
	assert(token_reads == 1);
	assert(event_count == 3);
	assert_event(0, 61, CAL_EVENT_STEP, CAL_OUTCOME_NONE, CAL_PHASE_COLLECT, 0);
	assert_event(1, 61, CAL_EVENT_STEP, CAL_OUTCOME_NONE, CAL_PHASE_APPLY_PENDING, 0);
	assert_event(2, 61, CAL_EVENT_END, CAL_OUTCOME_SUCCESS, CAL_PHASE_APPLIED, CAL_REASON_NONE);
	sensor_imu_calibration_t live;
	sensor_calibration_snapshot(&live);
	assert(live.gyro_bias[0] == 2.0f);
	storage_result = -ENOSPC;
	sensor_calibration_persist_pending();
	assert(event_count == 4);
	assert_event(3, 61, CAL_EVENT_STEP, CAL_OUTCOME_NONE, CAL_PHASE_STORAGE, CAL_REASON_STORAGE_ERROR);
}

static void test_silent_apply_and_storage(void)
{
	requested_operation = 0;
	preempt_on_submit = true;
	sensor_calibrate_imu();
	assert(preempted);
	storage_result = -ENOSPC;
	sensor_calibration_persist_pending();
	assert(event_count == 0);
	assert(event_notifications == 0);
}

static void test_terminal_failures(void)
{
	const int errors[] = {-1, -2, -3, BIAS_COLLECT_INSUFFICIENT_SAMPLES};
	const uint8_t reasons[] = {CAL_REASON_MOTION, CAL_REASON_SAMPLE_TIMEOUT,
		CAL_REASON_TEMPERATURE, CAL_REASON_INSUFFICIENT_SAMPLES};
	for (unsigned i = 0; i < sizeof(errors) / sizeof(errors[0]); i++) {
		event_count = 0;
		sample_result = errors[i];
		sensor_calibrate_imu();
		assert(event_count == 2);
		assert_event(1, 61, CAL_EVENT_END, CAL_OUTCOME_FAILED, CAL_PHASE_COLLECT, reasons[i]);
	}
	event_count = 0;
	still = false;
	sensor_calibrate_imu();
	assert(event_count == 1);
	assert_event(0, 61, CAL_EVENT_END, CAL_OUTCOME_FAILED, CAL_PHASE_WAIT_STILL, CAL_REASON_MOTION);
	event_count = 0;
	still = true;
	sample_result = 0;
	invalid_candidate = true;
	sensor_calibrate_imu();
	assert(event_count == 3);
	assert_event(2, 61, CAL_EVENT_END, CAL_OUTCOME_FAILED, CAL_PHASE_APPLY_PENDING, CAL_REASON_CANDIDATE_REJECTED);
}

static void isolated(void (*scenario)(void))
{
	pid_t child = fork();
	assert(child >= 0);
	if (child == 0) {
		scenario();
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
	sensor_calibration_set_consumer_ready(true);
	isolated(test_preempted_apply);
	isolated(test_silent_apply_and_storage);
	isolated(test_terminal_failures);
	puts("Actual IMU worker application/event ordering scenarios passed");
	return 0;
}
