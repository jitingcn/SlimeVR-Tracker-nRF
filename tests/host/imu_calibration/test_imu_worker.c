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

/* Semantic LED leaves. The worker owns the token it read once at admission, so
 * the fixture records submitted results per semantic without LED policy. */
static unsigned led_results[LED_SEMANTIC_COUNT];
static unsigned led_states[LED_SEMANTIC_COUNT];
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
	led_states[semantic]++;
	return LED_ADMITTED;
}
enum led_admission led_result(struct led_token token, uint32_t event_id, enum led_semantic semantic)
{
	(void)event_id;
	if (token.session) {
		led_results[semantic]++;
	}
	return LED_ADMITTED;
}
enum led_admission led_request_event(enum led_owner owner, uint32_t request_id, uint32_t event_id,
				     enum led_semantic semantic)
{
	(void)owner;
	(void)request_id;
	(void)event_id;
	(void)semantic;
	return LED_ADMITTED;
}
static void k_msleep(int ms) { (void)ms; }
static bool wait_for_motion(bool motion, int samples) { (void)motion; (void)samples; return still; }
uint16_t sensor_calibration_current_operation(void) { token_reads++; return requested_operation; }
/* Admission captures the owner token once; a silent request carries none. */
struct led_token sensor_calibration_current_feedback(void)
{
	return requested_operation ? (struct led_token){
		.owner = LED_OWNER_IMU, .session = requested_operation, .request_id = requested_operation,
	} : (struct led_token){0};
}
/* Request-generation guard: the worker captures the generation before
 * collection and the real owner validates it at commit. */
static bool generation_valid_result = true;

uint32_t sensor_calibration_current_generation(void) { return 1; }
bool sensor_calibration_generation_valid(uint32_t generation)
{
	(void)generation;
	return generation_valid_result;
}
void sensor_calibration_invalidate_requests(void) {}
/* Accepted reset admission clears one request kind atomically. */
void sensor_calibration_invalidate_kind(int kind) { (void)kind; }

/* Warm storage leaves: BMI storage wrapping and guarded writes call these. */
void sys_warm_transaction_begin(void) {}
void sys_warm_transaction_mark(uint16_t id, const void *data, size_t size)
{
	(void)id;
	(void)data;
	(void)size;
}
void sys_warm_transaction_end(bool schedule) { (void)schedule; }
int sys_flush_warm(void) { return 0; }

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
	/* The captured token survives until the sensor acknowledges the frame: a
	 * failed write alone is never reported as a terminal, green or otherwise. */
	assert(led_results[LED_APPLIED_NOT_SAVED] == 0 && led_results[LED_SUCCESS] == 0);
	assert(led_states[LED_COLLECT_STILL] > 0 && led_states[LED_PROCESSING] > 0);
	sensor_calibration_fusion_applied();
	assert(led_results[LED_APPLIED_NOT_SAVED] == 1);
	assert(led_results[LED_SUCCESS] == 0 && led_results[LED_PARTIAL] == 0);
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
	/* A silent operation id carries no token, so it submits no LED terminal. */
	sensor_calibration_fusion_applied();
	assert(led_results[LED_SUCCESS] == 0 && led_results[LED_APPLIED_NOT_SAVED] == 0);
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

static void test_stale_generation_candidate_refused(void)
{
	/* The worker hands its pre-collection generation to the real owner; a stale
	 * candidate is cancelled, never applied, stored or reported as failure of
	 * the user's current request. */
	preempt_on_submit = false;
	generation_valid_result = false;
	sensor_calibrate_imu();
	assert(event_count >= 2);
	assert(observed_events[event_count - 1].event == CAL_EVENT_END);
	assert(led_results[LED_CANCELLED] == 1);
	assert(led_results[LED_FAILED] == 0 && led_results[LED_SUCCESS] == 0);
	assert(led_results[LED_APPLIED_NOT_SAVED] == 0 && led_results[LED_PARTIAL] == 0);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_UNCHANGED);
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
	isolated(test_stale_generation_candidate_refused);
	puts("Actual IMU worker application/event ordering scenarios passed");
	return 0;
}
