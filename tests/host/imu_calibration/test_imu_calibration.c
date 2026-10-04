#include "globals.h"
#include "system/system.h"
#include "sensor/calibration/imu_calibration.h"

#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "event_probe.h"
static struct host_retained retained_storage;
struct host_retained *retained = &retained_storage;
void sensor_tcal_clear_doffset(void)
{
	retained->bootCalState.doffset_valid = false;
	memset(retained->bootCalState.doffset, 0, sizeof(retained->bootCalState.doffset));
}
static int storage_result;
static unsigned storage_writes;
static unsigned retained_updates;
static unsigned error_logs;
static int logged_error;
static const float zero[3];

int sys_write(uint16_t id, void *ptr, const void *data, size_t len)
{
	(void)id;
	/* Match production's eager retained update even when persistence fails. */
	memcpy(ptr, data, len);
	storage_writes++;
	return storage_result;
}

void retained_update(void)
{
	retained_updates++;
}

void host_log_error(const char *format, ...)
{
	(void)format;
	va_list args;
	va_start(args, format);
	logged_error = va_arg(args, int);
	va_end(args);
	error_logs++;
}

void host_log_warning(const char *format, ...)
{
	(void)format;
}

/* Semantic LED owner contract. The real system/led.h types arrive through
 * calibration.h; only submission is observed here, never rendering. */
enum led_record_kind { LED_RECORD_REQUEST, LED_RECORD_STATE, LED_RECORD_RESULT };
struct led_record {
	enum led_record_kind kind;
	struct led_token token;
	uint32_t event_id;
	enum led_semantic semantic;
};
static struct led_record led_records[64];
static unsigned led_record_count;
static uint32_t led_identity;

uint32_t led_request_id(void)
{
	return ++led_identity;
}

uint32_t led_event_id(void)
{
	return ++led_identity;
}

struct led_token led_begin(enum led_owner owner, uint32_t request_id)
{
	return (struct led_token){ .owner = owner, .session = ++led_identity, .request_id = request_id };
}

static void led_record(enum led_record_kind kind, struct led_token token, uint32_t event_id,
		       enum led_semantic semantic)
{
	assert(led_record_count < sizeof(led_records) / sizeof(led_records[0]));
	led_records[led_record_count++] = (struct led_record){
		.kind = kind,
		.token = token,
		.event_id = event_id,
		.semantic = semantic,
	};
}

enum led_admission led_state(struct led_token token, uint32_t revision, enum led_semantic semantic)
{
	led_record(LED_RECORD_STATE, token, revision, semantic);
	return LED_ADMITTED;
}

enum led_admission led_result(struct led_token token, uint32_t event_id, enum led_semantic semantic)
{
	led_record(LED_RECORD_RESULT, token, event_id, semantic);
	return LED_ADMITTED;
}

enum led_admission led_request_event(enum led_owner owner, uint32_t request_id, uint32_t event_id,
				     enum led_semantic semantic)
{
	led_record(LED_RECORD_REQUEST,
		   (struct led_token){ .owner = owner, .session = 0, .request_id = request_id },
		   event_id, semantic);
	return LED_ADMITTED;
}

static unsigned led_result_count(void)
{
	unsigned count = 0;
	for (unsigned i = 0; i < led_record_count; i++) {
		count += led_records[i].kind == LED_RECORD_RESULT;
	}
	return count;
}

static const struct led_record *led_result_record(unsigned index)
{
	for (unsigned i = 0; i < led_record_count; i++) {
		if (led_records[i].kind != LED_RECORD_RESULT) {
			continue;
		}
		if (index == 0) {
			return &led_records[i];
		}
		index--;
	}
	return NULL;
}

static void assert_led_results(unsigned expected)
{
	assert(led_result_count() == expected);
}

static void assert_led_result(unsigned index, struct led_token token, enum led_semantic semantic)
{
	const struct led_record *record = led_result_record(index);
	assert(record != NULL);
	assert(record->token.owner == token.owner);
	assert(record->token.session == token.session);
	assert(record->token.request_id == token.request_id);
	assert(record->semantic == semantic);
}

static struct led_token owner_token(enum led_owner owner)
{
	return led_begin(owner, led_request_id());
}

/* Request-generation guard owned by the request owner (calibration.c, which
 * this fixture does not build). Generation 0 is a direct commit; any other
 * value must still match the current global generation. */
static uint32_t led_generation = 1;
static unsigned led_invalidations;

uint32_t sensor_calibration_current_generation(void)
{
	return led_generation;
}

bool sensor_calibration_generation_valid(uint32_t generation)
{
	return generation == 0 || generation == led_generation;
}

void sensor_calibration_invalidate_requests(void)
{
	led_generation++;
	led_invalidations++;
}

/* Accepted reset admission targets one request kind; no live collector exists
 * in this fixture, so the atomic target-kind clear is a pure leaf. */
static unsigned led_kind_invalidations;

void sensor_calibration_invalidate_kind(int kind)
{
	(void)kind;
	led_kind_invalidations++;
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

static void near(float actual, float expected)
{
	assert(fabsf(actual - expected) < 0.00001f);
}

static void vector_equal(const float actual[3], const float expected[3])
{
	for (unsigned i = 0; i < 3; i++) {
		near(actual[i], expected[i]);
	}
}

static void snapshot_equal(const sensor_imu_calibration_t *actual, const sensor_imu_calibration_t *expected)
{
	vector_equal(actual->accel_bias, expected->accel_bias);
	vector_equal(actual->gyro_bias, expected->gyro_bias);
	for (unsigned row = 0; row < 4; row++) {
		vector_equal(actual->accel_matrix[row], expected->accel_matrix[row]);
	}
}

static sensor_imu_calibration_t snapshot(void)
{
	sensor_imu_calibration_t result;
	sensor_calibration_snapshot(&result);
	return result;
}

static void apply_and_ack(enum sensor_calibration_effect expected)
{
	unsigned writes_before = storage_writes;
	assert(sensor_calibration_apply_pending() == expected);
	assert(storage_writes == writes_before);
	assert(sensor_calibration_fusion_stale());
	sensor_calibration_fusion_applied();
	assert(sensor_calibration_fusion_stale());
	assert(sensor_calibration_commit_bias(zero, zero, true, 0, (struct led_token){0}, 0, 0) == -EBUSY);
	sensor_calibration_persist_pending();
	assert(!sensor_calibration_fusion_stale());
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_SAVE_FUSION);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_UNCHANGED);
}

static void baseline(void)
{
	storage_result = 0;
	sensor_calibration_set_consumer_ready(true);
	const unsigned events_before = event_count;
	const unsigned notifications_before = event_notifications;
	assert(sensor_calibration_reset_imu() == 0);
	apply_and_ack(SENSOR_CALIBRATION_FRAME_CHANGED);
	assert(sensor_calibration_reset_accel() == 0);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_COEFFICIENTS_CHANGED);
	assert(!sensor_calibration_fusion_stale());
	sensor_calibration_persist_pending();
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_UNCHANGED);
	assert(event_count == events_before);
	assert(event_notifications == notifications_before);
	event_count = 0;
	event_notifications = 0;
	storage_writes = 0;
	retained_updates = 0;
	error_logs = 0;
	led_record_count = 0;
	led_generation = 1;
	led_invalidations = 0;
}

static const float matrix_cases[][4][3] = {
	{{0}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}},
	{{0}, {1, .99999f, 0}, {.99999f, 1, 0}, {0, 0, 1}},
	{{0}, {100, 0, 0}, {0, 100, 0}, {0, 0, 100}},
	{{.1f, -.2f, .05f}, {1.1f, .2f, 0}, {0, 1.1f, -.1f}, {.03f, 0, 1.1f}},
};

static void test_stored_matrix_sanity(void)
{
	/* Each child starts before the load-once owner is initialized. */
	for (unsigned c = 0; c < sizeof(matrix_cases) / sizeof(matrix_cases[0]); c++) {
		pid_t child = fork();
		assert(child >= 0);
		if (child == 0) {
			memcpy(retained->accBAinv, matrix_cases[c], sizeof(retained->accBAinv));
			sensor_calibration_imu_load();
			sensor_imu_calibration_t actual = snapshot();
			float expected[4][3];
			sensor_calibration_identity_accel(expected);
			if (c == 3) memcpy(expected, matrix_cases[c], sizeof(expected));
			for (unsigned row = 0; row < 4; row++) {
				vector_equal(actual.accel_matrix[row], expected[row]);
				vector_equal(retained->accBAinv[row], expected[row]);
			}
			_exit(0);
		}
		int status;
		assert(waitpid(child, &status, 0) == child);
		assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);
	}
}

static void test_startup_and_load_once(void)
{
	float identity[4][3];
	sensor_calibration_identity_accel(identity);
	assert(!sensor_calibration_imu_ready());
	assert(sensor_calibration_commit_bias(zero, zero, true, 0, (struct led_token){0}, 0, 0) == -EAGAIN);
	assert(sensor_calibration_commit_accel(identity, 0, (struct led_token){0}, false, 0) == -EAGAIN);
	assert(!sensor_calibration_fusion_stale());
	assert(storage_writes == 0);

	const float accel_bias[3] = {0.1f, -0.1f, 0.2f};
	const float gyro_bias[3] = {1.0f, -2.0f, 3.0f};
	memcpy(retained->accelBias, accel_bias, sizeof(accel_bias));
	memcpy(retained->gyroBias, gyro_bias, sizeof(gyro_bias));
	memcpy(retained->accBAinv, identity, sizeof(identity));
	sensor_calibration_imu_load();
	assert(sensor_calibration_imu_ready());
	sensor_imu_calibration_t loaded = snapshot();
	vector_equal(loaded.accel_bias, accel_bias);
	vector_equal(loaded.gyro_bias, gyro_bias);
	for (unsigned row = 0; row < 4; row++) {
		vector_equal(loaded.accel_matrix[row], identity[row]);
	}
	retained->gyroBias[0] = 19.0f;
	retained->accBAinv[1][0] = 3.0f;
	sensor_calibration_imu_load();
	sensor_imu_calibration_t rescanned = snapshot();
	snapshot_equal(&rescanned, &loaded);
}

static void check_invalid_startup(bool nonfinite)
{
	sensor_calibration_identity_accel(retained->accBAinv);
	retained->accelBias[0] = nonfinite ? NAN : 0.5f;
	retained->gyroBias[0] = 1.0f;
	retained->accBAinv[1][0] = nonfinite ? INFINITY : 1.5f;
	retained->bootCalState.doffset_valid = true;
	for (unsigned i = 0; i < 3; i++) {
		retained->bootCalState.doffset[i] = (float)i + 1.0f;
	}
	retained->fusion_id = 7;
	assert(!sensor_calibration_imu_ready());
	sensor_calibration_imu_load();
	assert(sensor_calibration_imu_ready());
	sensor_imu_calibration_t healed = snapshot();
	vector_equal(healed.accel_bias, zero);
	vector_equal(healed.gyro_bias, zero);
	float identity[4][3];
	sensor_calibration_identity_accel(identity);
	for (unsigned row = 0; row < 4; row++) {
		vector_equal(healed.accel_matrix[row], identity[row]);
		vector_equal(retained->accBAinv[row], identity[row]);
	}
	vector_equal(retained->accelBias, zero);
	vector_equal(retained->gyroBias, zero);
	assert(!retained->bootCalState.doffset_valid);
	vector_equal(retained->bootCalState.doffset, zero);
	assert(retained->fusion_id == 7);
	assert(!sensor_calibration_fusion_stale());
	float accel[3] = {1.0f, 2.0f, 3.0f};
	const float input[3] = {1.0f, 2.0f, 3.0f};
	sensor_calibration_apply_accel(accel);
	vector_equal(accel, input);
	/* Reloading invalid retained data after readiness must not replace the owner. */
	retained->gyroBias[0] = INFINITY;
	sensor_calibration_imu_load();
	sensor_imu_calibration_t rescanned = snapshot();
	snapshot_equal(&rescanned, &healed);
}

static void test_nonfinite_startup(void)
{
	check_invalid_startup(true);
}

static void test_out_of_bounds_startup(void)
{
	check_invalid_startup(false);
}

static void test_pending_bias_atomic_application_and_busy(void)
{
	baseline();
	sensor_imu_calibration_t before = snapshot();
	const float accel_bias[3] = {0.1f, 0.2f, 0.3f};
	const float gyro_bias[3] = {1.0f, 2.0f, 3.0f};
	assert(sensor_calibration_commit_bias(accel_bias, gyro_bias, false, 41, (struct led_token){0}, 0, 0) == 0);
	assert(event_count == 0);
	assert(sensor_calibration_fusion_stale());
	sensor_imu_calibration_t pending = snapshot();
	snapshot_equal(&pending, &before);
	float gyro[3] = {10.0f, 20.0f, 30.0f};
	const float original_gyro[3] = {10.0f, 20.0f, 30.0f};
	sensor_calibration_subtract_gyro_bias(gyro);
	vector_equal(gyro, original_gyro);
	assert(sensor_calibration_commit_bias(zero, zero, true, 0, (struct led_token){0}, 0, 0) == -EBUSY);
	assert(sensor_calibration_commit_accel(before.accel_matrix, 0, (struct led_token){0}, false, 0) == -EBUSY);
	/* Rescan must preserve the first accepted transaction as well as the old view. */
	sensor_calibration_imu_load();
	apply_and_ack(SENSOR_CALIBRATION_BIAS_CHANGED);
	assert(event_count == 1);
	assert_event(0, 41, CAL_EVENT_END, CAL_OUTCOME_SUCCESS, CAL_PHASE_APPLIED, CAL_REASON_NONE);
	sensor_imu_calibration_t expected = before;
	memcpy(expected.accel_bias, accel_bias, sizeof(accel_bias));
	memcpy(expected.gyro_bias, gyro_bias, sizeof(gyro_bias));
	sensor_imu_calibration_t after = snapshot();
	snapshot_equal(&after, &expected);
	sensor_calibration_subtract_gyro_bias(gyro);
	const float corrected[3] = {9.0f, 18.0f, 27.0f};
	vector_equal(gyro, corrected);
	float exported_bias[3];
	sensor_calibration_gyro_bias(exported_bias);
	vector_equal(exported_bias, gyro_bias);
	/* Nonpersistent gyro calibration still applies, without changing retained baseline. */
	vector_equal(retained->gyroBias, zero);
	assert(storage_writes == 0);
}

static void test_invalid_candidates_leave_applied_unchanged(void)
{
	baseline();
	sensor_imu_calibration_t before = snapshot();
	const float nonfinite[3] = {NAN, 0.0f, 0.0f};
	const float accel_limit[3] = {0.5f, 0.0f, 0.0f};
	const float gyro_limit[3] = {0.0f, 50.0f, 0.0f};
	assert(sensor_calibration_commit_bias(NULL, zero, true, 0, (struct led_token){0}, 0, 0) == -EINVAL);
	assert(sensor_calibration_commit_bias(nonfinite, zero, true, 0, (struct led_token){0}, 0, 0) == -EINVAL);
	assert(sensor_calibration_commit_bias(zero, nonfinite, true, 0, (struct led_token){0}, 0, 0) == -EINVAL);
	assert(sensor_calibration_commit_bias(accel_limit, zero, true, 0, (struct led_token){0}, 0, 0) == -EINVAL);
	assert(sensor_calibration_commit_bias(zero, gyro_limit, true, 0, (struct led_token){0}, 0, 0) == -EINVAL);
	float matrix[4][3];
	memcpy(matrix, before.accel_matrix, sizeof(matrix));
	matrix[1][1] = INFINITY;
	assert(sensor_calibration_commit_accel(matrix, 0, (struct led_token){0}, false, 0) == -EINVAL);
	memcpy(matrix, before.accel_matrix, sizeof(matrix));
	matrix[0][2] = 0.5f;
	assert(sensor_calibration_commit_accel(matrix, 0, (struct led_token){0}, false, 0) == -EINVAL);
	memcpy(matrix, before.accel_matrix, sizeof(matrix));
	matrix[2][1] = 1.5f;
	assert(sensor_calibration_commit_accel(matrix, 0, (struct led_token){0}, false, 0) == -EINVAL);
	for (unsigned c = 0; c < 3; c++) {
		assert(sensor_calibration_commit_accel(matrix_cases[c], 0,
			(struct led_token){0}, false, 0) == -EINVAL);
	}
	assert(sensor_calibration_commit_accel(NULL, 0, (struct led_token){0}, false, 0) == -EINVAL);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_UNCHANGED);
	assert(!sensor_calibration_fusion_stale());
	sensor_imu_calibration_t after = snapshot();
	snapshot_equal(&after, &before);
	assert(storage_writes == 0);
}

static void test_matrix_commit_sample_boundary(void)
{
	baseline();
	const float accel_bias[3] = {0.1f, 0.2f, 0.0f};
	const float gyro_bias[3] = {1.0f, 2.0f, 3.0f};
	assert(sensor_calibration_commit_bias(accel_bias, gyro_bias, true, 0, (struct led_token){0}, 0, 0) == 0);
	apply_and_ack(SENSOR_CALIBRATION_BIAS_CHANGED);
	sensor_imu_calibration_t before = snapshot();
	const float matrix[4][3] = {
		{0.1f, -0.2f, 0.05f},
		{1.1f, 0.2f, 0.0f},
		{0.0f, 1.1f, -0.1f},
		{0.05f, 0.0f, 1.1f},
	};
	assert(sensor_calibration_commit_accel(matrix, 0, (struct led_token){0}, false, 0) == 0);
	sensor_imu_calibration_t pending = snapshot();
	snapshot_equal(&pending, &before);
	float accel[3] = {1.0f, 2.0f, 3.0f};
	const float input[3] = {1.0f, 2.0f, 3.0f};
	sensor_calibration_apply_accel(accel);
	vector_equal(accel, input);
	apply_and_ack(SENSOR_CALIBRATION_FRAME_CHANGED);
	sensor_imu_calibration_t expected = before;
	memcpy(expected.accel_matrix, matrix, sizeof(matrix));
	sensor_imu_calibration_t after = snapshot();
	snapshot_equal(&after, &expected);
	sensor_calibration_apply_accel(accel);
	const float transformed[3] = {1.43f, 2.125f, 3.29f};
	vector_equal(accel, transformed);
	float gyro[3] = {4.0f, 5.0f, 6.0f};
	sensor_calibration_subtract_gyro_bias(gyro);
	const float corrected_gyro[3] = {3.0f, 3.0f, 3.0f};
	vector_equal(gyro, corrected_gyro);
	/* Manual matrix reset changes coefficients, not the fusion frame/bias. */
	assert(sensor_calibration_reset_accel() == 0);
	assert(!sensor_calibration_fusion_stale());
	sensor_imu_calibration_t reset_pending = snapshot();
	snapshot_equal(&reset_pending, &after);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_COEFFICIENTS_CHANGED);
	assert(!sensor_calibration_fusion_stale());
	assert(sensor_calibration_commit_bias(zero, zero, true, 0, (struct led_token){0}, 0, 0) == -EBUSY);
	sensor_calibration_persist_pending();
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_UNCHANGED);
	sensor_imu_calibration_t reset = snapshot();
	snapshot_equal(&reset, &before);
	memcpy(accel, input, sizeof(accel));
	sensor_calibration_apply_accel(accel);
	vector_equal(accel, input);
}

static void test_reset_clears_boot_offset_at_application(void)
{
	baseline();
	retained->bootCalState.doffset_valid = true;
	for (unsigned i = 0; i < 3; i++) {
		retained->bootCalState.doffset[i] = (float)i + 1.0f;
	}
	assert(sensor_calibration_reset_imu() == 0);
	assert(retained->bootCalState.doffset_valid);
	apply_and_ack(SENSOR_CALIBRATION_FRAME_CHANGED);
	assert(!retained->bootCalState.doffset_valid);
	vector_equal(retained->bootCalState.doffset, zero);
}

/* The LED receipt belongs to the immutable transaction: a silent operation id
 * still holds the caller's token until frame consumption and promised writes
 * both finish, and it is reported exactly once. */
static void test_led_bias_token_completes_after_frame_and_storage(void)
{
	baseline();
	const float gyro_bias[3] = {1.0f, 2.0f, 3.0f};
	struct led_token token = owner_token(LED_OWNER_IMU);
	assert(sensor_calibration_commit_bias(zero, gyro_bias, true, 0, token, 0, 0) == 0);
	assert_led_results(0); /* Queueing is never completion. */
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_BIAS_CHANGED);
	assert_led_results(0);
	sensor_calibration_fusion_applied();
	assert_led_results(0); /* Fusion alone must not report a green terminal. */
	sensor_calibration_persist_pending();
	assert_led_results(1);
	assert_led_result(0, token, LED_SUCCESS);
	sensor_calibration_persist_pending();
	sensor_calibration_fusion_applied();
	assert_led_results(1); /* Terminal results cannot repeat. */
}

static void test_led_storage_failure_reports_applied_not_saved(void)
{
	baseline();
	const float gyro_bias[3] = {4.0f, 5.0f, 6.0f};
	struct led_token token = owner_token(LED_OWNER_IMU);
	storage_result = -ENOSPC;
	assert(sensor_calibration_commit_bias(zero, gyro_bias, true, 0, token, 0, 0) == 0);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_BIAS_CHANGED);
	sensor_calibration_fusion_applied();
	sensor_calibration_persist_pending();
	assert_led_results(1);
	assert_led_result(0, token, LED_APPLIED_NOT_SAVED);
	storage_result = 0;
}

static void test_led_prior_storage_error_is_not_reported_as_success(void)
{
	baseline();
	struct led_token token = owner_token(LED_OWNER_IMU);
	assert(sensor_calibration_commit_bias(zero, zero, true, 0, token, -EIO, 0) == 0);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_BIAS_CHANGED);
	sensor_calibration_fusion_applied();
	sensor_calibration_persist_pending();
	assert_led_results(1);
	assert_led_result(0, token, LED_APPLIED_NOT_SAVED);
}

static void test_led_bookkeeping_error_before_first_frame_is_retained(void)
{
	baseline();
	const float gyro_bias[3] = {1.0f, 2.0f, 3.0f};
	const uint32_t generation = sensor_calibration_current_generation();
	struct led_token token = owner_token(LED_OWNER_IMU);
	assert(sensor_calibration_commit_bias(zero, gyro_bias, true, 0, token, 0, generation) == 0);
	sensor_calibration_record_storage_error(generation, -EIO);
	assert_led_results(0); /* Bookkeeping can finish before the first frame. */
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_BIAS_CHANGED);
	sensor_calibration_fusion_applied();
	sensor_calibration_persist_pending(); /* Coefficient writes themselves succeed. */
	assert_led_results(1);
	assert_led_result(0, token, LED_APPLIED_NOT_SAVED);
}

static void test_led_partial_accel_completion(void)
{
	baseline();
	const float matrix[4][3] = {
		{0.1f, 0.0f, 0.0f},
		{1.0f, 0.2f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f},
	};
	struct led_token token = owner_token(LED_OWNER_ACC);
	assert(sensor_calibration_commit_accel(matrix, 0, token, true, 0) == 0);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_FRAME_CHANGED);
	sensor_calibration_fusion_applied();
	assert_led_results(0);
	sensor_calibration_persist_pending();
	assert_led_results(1);
	assert_led_result(0, token, LED_PARTIAL);
}

/* Storage may finish before the sensor acknowledges the frame, and admission
 * reopens at that point. A candidate queued afterwards must not steal the
 * earlier transaction's frame acknowledgement. */
static void test_led_frame_receipt_survives_reopened_admission(void)
{
	baseline();
	const float gyro_a[3] = {1.0f, 2.0f, 3.0f};
	const float gyro_b[3] = {4.0f, 5.0f, 6.0f};
	struct led_token a = owner_token(LED_OWNER_IMU);
	assert(sensor_calibration_commit_bias(zero, gyro_a, true, 0, a, 0, 0) == 0);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_BIAS_CHANGED);
	sensor_calibration_persist_pending();
	assert_led_results(0); /* Storage alone cannot acknowledge the frame. */
	struct led_token b = owner_token(LED_OWNER_IMU);
	assert(b.session != a.session);
	assert(sensor_calibration_commit_bias(zero, gyro_b, true, 0, b, 0, 0) == 0);
	sensor_calibration_fusion_applied();
	assert_led_results(1);
	assert_led_result(0, a, LED_SUCCESS);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_BIAS_CHANGED);
	sensor_calibration_persist_pending();
	assert_led_results(1);
	sensor_calibration_fusion_applied();
	assert_led_results(2);
	assert_led_result(1, b, LED_SUCCESS);
}

static void test_led_frame_receipt_keeps_failed_storage(void)
{
	baseline();
	const float gyro_a[3] = {1.0f, 2.0f, 3.0f};
	const float gyro_b[3] = {4.0f, 5.0f, 6.0f};
	struct led_token a = owner_token(LED_OWNER_IMU);
	storage_result = -ENOSPC;
	assert(sensor_calibration_commit_bias(zero, gyro_a, true, 0, a, 0, 0) == 0);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_BIAS_CHANGED);
	sensor_calibration_persist_pending();
	assert_led_results(0);
	storage_result = 0;
	struct led_token b = owner_token(LED_OWNER_IMU);
	assert(sensor_calibration_commit_bias(zero, gyro_b, true, 0, b, 0, 0) == 0);
	sensor_calibration_fusion_applied();
	assert_led_results(1);
	assert_led_result(0, a, LED_APPLIED_NOT_SAVED);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_BIAS_CHANGED);
	sensor_calibration_persist_pending();
	sensor_calibration_fusion_applied();
	assert_led_results(2);
	assert_led_result(1, b, LED_SUCCESS);
}

/* A collector accepted under an older global generation must be refused
 * outright after reset-all invalidation, and the new generation must still be
 * the one that owns admission and can complete. */
static void test_led_stale_generation_candidate_is_refused(void)
{
	baseline();
	const float gyro_a[3] = {1.0f, 2.0f, 3.0f};
	const float gyro_b[3] = {4.0f, 5.0f, 6.0f};
	const uint32_t stale = sensor_calibration_current_generation();
	/* A boot/runtime doffset must survive every refused candidate. */
	retained->bootCalState.doffset_valid = true;
	retained->bootCalState.doffset[0] = 5.0f;
	sensor_calibration_invalidate_requests();
	assert(led_invalidations == 1);
	assert(!sensor_calibration_generation_valid(stale));
	assert(sensor_calibration_generation_valid(sensor_calibration_current_generation()));
	struct led_token token_a = owner_token(LED_OWNER_IMU);
	assert(sensor_calibration_commit_bias(zero, gyro_a, true, 0, token_a, 0, stale) == -ECANCELED);
	assert_led_results(0);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_UNCHANGED);
	assert(storage_writes == 0);
	assert(!sensor_calibration_fusion_stale());
	assert(retained->bootCalState.doffset_valid);
	assert(retained->bootCalState.doffset[0] == 5.0f);
	struct led_token token_b = owner_token(LED_OWNER_IMU);
	assert(sensor_calibration_commit_bias(zero, gyro_b, true, 0, token_b, 0,
					      sensor_calibration_current_generation()) == 0);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_BIAS_CHANGED);
	/* Only the accepted current-generation candidate clears the boot offset. */
	assert(!retained->bootCalState.doffset_valid);
	sensor_calibration_fusion_applied();
	sensor_calibration_persist_pending();
	assert(storage_writes == 1);
	assert_led_results(1);
	assert_led_result(0, token_b, LED_SUCCESS);
}

/* The owner begins its own session for a reset, rather than borrowing a caller
 * token. That generated token is the one the eventual terminal must carry. */
static void test_led_reset_session_owns_its_token(void)
{
	baseline();
	assert(sensor_calibration_reset_imu() == 0);
	assert_led_results(1);
	const struct led_record *accepted = led_result_record(0);
	assert(accepted != NULL);
	assert(accepted->semantic == LED_ACCEPTED);
	struct led_token token = accepted->token;
	assert(token.session != 0);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_FRAME_CHANGED);
	sensor_calibration_fusion_applied();
	assert_led_results(1);
	sensor_calibration_persist_pending();
	assert_led_results(2);
	assert_led_result(1, token, LED_SUCCESS);
}

static void test_power_close_is_permanent_and_fusion_ack(void)
{
	baseline();
	const float gyro_bias[3] = {4.0f, -5.0f, 6.0f};
	assert(sensor_calibration_commit_bias(zero, gyro_bias, true, 0, (struct led_token){0}, 0, 0) == 0);
	retained->fusion_id = 9;
	sensor_calibration_prepare_power_down();
	sensor_imu_calibration_t applied = snapshot();
	vector_equal(applied.gyro_bias, gyro_bias);
	vector_equal(retained->gyroBias, gyro_bias);
	assert(storage_writes == 1);
	assert(retained->fusion_id == 0);
	assert(sensor_calibration_commit_bias(zero, zero, true, 0, (struct led_token){0}, 0, 0) == -ESHUTDOWN);
	assert(sensor_calibration_commit_accel(applied.accel_matrix, 0, (struct led_token){0}, false, 0) == -ESHUTDOWN);
	assert(sensor_calibration_fusion_stale());
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_FRAME_CHANGED);
	sensor_calibration_set_consumer_ready(false);
	sensor_calibration_set_consumer_ready(true);
	assert(sensor_calibration_fusion_stale());
	sensor_calibration_fusion_applied();
	assert(!sensor_calibration_fusion_stale());
	/* The unacknowledged FRAME_CHANGED above must not consume the save request. */
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_SAVE_FUSION);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_UNCHANGED);
	assert(sensor_calibration_commit_bias(zero, zero, true, 0, (struct led_token){0}, 0, 0) == -ESHUTDOWN);
	assert(sensor_calibration_reset_accel() == -ESHUTDOWN);
}

static void test_storage_failure_keeps_applied_ram_and_reports_error(void)
{
	baseline();
	const float gyro_bias[3] = {7.0f, 8.0f, 9.0f};
	storage_result = -ENOSPC;
	assert(sensor_calibration_commit_bias(zero, gyro_bias, true, 42, (struct led_token){0}, 0, 0) == 0);
	assert(event_count == 0);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_BIAS_CHANGED);
	assert(event_count == 1);
	assert_event(0, 42, CAL_EVENT_END, CAL_OUTCOME_SUCCESS, CAL_PHASE_APPLIED, CAL_REASON_NONE);
	assert(storage_writes == 0);
	assert(error_logs == 0);
	vector_equal(retained->gyroBias, zero);
	assert(sensor_calibration_commit_bias(zero, zero, true, 0, (struct led_token){0}, 0, 0) == -EBUSY);
	sensor_imu_calibration_t applied = snapshot();
	vector_equal(applied.gyro_bias, gyro_bias);
	float gyro[3] = {10.0f, 10.0f, 10.0f};
	sensor_calibration_subtract_gyro_bias(gyro);
	const float corrected[3] = {3.0f, 2.0f, 1.0f};
	vector_equal(gyro, corrected);
	sensor_calibration_persist_pending();
	assert(storage_writes == 1);
	assert(error_logs == 1);
	assert(logged_error == -ENOSPC);
	assert(event_count == 2);
	assert_event(1, 42, CAL_EVENT_STEP, CAL_OUTCOME_NONE, CAL_PHASE_STORAGE, CAL_REASON_STORAGE_ERROR);
	vector_equal(retained->gyroBias, gyro_bias);
	sensor_imu_calibration_t persisted = snapshot();
	snapshot_equal(&persisted, &applied);
	assert(sensor_calibration_fusion_stale());
	sensor_calibration_fusion_applied();
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_SAVE_FUSION);
	assert(sensor_calibration_apply_pending() == SENSOR_CALIBRATION_UNCHANGED);
	/* Failed persistence does not leave the transaction slot permanently busy. */
	storage_result = 0;
	assert(sensor_calibration_reset_imu() == 0);
	apply_and_ack(SENSOR_CALIBRATION_FRAME_CHANGED);
	assert(event_count == 2); /* Storage and a silent reset cannot reopen the operation. */
}

static void run_isolated(void (*test)(void))
{
	pid_t child = fork();
	assert(child >= 0);
	if (child == 0) {
		test();
		_exit(0);
	}
	int status;
	assert(waitpid(child, &status, 0) == child);
	assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);
}

int main(void)
{
	test_stored_matrix_sanity();
	run_isolated(test_nonfinite_startup);
	run_isolated(test_out_of_bounds_startup);
	test_startup_and_load_once();
	test_pending_bias_atomic_application_and_busy();
	test_invalid_candidates_leave_applied_unchanged();
	test_matrix_commit_sample_boundary();
	test_reset_clears_boot_offset_at_application();
	run_isolated(test_led_bias_token_completes_after_frame_and_storage);
	run_isolated(test_led_storage_failure_reports_applied_not_saved);
	run_isolated(test_led_prior_storage_error_is_not_reported_as_success);
	run_isolated(test_led_bookkeeping_error_before_first_frame_is_retained);
	run_isolated(test_led_partial_accel_completion);
	run_isolated(test_led_frame_receipt_survives_reopened_admission);
	run_isolated(test_led_frame_receipt_keeps_failed_storage);
	run_isolated(test_led_stale_generation_candidate_is_refused);
	run_isolated(test_led_reset_session_owns_its_token);
	run_isolated(test_power_close_is_permanent_and_fusion_ack);
	test_storage_failure_keeps_applied_ram_and_reports_error();
	puts("IMU calibration transaction host tests passed");
	return 0;
}
