#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "globals.h"
#include "../../../src/sensor/calibration/online_mag.c"

static struct retained_fixture storage;
struct retained_fixture *retained = &storage;
float magBAinv[4][3];
uint8_t magneto_progress;
#ifdef REAL_FIT
#define FIT_HAS_STARTED true
#else
#define FIT_HAS_STARTED (solver_calls != 0)
#endif
static uint32_t clock_ms;
static unsigned dirty_marks, transaction, feeds, solver_calls;
static unsigned log_count, storage_writes;
static bool automatic_sensor = true;
static int cancel_action;
static float last_raw[3], last_up[3];
static bool last_valid;
static float fusion_norm, fusion_dip;
static const float perfect[4][3] = {{.08f, -.04f, .03f}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
static uint16_t next_operation;
static unsigned ended_events;
static struct {
	uint16_t operation;
	uint8_t outcome, phase, reason;
	float live[4][3];
	unsigned dirty;
} last_end, prior_end;

uint16_t cal_event_begin(uint8_t kind, uint8_t phase, uint8_t detail)
{
	assert(kind == (CAL_KIND_MAG_ONLINE | CAL_EVENT_ORIGIN_AUTO));
	assert(phase == CAL_PHASE_FREEZE && detail == 0);
	if (++next_operation == 0) {
		++next_operation;
	}
	return next_operation;
}
void cal_event_step(uint16_t op, uint8_t phase, uint8_t detail)
{
	(void)op;
	(void)phase;
	(void)detail;
}
void cal_event_end(uint16_t op, uint8_t outcome, uint8_t phase, uint8_t reason)
{
	if (!op) {
		return;
	}
	prior_end = last_end;
	++ended_events;
	last_end.operation = op;
	last_end.outcome = outcome;
	last_end.phase = phase;
	last_end.reason = reason;
	memcpy(last_end.live, magBAinv, sizeof(last_end.live));
	last_end.dirty = dirty_marks;
}
void tracker_events_notify(void)
{
	assert(!online_lock.held && !transaction);
}

void fixture_log(const char *format, ...)
{
	(void)format;
	assert(!online_lock.held && !transaction);
	++log_count;
}

k_spinlock_key_t k_spin_lock(struct k_spinlock *lock)
{
	assert(!lock->held);
	lock->held = 1;
	return 0;
}
void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
	(void)key;
	assert(lock->held);
	lock->held = 0;
}
uint32_t k_uptime_get_32(void)
{
	return clock_ms;
}
void *k_malloc(size_t size)
{
	return malloc(size);
}
void k_free(void *p)
{
	free(p);
}
void watchdog_feed(int channel)
{
	(void)channel;
	++feeds;
}
void sys_warm_transaction_begin(void)
{
	assert(!online_lock.held && !transaction);
	transaction = 1;
}
void sys_warm_transaction_end(bool changed)
{
	(void)changed;
	assert(transaction);
	transaction = 0;
}
void sys_warm_transaction_mark(int id, void *p, size_t size)
{
	(void)p;
	(void)size;
	assert(transaction && id == MAIN_MAG_BIAS_ID);
	++dirty_marks;
}
void sys_write(int id, void *dst, const void *src, size_t size)
{
	(void)id;
	++storage_writes;
	memcpy(dst, src, size);
}
void sensor_refresh_sensor_ids(void)
{
}
bool sensor_fusion_get_mag_ref(float *norm, float *dip)
{
	assert(!online_lock.held);
	*norm = fusion_norm;
	*dip = fusion_dip;
	return fusion_norm > 0;
}
#include ".generated/production.inc"

void fixture_reset(uint32_t now, int trusted)
{
	memset(&online, 0, sizeof(online));
	memset(&online_lock, 0, sizeof(online_lock));
	sensor_calibration_set_online_mag_debug(false); /* Simulate a fresh boot. */
	memset(pool, 0, sizeof(pool));
	memset(counts, 0, sizeof(counts));
	memset(heads, 0, sizeof(heads));
	memset(directions, 0, sizeof(directions));
	memset(&storage, 0, sizeof(storage));
	clock_ms = now;
	dirty_marks = transaction = feeds = solver_calls = 0;
	log_count = storage_writes = 0;
	next_operation = 0;
	ended_events = 0;
	memset(&last_end, 0, sizeof(last_end));
	memset(&prior_end, 0, sizeof(prior_end));
	cancel_action = 0;
	automatic_sensor = true;
	fusion_norm = trusted ? .5f : 0;
	fusion_dip = -asinf(.6f);
	magneto_progress = 0;
	float initial[4][3] = {0};
	if (trusted) {
		initial[1][0] = initial[2][1] = initial[3][2] = 1.0f;
		initial[0][0] = .02f;
	}
	memcpy(storage.magBAinv, initial, sizeof(initial));
	magneto_online_replace_BAinv_and_reset(initial, 0);
	magneto_online_runtime_configure(true);
	last_raw[0] = .5f;
	last_raw[1] = last_raw[2] = 0;
	last_up[0] = .6f;
	last_up[1] = .8f;
	last_up[2] = 0;
	last_valid = true;
}

void fixture_feed(const float raw[3], const float up[3], int valid, uint32_t dt)
{
	clock_ms += dt;
	memcpy(last_raw, raw, sizeof(last_raw));
	memcpy(last_up, up, sizeof(last_up));
	last_valid = valid;
	sensor_calibration_online_mag_sample(raw, up, valid);
	float norm, dip;
	if (magneto_online_take_mag_ref(&norm, &dip)) {
		fusion_norm = norm;
		fusion_dip = dip;
	}
}

void fixture_reference(float norm, float dip)
{
	fusion_norm = norm;
	fusion_dip = dip;
}

void fixture_model(const float value[12])
{
	memcpy(storage.magBAinv, value, sizeof(storage.magBAinv));
	magneto_online_replace_BAinv_and_reset((const float (*)[3])value, 0);
}
void fixture_clear(void)
{
	sensor_calibration_clear_mag(NULL, true);
}
int fixture_outcome(void)
{
	struct online_mag_diagnostics d;
	sensor_calibration_online_mag_diagnostics(&d);
	return d.outcome;
}
int fixture_dip_known(void)
{
	return online.dip_known;
}
float fixture_reference_norm(void)
{
	return fusion_norm;
}
float fixture_reference_dip(void)
{
	return fusion_dip;
}

/* Exercise the actual boot/read call ordering with a confirmed zero-bias
 * identity, including read while an already-started sensor queues replacement. */
void fixture_restore_identity(int sensor_started)
{
	if (sensor_started) {
		fixture_feed(last_raw, last_up, 1, 40);
	}
	float identity[4][3] = {{0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	memcpy(storage.magBAinv, identity, sizeof(identity));
	storage.onlineMagState.update_count = 1;
	storage.onlineMagState.last_buf_avg_norm = .5f;
	magneto_online_replace_BAinv_and_reset(storage.magBAinv, 0);
	magneto_online_runtime_configure(true);
	magneto_online_runtime_load_retained();
}

void k_msleep(unsigned ms)
{
	assert(!online_lock.held);
	clock_ms += ms;
	if (cancel_action && online.phase == FROZEN && FIT_HAS_STARTED) {
		int action = cancel_action;
		cancel_action = 0;
		if (action == 1) {
			magneto_online_reset();
		}
		if (action == 2) {
			sensor_calibration_set_online_mag_enabled(false);
		}
		if (action == 3) {
			sensor_calibration_online_mag_prepare_power_down();
		}
		if (action == 4) {
			magneto_online_replace_BAinv_and_reset(perfect, 0);
		}
	}
	if (automatic_sensor) {
		sensor_calibration_online_mag_sample(last_raw, last_up, last_valid);
	}
}

int fixture_check(void)
{
	return sensor_calibration_online_mag_check();
}
int fixture_phase(void)
{
	return online.phase;
}
int fixture_count(void)
{
	return sensor_calibration_online_mag_status(NULL);
}
unsigned fixture_dirty(void)
{
	return dirty_marks;
}
unsigned fixture_updates(void)
{
	return cal_online_mag_update_count();
}
void fixture_live(float out[12])
{
	magneto_online_snapshot_BAinv((float (*)[3])out);
}
void fixture_retained(float out[12])
{
	memcpy(out, storage.magBAinv, sizeof(storage.magBAinv));
}
void fixture_advance(uint32_t dt)
{
	clock_ms += dt;
}
void fixture_cancel(int action)
{
	cancel_action = action;
}

/* Observable pool freshness, without exposing or mutating lifecycle policy. */
unsigned fixture_pool_newer_than(uint32_t when)
{
	unsigned count = 0;
	for (unsigned i = 0; i < ONLINE_SLOTS; ++i) {
		count += occupied(i) && (int32_t)(pool[i].time - when) > 0;
	}
	return count;
}
uint32_t fixture_now(void)
{
	return clock_ms;
}
float fixture_old_rms(void)
{
	struct online_mag_diagnostics d;
	sensor_calibration_online_mag_diagnostics(&d);
	return d.old_rms;
}
float fixture_new_rms(void)
{
	struct online_mag_diagnostics d;
	sensor_calibration_online_mag_diagnostics(&d);
	return d.new_rms;
}
unsigned fixture_poles(void)
{
	struct online_mag_diagnostics d;
	sensor_calibration_online_mag_diagnostics(&d);
	return d.radial_poles;
}
unsigned fixture_rejection(void)
{
	struct online_mag_diagnostics d;
	sensor_calibration_online_mag_diagnostics(&d);
	return d.rejection;
}

#ifndef REAL_FIT
int magneto_robust_fit(
	unsigned slots,
	mag_fit_read_fn read,
	mag_fit_poll_fn poll,
	void *ctx,
	const float old[4][3],
	float field,
	float out[4][3],
	struct mag_fit_result *result
)
{
	(void)old;
	(void)field;
	++solver_calls;
	assert(online.phase == FROZEN && online.fitter && !online_lock.held);
	struct online_sample saved[ONLINE_SLOTS];
	memcpy(saved, pool, sizeof(saved));
	float before[4][3];
	magneto_online_snapshot_BAinv(before);
	for (unsigned i = 0; i < slots; i++) {
		float raw[3];
		(void)read(ctx, i, raw);
		bool valid = poll(ctx);
		assert(!memcmp(saved, pool, sizeof(saved))); /* cancellation must NOT clear borrowed raw */
		if (!valid) {
			return -ECANCELED;
		}
		float after[4][3];
		magneto_online_snapshot_BAinv(after);
		assert(!memcmp(before, after, sizeof(before))); /* fusion still has trusted output */
	}
	memcpy(out, perfect, sizeof(perfect));
	*result = (struct mag_fit_result){.field_norm = .5f, .condition = 2, .rms = 0};
	return 0;
}
#endif

#ifndef FIXTURE_LIBRARY
static void point(unsigned i, float raw[3], float up[3], bool upper)
{
	unsigned axis = i % 3, signs = (i / 3) % 8;
	float u[3];
	for (unsigned j = 0; j < 3; j++) {
		u[j] = (j == axis ? 1.0f : .3f) * ((signs & (1U << j)) ? -1 : 1);
	}
	if (upper) {
		u[2] = fabsf(u[2]);
	}
	float n = sqrtf(dot3(u, u));
	for (unsigned j = 0; j < 3; j++) {
		u[j] /= n;
	}
	float cross[3] = {-u[1], u[0], 0};
	n = sqrtf(dot3(cross, cross));
	for (unsigned j = 0; j < 3; j++) {
		up[j] = .6f * u[j] + .8f * cross[j] / n;
		raw[j] = .5f * u[j] + perfect[0][j];
	}
}
static void samples(unsigned count, bool upper, bool valid)
{
	for (unsigned i = 0; i < count; i++) {
		float r[3], u[3];
		point(i, r, u, upper);
		fixture_feed(r, u, valid, 40);
	}
}
static void train(void)
{
	samples(900, false, true);
	assert(fixture_count() >= MAG_CAL_MIN_SAMPLES);
	fixture_check();
	assert(fixture_phase() == VALIDATION_READY);
}
static void trial(void)
{
	train();
	samples(200, false, true);
	assert(fixture_phase() == PROBATION);
	assert(!dirty_marks && !memcmp(storage.magBAinv, previous, sizeof(previous)));
}
static void test_debug_logging(void)
{
	fixture_reset(1000, 0);
	assert(!sensor_calibration_get_online_mag_debug());
	trial();
	assert(log_count == 0 && !dirty_marks && !storage_writes);
	sensor_calibration_set_online_mag_debug(true);
	assert(sensor_calibration_get_online_mag_debug());
	assert(log_count == 0 && !dirty_marks && !storage_writes);
	float bad[3] = {2, 0, 0}, up[3] = {0, 0, 1};
	fixture_feed(bad, up, 1, 40);
	assert(log_count > 0 && fixture_phase() == TRAINING);
	magneto_online_runtime_reset();
	assert(sensor_calibration_get_online_mag_debug()); /* Reset is not a reboot. */
	unsigned before_fit = log_count;
	train();
	assert(log_count >= before_fit + 2); /* Successful fit start and result. */
	assert(!dirty_marks && !storage_writes);
	sensor_calibration_set_online_mag_debug(false);
	unsigned quiet_count = log_count;
	samples(800, false, true);
	assert(fixture_phase() == CONFIRMATION_READY && !dirty_marks);
	assert(fixture_check() && dirty_marks == 1);
	assert(log_count == quiet_count && !storage_writes);
}

static void test_window_minimums(void)
{
	fixture_reset(1000, 0);
	train();
	samples(1, false, true);
	uint32_t start = clock_ms;
	for (unsigned i = 0; fixture_phase() == VALIDATING && i < 200; ++i) {
		float raw[3], up[3];
		point(i, raw, up, false);
		fixture_feed(raw, up, 1, 30);
	}
	assert(fixture_phase() == PROBATION);
	assert(ELAPSED(clock_ms, start) == 3000);
	start = clock_ms;
	for (unsigned i = 0; fixture_phase() == PROBATION && i < 300; ++i) {
		float raw[3], up[3];
		point(i, raw, up, false);
		fixture_feed(raw, up, 1, 40);
	}
	assert(fixture_phase() == CONFIRMATION_READY && !dirty_marks);
	assert(ELAPSED(clock_ms, start) == 10000);
}

static void test_cancel_frozen(void)
{
	for (int action = 1; action <= 4; action++) {
		fixture_reset(1000, 1);
		samples(900, false, true);
		cancel_action = action;
		fixture_check();
		assert(!online.fitter && !dirty_marks);
		assert(online.generation != online.served); /* release, then sensor can recycle */
		samples(1, false, true);
		assert(online.generation == online.served);
		if (action == 4) {
			assert(!memcmp(magBAinv, perfect, sizeof(perfect)));
		}
	}
	fixture_reset(1000, 1);
	samples(900, false, true);
	automatic_sensor = false;
	uint32_t started = clock_ms;
	fixture_check();
	assert(ELAPSED(clock_ms, started) <= ONLINE_FREEZE_TIMEOUT_MS + 1 && !online.fitter);
}
static void test_holdout_and_confirmation(void)
{
	fixture_reset(1000, 0);
	train();
	float initial[4][3];
	memcpy(initial, storage.magBAinv, sizeof(initial));
	/* Fits cannot use training as validation, even after enough wall time. */
	fixture_advance(ONLINE_VALIDATE_MS);
	fixture_check();
	assert(!memcmp(magBAinv, initial, sizeof(initial)) && !dirty_marks);
	samples(250, false, false);
	assert(fixture_phase() == PROBATION && !dirty_marks);
	assert(!memcmp(storage.magBAinv, initial, sizeof(initial)));
	assert(fusion_norm == 0); /* No fabricated horizontal field without gravity. */
	samples(400, false, false);
	assert(fixture_phase() == CONFIRMATION_READY && !dirty_marks);
	assert(ended_events == 0);
	assert(fixture_check());
	assert(dirty_marks == 1 && fixture_updates() == 1);
	assert(!memcmp(storage.magBAinv, perfect, sizeof(perfect)));
	assert(!online.dip_known);
	assert(ended_events == 1 && last_end.operation != 0);
	assert(last_end.outcome == CAL_OUTCOME_SUCCESS && last_end.phase == CAL_PHASE_CONFIRM);
	assert(last_end.dirty == 1 && !memcmp(last_end.live, perfect, sizeof(perfect)));
}
static void test_repeated_enable_preserves_trial(void)
{
	fixture_reset(1000, 1);
	sensor_calibration_set_online_mag_enabled(true);
	trial();
	float active[4][3];
	magneto_online_snapshot_BAinv(active);
	sensor_calibration_set_online_mag_enabled(true);
	samples(1, false, true);
	float after[4][3];
	magneto_online_snapshot_BAinv(after);
	assert(!memcmp(active, after, sizeof(active)));
	assert(!dirty_marks && !memcmp(storage.magBAinv, previous, sizeof(previous)));
	samples(400, false, true);
	assert(fixture_check());
	assert(dirty_marks == 1 && fixture_updates() == 1);
	assert(!memcmp(storage.magBAinv, active, sizeof(active)));
}
static void test_rejection_and_rollback(void)
{
	fixture_reset(1000, 1);
	trial();
	float old[4][3];
	memcpy(old, storage.magBAinv, sizeof(old));
	float bad[3] = {2, 0, 0}, up[3] = {0, 0, 1};
	fixture_feed(bad, up, 1, 40);
	assert(!memcmp(magBAinv, old, sizeof(old)) && !dirty_marks);
	fixture_reset(1000, 1);
	trial();
	fixture_advance(ONLINE_EPISODE_TIMEOUT_MS + 1);
	samples(1, false, true);
	assert(!memcmp(magBAinv, storage.magBAinv, sizeof(magBAinv)) && !dirty_marks);
	fixture_reset(1000, 1);
	trial();
	sensor_calibration_online_mag_prepare_power_down();
	sensor_calibration_online_mag_retained_save();
	assert(!memcmp(storage.magBAinv, previous, sizeof(previous)) && !dirty_marks);
	fixture_reset(1000, 0);
	train();
	samples(900, true, true);
	assert(fixture_phase() == VALIDATING && !dirty_marks); /* hemisphere lacks a pole */
	fixture_reset(1000, 0);
	train();
	fixture_feed(bad, up, 1, 40);
	fixture_feed(bad, up, 1, 40);
	assert(!memcmp(magBAinv, storage.magBAinv, sizeof(magBAinv)) && !dirty_marks);
	fixture_reset(1000, 1);
	trial();
	sensor_calibration_set_online_mag_enabled(false);
	assert(ended_events == 0);
	samples(1, false, true);
	assert(!memcmp(magBAinv, storage.magBAinv, sizeof(magBAinv)) && !dirty_marks);
	assert(ended_events == 1 && last_end.outcome == CAL_OUTCOME_CANCELLED);
	assert(last_end.reason == CAL_REASON_DISABLED);
	assert(!memcmp(last_end.live, storage.magBAinv, sizeof(magBAinv)));
	fixture_reset(1000, 1);
	trial();
	float manual[4][3];
	memcpy(manual, perfect, sizeof(manual));
	manual[0][0] = .06f;
	magneto_online_replace_BAinv_and_reset(manual, 0);
	assert(ended_events == 0);
	samples(1, false, true);
	assert(!memcmp(magBAinv, manual, sizeof(manual)) && !dirty_marks);
	assert(ended_events == 1 && last_end.outcome == CAL_OUTCOME_CANCELLED);
	assert(last_end.reason == CAL_REASON_REPLACED);
	assert(!memcmp(last_end.live, manual, sizeof(manual)));
	/* A manual result has its own token; retiring the old trial must not
	 * restore its previous matrix over the new model or complete it early. */
	fixture_reset(1000, 1);
	trial();
	uint16_t old_operation = online.operation;
	magneto_online_replace_BAinv_and_reset(manual, 100);
	assert(ended_events == 0);
	samples(1, false, true);
	assert(ended_events == 2);
	assert(prior_end.operation == old_operation && prior_end.outcome == CAL_OUTCOME_CANCELLED);
	assert(prior_end.reason == CAL_REASON_REPLACED && !memcmp(prior_end.live, manual, sizeof(manual)));
	assert(last_end.operation == 100 && last_end.outcome == CAL_OUTCOME_SUCCESS);
	assert(last_end.phase == CAL_PHASE_APPLIED && !memcmp(last_end.live, manual, sizeof(manual)));
	/* Overwriting an unapplied manual result cancels only that result. */
	fixture_reset(1000, 1);
	samples(1, false, true);
	magneto_online_replace_BAinv_and_reset(perfect, 100);
	magneto_online_replace_BAinv_and_reset(manual, 101);
	assert(ended_events == 1 && last_end.operation == 100 && last_end.outcome == CAL_OUTCOME_CANCELLED);
	samples(1, false, true);
	assert(ended_events == 2 && last_end.operation == 101 && last_end.outcome == CAL_OUTCOME_SUCCESS);
	assert(!memcmp(last_end.live, manual, sizeof(manual)));
	/* No sensor callback arrives to service timeout before the delayed worker.
	 * A completed probation must not authorize stale retained publication. */
	fixture_reset(1000, 0);
	train();
	samples(800, false, true);
	assert(fixture_phase() == CONFIRMATION_READY);
	fixture_advance(ONLINE_EPISODE_TIMEOUT_MS + 1);
	assert(!fixture_check() && !dirty_marks);
	assert(!memcmp(storage.magBAinv, previous, sizeof(previous)));
	samples(1, false, true);
	assert(!memcmp(magBAinv, storage.magBAinv, sizeof(magBAinv)));
}
static void test_ttl_wrap(void)
{
	fixture_reset(UINT32_MAX - 10000U, 0);
	samples(900, false, true);
	assert(fixture_count() >= MAG_CAL_MIN_SAMPLES);
	fixture_advance(ONLINE_TTL_MS + 1001U);
	samples(1, false, true);
	assert(fixture_count() == 0); /* stale cache refreshed before the one new insert */
}
int main(void)
{
	assert(!sensor_calibration_get_online_mag_debug()); /* Static boot default. */
	test_debug_logging();
	test_window_minimums();
	test_cancel_frozen();
	test_holdout_and_confirmation();
	test_repeated_enable_preserves_trial();
	test_rejection_and_rollback();
	test_ttl_wrap();
	puts("online_mag lifecycle: freeze/cancel, independent holdout, confirmation, rollback, TTL passed");
	return 0;
}
#endif
