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
#include "sensor/sensor.h"
#include "system/system.h"
#include "util.h"

#include <math.h>
#include <string.h>
#include <zephyr/sys/atomic.h>

#include "sensor/magneto/magneto1_4.h"

#include "calibration.h"
#include "mag_common.h"
#include "online_mag.h"

LOG_MODULE_REGISTER(cal_online_mag, LOG_LEVEL_INF);

static mag_center_estimator_t online_center_estimator;

// Per-quadrant ring buffer for online magnetometer calibration.
// Combines the directional coverage guarantee of quadrant-based sampling
// (8 octants based on sign of x, y, z) with the natural aging of FIFO
// per-quadrant sliding windows. Each octant independently wraps after
// QUADRANT_BUF_SIZE samples — staying in one orientation only updates
// that octant, leaving the other 7 with diverse data.
#define QUADRANT_BUF_SIZE 32
#define ONLINE_QUADRANT_COUNT 8
#define ONLINE_BUFFER_SAMPLE_CAPACITY (ONLINE_QUADRANT_COUNT * QUADRANT_BUF_SIZE)

typedef struct {
	float x, y, z;
} quadrant_sample_t;

typedef struct {
	quadrant_sample_t samples[QUADRANT_BUF_SIZE];
	uint8_t head;   // next write position
	uint8_t count;  // valid samples (0..QUADRANT_BUF_SIZE)
	uint32_t last_seq; // global accepted-sample sequence of the newest sample in this octant
} quadrant_buf_t;

// Incremental calibration blending (EMA on BAinv elements)
// Base alpha: weight given to new trial calibration. A value of 0.35 means
// 35% new + 65% existing → gradual convergence over ~3 updates.
// Higher when trial diverges significantly from existing (environment change).
#define ONLINE_BLEND_BASE_ALPHA 0.35f
#define ONLINE_BLEND_MIN_ALPHA 0.12f   // floor: very similar calibrations
#define ONLINE_BLEND_MAX_ALPHA 0.70f   // ceiling: significant divergence detected
#define ONLINE_BLEND_SIMILARITY_LOW 0.85f   // below this similarity, increase alpha
#define ONLINE_BLEND_SIMILARITY_HIGH 0.97f  // above this similarity, use min alpha

static quadrant_buf_t quad_buf[ONLINE_QUADRANT_COUNT];
/* Cal-thread snapshot (~3KB static: too big for stack). */
static quadrant_buf_t quad_buf_snap[ONLINE_QUADRANT_COUNT];
static K_MUTEX_DEFINE(quad_buf_snap_lock);
static K_MUTEX_DEFINE(quad_buf_lock);
/*
 * The sensor thread is the only writer, but readers can run concurrently on an
 * SMP target. A mutex protects the ordinary C objects from data races without
 * keeping interrupts disabled for the ~3 KB snapshot copy. Clearing history
 * only bumps quad_buf_gen; the sensor thread applies that reset on its next
 * sample and mirrors the value into quad_buf_gen_served. Until then, readers
 * treat the whole buffer as empty.
 */
static atomic_t quad_buf_gen;        /* bumped by any thread to discard history */
static atomic_t quad_buf_gen_served; /* stored by the sensor thread only */
static struct k_spinlock online_publish_lock;
/*
 * Every scalar below is shared across the sensor (7), calibration (8), console
 * (6), ESB (5) and power (6) threads, so each one is a single 32-bit word held in
 * an atomic_t rather than an int64_t: this is a 32-bit Cortex-M, where a 64-bit
 * load or store is two instructions and can tear across a preemption.
 *
 * Sample counters are free-running uint32 and are compared with wrap-safe
 * unsigned subtraction. Timestamps are uptime milliseconds truncated to uint32
 * and compared with ONLINE_ELAPSED / ONLINE_TIME_GE, which use a signed 32-bit
 * difference and therefore stay correct across the ~49.7 day wrap.
 */
static atomic_t online_total_sample_count;        /* uint32; only the sensor thread stores */
static atomic_t online_last_checked_sample_count; /* uint32 */
static atomic_t online_last_check_time;           /* uint32 ms, 0 = never */
static atomic_t online_last_sample_time;          /* uint32 ms, 0 = never (rate limiting) */

/* Wrap-safe uptime-millisecond helpers. Both operands must be uint32 ms. */
#define ONLINE_ELAPSED(now, since) ((uint32_t)((uint32_t)(now) - (uint32_t)(since)))
#define ONLINE_TIME_GE(a, b) ((int32_t)((uint32_t)(a) - (uint32_t)(b)) >= 0)

// Drop octants that have not been refreshed for too long.
// This is kept separate from the check cadence: stale-history rejection should
// not depend on how often the background thread decides to run Magneto.
#define ONLINE_STALE_QUADRANT_MAX_AGE (ONLINE_BUFFER_SAMPLE_CAPACITY * 5 / 2)
// Minimum direction change to accept an online sample. The configured value is
// expressed in degrees and converted to the equivalent 1 - cos(theta) threshold.
static float online_last_dir[3];
static float online_last_accel_dir[3]; // accel direction for cross-validation

#define ONLINE_MIN_DIR_CHANGE_DEG 10.0f
#define ONLINE_MIN_INTERVAL_MS 30  // minimum 30ms between online samples
// Background checks should not run on every calibration-thread pass.
// Tie the minimum check spacing to roughly one fresh fit's worth of accepted
// samples at the maximum online sampling rate.
#define ONLINE_MIN_CHECK_INTERVAL_MS (ONLINE_BUFFER_SAMPLE_CAPACITY * ONLINE_MIN_INTERVAL_MS)

typedef struct {
	float cal_norm_ema;
	float cal_norm_var_ema;
	uint32_t cal_norm_count;
	int update_count;
	float last_buf_avg_norm;
} online_runtime_state_t;

static online_runtime_state_t online_runtime_state;
static struct k_spinlock online_runtime_state_lock;

#define CAL_NORM_EMA_ALPHA 0.01f  // smoothing factor (~100 sample window)
// Don't update calibration if current norm CV is below this threshold
#define CAL_NORM_GOOD_CV 0.05f    // 5% = good enough calibration

// Minimum time between online calibration updates (prevents frequent VQF mag ref resets)
#define ONLINE_MIN_UPDATE_INTERVAL_S 6  // 6 seconds cooldown
static atomic_t online_last_update_time; /* uint32 ms, 0 = never */

// Suppress online sample collection for N ms after buffer resets (wake-up,
// reboot, environment change, calibration update).  This lets sensor data
// stabilise before collecting, avoiding transient/mixed-environment samples
// that produce poor calibration fits.
#define ONLINE_COLLECTION_SUPPRESS_MS 1500
static atomic_t online_collection_suppress_until; /* uint32 ms, 0 = not suppressed */

// Minimum sustained VQF magnetic disturbance duration before allowing an online
// calibration update.  Short disturbance bursts are usually transient interference;
// updating calibration during those resets VQF's heading reference for no benefit.
#define ONLINE_VQF_DIST_MIN_DURATION_MS 3000
static atomic_t online_mag_dist_start_time; /* uint32 ms, 0 = no disturbance */
static atomic_t online_commits_suspended;
static atomic_t online_enabled;

// Require at least N successful calibration updates before trusting the
// norm CV gate. Prevents a single early fit from being declared "good enough"
// when the buffer is still filling and directional coverage is incomplete.
#define ONLINE_MIN_UPDATES 3


typedef struct {
	uint32_t total; /* accepted-sample counter at the instant of the copy */
	unsigned gen;   /* generation the snapshot belongs to (only if valid) */
	bool valid;     /* false: snapshot was zeroed, nothing usable in it */
} quad_buf_snapshot_t;

static void magneto_online_clear_history_at_locked(uint32_t now_ms);
static void magneto_online_clear_progress(uint32_t now_ms);
static double magneto_online_collect_recent(double ata_out[100], double *norm_sum_out,
					   float dir_sum_out[3], float *raw_range_out,
					   quad_buf_snapshot_t *snap_out);
static int magneto_online_recent_sample_count(void);
static float magneto_online_recent_dir_bias(void);
static float magneto_directional_bias(const float ds[3], double count);
static void magneto_accumulate_direction(float ds[3], const float v[3]);
static float magneto_BAinv_similarity(float existing[4][3], float candidate[4][3]);
static bool magneto_blend_BAinv(float out[4][3], float existing[4][3], float candidate[4][3]);

static online_runtime_state_t magneto_online_runtime_state_snapshot(void)
{
	k_spinlock_key_t key = k_spin_lock(&online_runtime_state_lock);
	online_runtime_state_t state = online_runtime_state;
	k_spin_unlock(&online_runtime_state_lock, key);
	return state;
}

static void magneto_online_calibration_state_snapshot(float BAinv_out[4][3],
						       online_runtime_state_t *runtime_out,
						       unsigned *generation_out)
{
	k_spinlock_key_t publish_key = k_spin_lock(&online_publish_lock);
	memcpy(BAinv_out, magBAinv, sizeof(magBAinv));
	*generation_out = (unsigned)atomic_get(&quad_buf_gen);
	k_spinlock_key_t state_key = k_spin_lock(&online_runtime_state_lock);
	*runtime_out = online_runtime_state;
	k_spin_unlock(&online_runtime_state_lock, state_key);
	k_spin_unlock(&online_publish_lock, publish_key);
}

static float magneto_online_mag_quality_from_state(const online_runtime_state_t *state)
{
	if (state->cal_norm_count < 100 || state->cal_norm_ema < 1e-6f) {
		return 1.0f;
	}
	return sqrtf(state->cal_norm_var_ema) / state->cal_norm_ema;
}

static void magneto_online_norm_state_reset(void)
{
	k_spinlock_key_t key = k_spin_lock(&online_runtime_state_lock);
	online_runtime_state.cal_norm_count = 0;
	online_runtime_state.cal_norm_ema = 0.0f;
	online_runtime_state.cal_norm_var_ema = 0.0f;
	k_spin_unlock(&online_runtime_state_lock, key);
}

int cal_online_mag_update_count(void)
{
	return magneto_online_runtime_state_snapshot().update_count;
}

uint32_t cal_online_mag_norm_count(void)
{
	return magneto_online_runtime_state_snapshot().cal_norm_count;
}

void magneto_online_runtime_load_retained(void)
{
	k_spinlock_key_t publish_key = k_spin_lock(&online_publish_lock);
	magneto_online_clear_history_at_locked(k_uptime_get_32());
	atomic_set(&online_last_update_time, 0);
	k_spinlock_key_t key = k_spin_lock(&online_runtime_state_lock);
	memset(&online_runtime_state, 0, sizeof(online_runtime_state));
	online_runtime_state.update_count = retained->onlineMagState.update_count;
	online_runtime_state.last_buf_avg_norm = retained->onlineMagState.last_buf_avg_norm;
	k_spin_unlock(&online_runtime_state_lock, key);
	k_spin_unlock(&online_publish_lock, publish_key);
}

bool sensor_calibration_get_online_mag_enabled(void)
{
	return atomic_get(&online_enabled) != 0;
}

void sensor_calibration_set_online_mag_enabled(bool enabled)
{
	uint8_t mode = enabled ? MAG_ONLINE_CALIBRATION_ENABLED : MAG_ONLINE_CALIBRATION_DISABLED;

	if (sensor_calibration_get_online_mag_enabled() == enabled &&
	    retained->mag_online_calibration_mode == mode) {
		LOG_INF("Online mag calibration already %s", enabled ? "enabled" : "disabled");
		return;
	}

	magneto_online_runtime_configure(enabled);
	if (!enabled) {
		sensor_calibration_online_mag_retained_clear();
	}
	sys_write(
		MAG_ONLINE_CALIBRATION_ID,
		&retained->mag_online_calibration_mode,
		&mode,
		sizeof(mode)
	);
	LOG_INF("Online mag calibration %s (persisted)", enabled ? "enabled" : "disabled");
}

void sensor_calibration_online_mag_retained_save(void)
{
	sys_warm_transaction_begin();
	online_runtime_state_t state = magneto_online_runtime_state_snapshot();
	if (!sensor_calibration_get_online_mag_enabled()) {
		memset(&retained->onlineMagState, 0, sizeof(retained->onlineMagState));
	} else {
		retained->onlineMagState.update_count = (uint8_t)CLAMP(state.update_count, 0, 255);
		retained->onlineMagState.last_buf_avg_norm = state.last_buf_avg_norm;
	}
	sys_warm_transaction_end(true);
}

void sensor_calibration_online_mag_retained_clear(void)
{
	sys_warm_transaction_begin();
	memset(&retained->onlineMagState, 0, sizeof(retained->onlineMagState));
	sys_warm_transaction_end(true);
}

void sensor_calibration_online_mag_cold_start(void)
{
	magneto_online_runtime_reset();
	sensor_calibration_online_mag_retained_clear();
}


void magneto_online_reset(void)
{
	k_spinlock_key_t key = k_spin_lock(&online_publish_lock);
	magneto_online_clear_history_at_locked(k_uptime_get_32());
	/* online_last_sample_time belongs to the sensor thread; the buffer clear it
	 * services resets it (magneto_online_service_clear). */
	atomic_set(&online_last_update_time, 0);
	k_spin_unlock(&online_publish_lock, key);
}

void magneto_online_runtime_reset(void)
{
	k_spinlock_key_t publish_key = k_spin_lock(&online_publish_lock);
	magneto_online_clear_history_at_locked(k_uptime_get_32());
	atomic_set(&online_last_update_time, 0);
	k_spinlock_key_t state_key = k_spin_lock(&online_runtime_state_lock);
	memset(&online_runtime_state, 0, sizeof(online_runtime_state));
	k_spin_unlock(&online_runtime_state_lock, state_key);
	k_spin_unlock(&online_publish_lock, publish_key);
}

void magneto_online_runtime_configure(bool enabled)
{
	k_spinlock_key_t publish_key = k_spin_lock(&online_publish_lock);
	atomic_set(&online_enabled, enabled ? 1 : 0);
	magneto_online_clear_history_at_locked(k_uptime_get_32());
	atomic_set(&online_last_update_time, 0);
	k_spinlock_key_t state_key = k_spin_lock(&online_runtime_state_lock);
	memset(&online_runtime_state, 0, sizeof(online_runtime_state));
	k_spin_unlock(&online_runtime_state_lock, state_key);
	k_spin_unlock(&online_publish_lock, publish_key);
}

static void magneto_online_clear_history_at_locked(uint32_t now_ms)
{
	/*
	 * Discard every buffered sample without touching quad_buf: the sensor
	 * thread owns that memory and services the reset on its next sample
	 * (magneto_online_service_clear). Readers observe an empty buffer
	 * immediately, before the reset physically happens.
	 * online_last_dir / online_last_accel_dir / online_center_estimator are
	 * sensor-thread state too, so they are reset there for the same reason.
	 *
	 * The generation bump comes first and doubles as the cancellation point for
	 * an online fit that is already in flight: such a fit re-validates the
	 * generation of the samples it was built from before it publishes magBAinv
	 * (magneto_online_commit_BAinv), so whoever bumps first wins.
	 */
	atomic_inc(&quad_buf_gen);
	atomic_set(&online_last_checked_sample_count, 0);
	atomic_set(&online_last_check_time, 0);
	// Suppress collection for a few seconds so transient/stale samples from
	// wake-up, reboot, or environment transitions are not mixed into the
	// fresh buffer.
	atomic_set(&online_collection_suppress_until,
	           (atomic_val_t)(now_ms + ONLINE_COLLECTION_SUPPRESS_MS));
}

static void magneto_online_clear_progress(uint32_t now_ms)
{
	k_spinlock_key_t publish_key = k_spin_lock(&online_publish_lock);
	magneto_online_clear_history_at_locked(now_ms);
	k_spinlock_key_t state_key = k_spin_lock(&online_runtime_state_lock);
	online_runtime_state.update_count = 0;
	online_runtime_state.last_buf_avg_norm = 0.0f;
	k_spin_unlock(&online_runtime_state_lock, state_key);
	k_spin_unlock(&online_publish_lock, publish_key);
}

void magneto_online_snapshot_BAinv(float out[4][3])
{
	k_spinlock_key_t key = k_spin_lock(&online_publish_lock);
	memcpy(out, magBAinv, sizeof(magBAinv));
	k_spin_unlock(&online_publish_lock, key);
}

void magneto_online_replace_BAinv_and_reset(const float replacement[4][3])
{
	k_spinlock_key_t publish_key = k_spin_lock(&online_publish_lock);
	memcpy(magBAinv, replacement, sizeof(magBAinv));
	magneto_online_clear_history_at_locked(k_uptime_get_32());
	atomic_set(&online_last_update_time, 0);
	k_spinlock_key_t state_key = k_spin_lock(&online_runtime_state_lock);
	memset(&online_runtime_state, 0, sizeof(online_runtime_state));
	k_spin_unlock(&online_runtime_state_lock, state_key);
	k_spin_unlock(&online_publish_lock, publish_key);
}

void sensor_calibration_online_mag_prepare_power_down(void)
{
	k_spinlock_key_t key = k_spin_lock(&online_publish_lock);
	atomic_set(&online_commits_suspended, 1);
	atomic_inc(&quad_buf_gen);
	k_spin_unlock(&online_publish_lock, key);
}

/* Sensor thread only: apply a pending clear to the buffer this thread owns. */
static void magneto_online_service_clear(void)
{
	unsigned gen = (unsigned)atomic_get(&quad_buf_gen);
	if (gen == (unsigned)atomic_get(&quad_buf_gen_served)) {
		return;
	}

	k_mutex_lock(&quad_buf_lock, K_FOREVER);
	gen = (unsigned)atomic_get(&quad_buf_gen);
	memset(quad_buf, 0, sizeof(quad_buf));
	atomic_set(&online_total_sample_count, 0);
	atomic_set(&quad_buf_gen_served, (atomic_val_t)gen);
	k_mutex_unlock(&quad_buf_lock);

	atomic_set(&online_last_sample_time, 0); /* let the next sample through at once */
	magneto_center_reset(&online_center_estimator);
	memset(online_last_dir, 0, sizeof(online_last_dir));
	memset(online_last_accel_dir, 0, sizeof(online_last_accel_dir));
}

static bool magneto_online_quadrant_is_recent_at(const quadrant_buf_t *qbuf, uint32_t total_count)
{
	if (qbuf->count == 0) {
		return false;
	}
	/* Unsigned difference: correct across the uint32 counter wrap. */
	return (uint32_t)(total_count - qbuf->last_seq) <= ONLINE_STALE_QUADRANT_MAX_AGE;
}

/*
 * Thread-context snapshot of quad_buf. The mutex may briefly delay the sensor
 * thread while the ~3 KB copy completes, with Zephyr priority inheritance
 * preventing unbounded priority inversion. Unlike a seqlock over ordinary C
 * objects, this remains data-race-free when reader and writer run on different
 * CPUs.
 *
 * quad_buf_gen is re-read after the copy because a clear request does not take
 * quad_buf_lock or touch quad_buf. If the generation changed during the copy,
 * the user has already discarded that history and the snapshot is invalid.
 *
 * On success .gen carries the generation the samples belong to. A caller that
 * derives a calibration from the snapshot passes it back to
 * magneto_online_commit_BAinv() as a cancellation token.
 */
/* Caller must hold quad_buf_snap_lock. */
static quad_buf_snapshot_t magneto_online_quad_buf_snapshot_locked(void)
{
	unsigned gen = (unsigned)atomic_get(&quad_buf_gen);
	if (gen != (unsigned)atomic_get(&quad_buf_gen_served)) {
		memset(quad_buf_snap, 0, sizeof(quad_buf_snap));
		return (quad_buf_snapshot_t){.total = 0, .gen = 0, .valid = false};
	}

	k_mutex_lock(&quad_buf_lock, K_FOREVER);
	gen = (unsigned)atomic_get(&quad_buf_gen);
	unsigned served = (unsigned)atomic_get(&quad_buf_gen_served);
	uint32_t total = (uint32_t)atomic_get(&online_total_sample_count);
	bool valid = gen == served;
	if (valid) {
		memcpy(quad_buf_snap, quad_buf, sizeof(quad_buf));
		valid = (unsigned)atomic_get(&quad_buf_gen) == gen;
	}
	k_mutex_unlock(&quad_buf_lock);

	if (valid) {
		return (quad_buf_snapshot_t){.total = total, .gen = gen, .valid = true};
	}

	memset(quad_buf_snap, 0, sizeof(quad_buf_snap));
	return (quad_buf_snapshot_t){.total = 0, .gen = 0, .valid = false};
}

static double magneto_online_recent_center_from_snap(uint32_t total_count, mag_center_estimator_t *center)
{
	magneto_center_reset(center);

	double count = 0;
	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent_at(&quad_buf_snap[q], total_count)) {
			continue;
		}
		for (int i = 0; i < quad_buf_snap[q].count; i++) {
			quadrant_sample_t *s = &quad_buf_snap[q].samples[i];
			float m[3] = {s->x, s->y, s->z};
			magneto_center_update(center, m);
			count++;
		}
	}

	return count;
}

// Collect all valid samples from all 8 quadrant ring buffers.
// Recomputes ATA, norm_sum, centered dir_sum, and raw min/max range from raw samples.
// *snap_out receives the snapshot descriptor, whose .gen the caller must pass to
// magneto_online_commit_BAinv() when it publishes a fit built from these samples.
static double magneto_online_collect_recent(double ata_out[100], double *norm_sum_out,
                                            float dir_sum_out[3], float *raw_range_out,
                                            quad_buf_snapshot_t *snap_out)
{
	memset(ata_out, 0, sizeof(double) * 100);
	*norm_sum_out = 0;
	memset(dir_sum_out, 0, sizeof(float) * 3);

	k_mutex_lock(&quad_buf_snap_lock, K_FOREVER);
	quad_buf_snapshot_t snap = magneto_online_quad_buf_snapshot_locked();
	uint32_t total = snap.total;

	mag_center_estimator_t recent_center;
	double recent_sample_count = magneto_online_recent_center_from_snap(total, &recent_center);
	if (raw_range_out) {
		*raw_range_out = magneto_center_min_range(&recent_center);
	}

	double fit_sample_count = 0;
	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent_at(&quad_buf_snap[q], total)) {
			continue;
		}
		for (int i = 0; i < quad_buf_snap[q].count; i++) {
			quadrant_sample_t *s = &quad_buf_snap[q].samples[i];
			magneto_sample((double)s->x, (double)s->y, (double)s->z, ata_out, norm_sum_out, &fit_sample_count);
			float raw[3] = {s->x, s->y, s->z};
			float coverage_sample[3];
			magneto_coverage_sample(&recent_center, raw, coverage_sample);
			magneto_accumulate_direction(dir_sum_out, coverage_sample);
		}
	}
	k_mutex_unlock(&quad_buf_snap_lock);
	*snap_out = snap;
	return recent_sample_count;
}

static int magneto_online_recent_sample_count(void)
{
	k_mutex_lock(&quad_buf_snap_lock, K_FOREVER);
	uint32_t total = magneto_online_quad_buf_snapshot_locked().total;
	int count = 0;
	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent_at(&quad_buf_snap[q], total)) {
			continue;
		}
		count += quad_buf_snap[q].count;
	}
	k_mutex_unlock(&quad_buf_snap_lock);
	return count;
}

static float magneto_online_recent_dir_bias(void)
{
	float dir_sum_recent[3] = {0};
	mag_center_estimator_t recent_center;

	k_mutex_lock(&quad_buf_snap_lock, K_FOREVER);
	uint32_t total = magneto_online_quad_buf_snapshot_locked().total;
	double recent_sample_count = magneto_online_recent_center_from_snap(total, &recent_center);

	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent_at(&quad_buf_snap[q], total)) {
			continue;
		}
		for (int i = 0; i < quad_buf_snap[q].count; i++) {
			quadrant_sample_t *s = &quad_buf_snap[q].samples[i];
			float raw[3] = {s->x, s->y, s->z};
			float coverage_sample[3];
			magneto_coverage_sample(&recent_center, raw, coverage_sample);
			magneto_accumulate_direction(dir_sum_recent, coverage_sample);
		}
	}
	k_mutex_unlock(&quad_buf_snap_lock);

	return magneto_directional_bias(dir_sum_recent, recent_sample_count);
}

float magneto_online_min_dir_change_threshold(void)
{
	static bool initialized = false;
	static float threshold = 0.0f;

	if (!initialized) {
		const float deg_to_rad = 0.01745329251994329577f;
		threshold = 1.0f - cosf(ONLINE_MIN_DIR_CHANGE_DEG * deg_to_rad);
		initialized = true;
	}

	return threshold;
}


/**
 * Compute directional bias of accumulated mag samples.
 * Returns |sum(m/|m|)| / N, where 0=perfect sphere coverage, 1=all same direction.
 */
static float magneto_directional_bias(const float ds[3], double count)
{
	if (count < 2) {
		return 1.0f;
	}
	float inv_n = 1.0f / (float)count;
	float cx = ds[0] * inv_n;
	float cy = ds[1] * inv_n;
	float cz = ds[2] * inv_n;
	return sqrtf(cx * cx + cy * cy + cz * cz);
}


/**
 * Accumulate a normalized direction for diversity tracking.
 */
static void magneto_accumulate_direction(float ds[3], const float v[3])
{
	float dir[3];
	if (!magneto_normalize_direction(v, dir)) {
		return;
	}
	ds[0] += dir[0];
	ds[1] += dir[1];
	ds[2] += dir[2];
}


/**
 * Compute similarity between two BAinv calibration matrices.
 * Uses normalized Frobenius norm: similarity = 1.0 - ||candidate - existing|| / ||existing||.
 * Returns 1.0 for identical calibrations, approaching 0 for very different ones.
 * The offset row (row 0) and soft-iron rows (1-3) contribute equally to the norm.
 */
static float magneto_BAinv_similarity(float existing[4][3], float candidate[4][3])
{
	float diff_norm_sq = 0;
	float existing_norm_sq = 0;

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 3; c++) {
			float d = candidate[r][c] - existing[r][c];
			diff_norm_sq += d * d;
			existing_norm_sq += existing[r][c] * existing[r][c];
		}
	}

	if (existing_norm_sq < 1e-12f) {
		// Existing calibration is near-zero (identity): treat as low similarity
		return 0.0f;
	}

	float similarity = 1.0f - sqrtf(diff_norm_sq / existing_norm_sq);
	if (similarity < 0.0f) { similarity = 0.0f; }
	if (similarity > 1.0f) { similarity = 1.0f; }
	return similarity;
}

/**
 * Blend two BAinv calibrations using exponential moving average (EMA).
 * blended = (1 - alpha) * existing + alpha * candidate
 * Alpha is computed from similarity: more similar → lower alpha (conservative),
 * more different → higher alpha (adaptive to environmental change).
 *
 * The blended result is validated with the same structural checks as
 * magneto_quality_check. If blending produces an invalid result, the
 * candidate is used directly (fallback to full replacement).
 *
 * Returns true if the blend (or fallback) is valid, false if both are invalid.
 */
static bool magneto_blend_BAinv(float out[4][3], float existing[4][3],
                                float candidate[4][3])
{
	float similarity = magneto_BAinv_similarity(existing, candidate);

	// Compute adaptive blending weight
	float alpha;
	if (similarity >= ONLINE_BLEND_SIMILARITY_HIGH) {
		// Very similar: candidate is just a minor refinement → low alpha
		alpha = ONLINE_BLEND_MIN_ALPHA;
	} else if (similarity <= ONLINE_BLEND_SIMILARITY_LOW) {
		// Significant divergence (possible environment change) → high alpha
		alpha = ONLINE_BLEND_MAX_ALPHA;
	} else {
		// Linear interpolation between low and high thresholds
		float t = (similarity - ONLINE_BLEND_SIMILARITY_LOW)
		        / (ONLINE_BLEND_SIMILARITY_HIGH - ONLINE_BLEND_SIMILARITY_LOW);
		alpha = ONLINE_BLEND_MAX_ALPHA
		      + t * (ONLINE_BLEND_MIN_ALPHA - ONLINE_BLEND_MAX_ALPHA);
	}

	// Blend
	float blended[4][3];
	float one_minus_alpha = 1.0f - alpha;
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 3; c++) {
			blended[r][c] = one_minus_alpha * existing[r][c]
			              + alpha * candidate[r][c];
		}
	}

	if (mag_bainv_structurally_ok(blended, 0.0f)) {
		memcpy(out, blended, sizeof(blended));
		return true;
	}

	if (mag_bainv_structurally_ok(candidate, 0.0f)) {
		memcpy(out, candidate, sizeof(float) * 4 * 3);
		return true;
	}

	return false;
}


// Phase 2: Background online magnetometer calibration
// Called from sensor loop for each new raw mag sample during normal operation.
// Gated by: VQF disturbance detection, accel magnitude, time interval, and direction change.
void sensor_calibration_online_mag_sample(const float m[3])
{
	if (!sensor_calibration_get_online_mag_enabled()) {
		return;
	}

	/* This thread owns quad_buf, so it performs any clear requested elsewhere. */
	magneto_online_service_clear();

	// Don't accumulate during manual calibration
	if (magneto_progress & 0x80) {
		return;
	}

	uint32_t now = k_uptime_get_32();

	// Track fusion mag-disturbance duration (before any sample gates) so the
	// background check function knows how long disturbance has persisted.
	if (sensor_fusion_get_mag_dist_detected()) {
		if ((uint32_t)atomic_get(&online_mag_dist_start_time) == 0) {
			atomic_set(&online_mag_dist_start_time, (atomic_val_t)now);
		}
	} else {
		atomic_set(&online_mag_dist_start_time, 0);
	}

	// Suppress collection after buffer resets (wake-up, reboot, environment
	// change, calibration update) to let sensor data stabilise.
	uint32_t suppress_until = (uint32_t)atomic_get(&online_collection_suppress_until);
	if (suppress_until != 0 && !ONLINE_TIME_GE(now, suppress_until)) {
		return;
	}

	// Reject if fusion detects magnetic disturbance (only when we have an existing
	// calibration — fusion only receives mag data when calibrated, so mag_dist_detected
	// is meaningless without calibration).
	// Exception 1: if current calibration quality is bad (norm CV > 6%), the disturbance
	// detection itself may be unreliable due to the bad calibration, so skip the gate.
	// Exception 2: if fusion has been reporting disturbance continuously for a long time,
	// the "disturbance" is likely a calibration drift or environment change rather than
	// transient interference.  Allow samples through to enable recalibration.
	// Without this, a deadlock occurs: disturbance → gate blocks samples →
	// cal_norm_count stops updating (guarded by !magDistDetected in sensor.c) → CV
	// stays frozen at a low value → gate never opens → no recalibration possible.
	{
		float zero[3] = {0};
		float current_cal[4][3];
		magneto_online_snapshot_BAinv(current_cal);
		bool has_cal = (v_diff_mag(current_cal[0], zero) != 0);
		float current_cv = sensor_calibration_get_mag_quality();
		if (has_cal && current_cv < 0.06f && sensor_fusion_get_mag_dist_detected()) {
			// Sustained disturbance override: if disturbance has persisted for
			// more than 5 seconds, allow samples through.
			uint32_t dist_start = (uint32_t)atomic_get(&online_mag_dist_start_time);
			bool sustained = (dist_start != 0 && ONLINE_ELAPSED(now, dist_start) > 5000U);
			if (!sustained) {
				return;
			}
		}
	}

	// Rate limit: minimum interval between samples
	uint32_t last_sample = (uint32_t)atomic_get(&online_last_sample_time);
	if (last_sample != 0 && ONLINE_ELAPSED(now, last_sample) < ONLINE_MIN_INTERVAL_MS) {
		return;
	}

	// Gate by accel magnitude: reject samples under strong linear acceleration
	float accel_mag_sq = aBuf[0] * aBuf[0] + aBuf[1] * aBuf[1] + aBuf[2] * aBuf[2];
	if (accel_mag_sq < MAG_CAL_ACCEL_MAG_MIN_SQ || accel_mag_sq > MAG_CAL_ACCEL_MAG_MAX_SQ) {
		return;
	}

	// Direction diversity gate: accept sample if either mag direction OR
	// accelerometer (gravity) direction has changed since last accepted sample.
	// Pure magnetometer-based direction check can suffer from "direction lock-in"
	// during strong magnetic interference: the mag reading points to a distorted
	// but stable direction while the tracker physically rotates.  The accelerometer
	// cross-check breaks this deadlock — if the device has physically moved
	// (accel direction changed), accept the sample regardless of mag direction.
	float raw_mag[3] = {m[0], m[1], m[2]};
	float cur_dir[3];
	if (magneto_norm_sq(raw_mag) < 1e-8f) {
		return;
	}
	magneto_center_update(&online_center_estimator, raw_mag);
	if (!magneto_centered_direction(&online_center_estimator, raw_mag, cur_dir)) {
		return;
	}

	// Normalize accelerometer to get gravity direction
	// aBuf magnitude already validated (~1g) by the accel gate above
	float accel_norm = sqrtf(accel_mag_sq);
	float accel_inv = 1.0f / accel_norm;
	float cur_accel_dir[3] = {aBuf[0] * accel_inv, aBuf[1] * accel_inv, aBuf[2] * accel_inv};

	if ((uint32_t)atomic_get(&online_total_sample_count) > 0) {
		float mag_dot = cur_dir[0] * online_last_dir[0]
		              + cur_dir[1] * online_last_dir[1]
		              + cur_dir[2] * online_last_dir[2];
		float accel_dot = cur_accel_dir[0] * online_last_accel_dir[0]
		                + cur_accel_dir[1] * online_last_accel_dir[1]
		                + cur_accel_dir[2] * online_last_accel_dir[2];

		float min_change = magneto_online_min_dir_change_threshold();
		bool mag_changed = (1.0f - mag_dot >= min_change);
		bool accel_changed = (1.0f - accel_dot >= min_change);

		if (!mag_changed && !accel_changed) {
			return; // neither mag nor accel direction changed enough
		}
	}

	atomic_set(&online_last_sample_time, (atomic_val_t)now);
	online_last_dir[0] = cur_dir[0];
	online_last_dir[1] = cur_dir[1];
	online_last_dir[2] = cur_dir[2];
	online_last_accel_dir[0] = cur_accel_dir[0];
	online_last_accel_dir[1] = cur_accel_dir[1];
	online_last_accel_dir[2] = cur_accel_dir[2];

	float route_mag[3];
	magneto_coverage_sample(&online_center_estimator, raw_mag, route_mag);
	if (magneto_norm_sq(route_mag) < 1e-8f) {
		memcpy(route_mag, raw_mag, sizeof(route_mag));
	}

	// Route sample to its octant based on sign relative to the min/max center.
	// This guarantees each octant independently rolls its ring buffer,
	// preventing a single orientation from evicting diverse data in other octants.
	int octant = 0;
	if (route_mag[0] < 0) octant |= 1;
	if (route_mag[1] < 0) octant |= 2;
	if (route_mag[2] < 0) octant |= 4;

	k_mutex_lock(&quad_buf_lock, K_FOREVER);
	if ((unsigned)atomic_get(&quad_buf_gen)
	    != (unsigned)atomic_get(&quad_buf_gen_served)) {
		k_mutex_unlock(&quad_buf_lock);
		return;
	}
	quadrant_buf_t *qbuf = &quad_buf[octant];
	uint32_t total = (uint32_t)atomic_get(&online_total_sample_count) + 1U;
	atomic_set(&online_total_sample_count, (atomic_val_t)total);
	qbuf->last_seq = total;
	qbuf->samples[qbuf->head].x = m[0];
	qbuf->samples[qbuf->head].y = m[1];
	qbuf->samples[qbuf->head].z = m[2];
	qbuf->head = (qbuf->head + 1) % QUADRANT_BUF_SIZE;
	if (qbuf->count < QUADRANT_BUF_SIZE) {
		qbuf->count++;
	}
	k_mutex_unlock(&quad_buf_lock);
}

/* The storage mutex makes retained publication and dirty registration atomic to flush. */
static bool magneto_online_commit_BAinv(const float m_inv[4][3], unsigned snap_gen,
                                        uint32_t now_ms, int update_count,
						float buf_avg_norm)
{
	sys_warm_transaction_begin();
	k_spinlock_key_t key = k_spin_lock(&online_publish_lock);
	if (atomic_get(&online_commits_suspended) != 0
	    || atomic_get(&online_enabled) == 0
	    || (unsigned)atomic_get(&quad_buf_gen) != snap_gen) {
		k_spin_unlock(&online_publish_lock, key);
		sys_warm_transaction_end(false);
		return false;
	}
	memcpy(magBAinv, m_inv, sizeof(magBAinv));
	memcpy(&retained->magBAinv, m_inv, sizeof(magBAinv));
	magneto_online_clear_history_at_locked(now_ms);
	atomic_set(&online_last_update_time, (atomic_val_t)now_ms);
	k_spinlock_key_t state_key = k_spin_lock(&online_runtime_state_lock);
	online_runtime_state.update_count = update_count;
	online_runtime_state.last_buf_avg_norm = buf_avg_norm;
	k_spin_unlock(&online_runtime_state_lock, state_key);
	k_spin_unlock(&online_publish_lock, key);
	sys_warm_transaction_mark(MAIN_MAG_BIAS_ID, &retained->magBAinv, sizeof(magBAinv));
	sys_warm_transaction_end(true);
	return true;
}

bool sensor_calibration_online_mag_check(void)
{
	if (!sensor_calibration_get_online_mag_enabled()
	    || atomic_get(&online_commits_suspended) != 0) {
		return false;
	}
	float existing_cal[4][3];
	online_runtime_state_t runtime_state;
	unsigned state_generation;
	magneto_online_calibration_state_snapshot(existing_cal, &runtime_state, &state_generation);
	int online_update_count = runtime_state.update_count;
	float online_last_buf_avg_norm = runtime_state.last_buf_avg_norm;

	int recent_sample_count_now = magneto_online_recent_sample_count();
	uint32_t now = k_uptime_get_32();

	if (recent_sample_count_now < MAG_CAL_MIN_SAMPLES) {
		return false;
	}
	/*
	 * Change detector only: the counter is a single atomic word, so this read is
	 * well-defined even though the sensor thread advances it concurrently. It is
	 * never used to index the snapshot — that value comes from the snapshot
	 * descriptor, taken atomically with the buffer copy.
	 */
	uint32_t total_now = (uint32_t)atomic_get(&online_total_sample_count);
	if (total_now == (uint32_t)atomic_get(&online_last_checked_sample_count)) {
		return false;
	}
	uint32_t last_check = (uint32_t)atomic_get(&online_last_check_time);
	if (last_check != 0 && ONLINE_ELAPSED(now, last_check) < ONLINE_MIN_CHECK_INTERVAL_MS) {
		return false;
	}

	atomic_set(&online_last_checked_sample_count, (atomic_val_t)total_now);
	atomic_set(&online_last_check_time, (atomic_val_t)now);

	float zero[3] = {0};
	bool has_existing = (v_diff_mag(existing_cal[0], zero) != 0);
	float current_cv = has_existing ? magneto_online_mag_quality_from_state(&runtime_state) : 1.0f;

	// When fusion has a reliable calibration (CV < 4%) and is NOT experiencing
	// sustained magnetic disturbance, skip the calibration check entirely.
	// Updating calibration resets the fusion mag reference, causing ~6s of
	// heading instability.  Only attempt a recalibration when fusion has been
	// detecting disturbance for >3 seconds, indicating the current calibration
	// is genuinely insufficient.
	if (has_existing && current_cv < 0.04f) {
		uint32_t dist_start = (uint32_t)atomic_get(&online_mag_dist_start_time);
		if (dist_start == 0) {
			return false; // fusion not disturbed — current cal is fine
		}
		if (ONLINE_ELAPSED(now, dist_start) < ONLINE_VQF_DIST_MIN_DURATION_MS) {
			return false; // transient disturbance — wait
		}
	}

	// If the current calibration is already good enough AND we've had enough
	// updates to trust that assessment, skip the heavy Magneto fit.
	//
	// Two-tier convergence:
	//   Tier 1 (strict): CV is good AND directional coverage is adequate
	//     → require low dir_bias (sphere sampled evenly)
	//   Tier 2 (relaxed): CV is excellent AND we've done many updates
	//     → trust the fit regardless of dir_bias (directional fluctuations
	//       during normal rotation are just sampling noise, not real problems)
	//
	// Exception: skip convergence checks when fusion is experiencing sustained
	// magnetic disturbance.  The CV value is frozen during disturbance (norm
	// tracking gated by !magDistDetected in sensor.c), so a low frozen CV
	// does NOT mean the calibration is still good — the environment may have
	// changed.  If the code reached here past the CV < 4% disturbance gate
	// above, the disturbance is sustained and recalibration should proceed.
	uint32_t dist_start_now = (uint32_t)atomic_get(&online_mag_dist_start_time);
	bool mag_sustained_dist = (dist_start_now != 0 &&
	                           ONLINE_ELAPSED(now, dist_start_now) > ONLINE_VQF_DIST_MIN_DURATION_MS);
	if (has_existing && current_cv < CAL_NORM_GOOD_CV && online_update_count >= ONLINE_MIN_UPDATES
	    && !mag_sustained_dist) {
		// Tier 2: Excellent fit + sufficient history — lock it in.
		// CV < 0.035 and 3+ updates mean the calibration has reliably
		// converged.  Further updates would only add noise.
		if (current_cv < 0.035f && online_update_count >= 3) {
			LOG_INF("Online mag cal: converged (cv=%.3f, %d updates)",
			        (double)current_cv, online_update_count);
			return false;
		}

		float dir_bias_check = magneto_online_recent_dir_bias();

		// Tier 1: Good fit with adequate directional coverage
		if (dir_bias_check < 0.10f) {
			LOG_INF("Online mag cal: skipping (cv=%.3f < %.3f, dir_bias=%.3f, %d updates)",
			        (double)current_cv, (double)CAL_NORM_GOOD_CV,
			        (double)dir_bias_check, online_update_count);
			return false;
		}
		// Directional bias still too high: buffer samples are clustered.
		// Fall through to run calibration even though CV looks good.
		LOG_INF("Online mag cal: CV ok but dir_bias=%.3f >= 0.10, continuing",
		        (double)dir_bias_check);
	}

	double ata_recent[100];
	double recent_norm_sum;
	float recent_dir_sum[3];
	float recent_raw_range;
	quad_buf_snapshot_t snap;
	double recent_sample_count = magneto_online_collect_recent(ata_recent, &recent_norm_sum,
	                                                           recent_dir_sum, &recent_raw_range,
	                                                           &snap);
	if (!snap.valid || snap.gen != state_generation
	    || recent_sample_count < MAG_CAL_MIN_SAMPLES) {
		return false;
	}
	if (recent_raw_range < MAG_CAL_MIN_RAW_AXIS_RANGE) {
		LOG_INF("Online mag cal: need more rotation (raw_range=%.3f < %.3f, %d recent samples)",
		        (double)recent_raw_range, (double)MAG_CAL_MIN_RAW_AXIS_RANGE,
		        (int)recent_sample_count);
		return false;
	}

	// Detect magnetic environment changes by comparing the buffer's
	// average raw field strength against the last update's reference.
	// When the norm changes by >25% (e.g., moving between a desk and
	// a high-interference area), clear buffers to prevent mixed-data
	// fits.  Direction is preserved — only scale changes.
	float buf_avg_norm = (float)(recent_norm_sum / recent_sample_count);

	if (has_existing && online_update_count > 0 && online_last_buf_avg_norm > 0.0f) {
		float norm_ratio = buf_avg_norm / online_last_buf_avg_norm;
		if (norm_ratio > 1.25f || norm_ratio < 0.80f) {
			LOG_WRN("Online mag cal: env change detected (buf norm %.3f -> %.3f, ratio %.2f), "
			        "resetting buffers",
			        (double)online_last_buf_avg_norm, (double)buf_avg_norm, (double)norm_ratio);
			magneto_online_clear_progress(now);
			return false;
		}
	}

	float dbias = magneto_directional_bias(recent_dir_sum, recent_sample_count);
	LOG_INF("Online mag cal: coverage raw_range=%.3f, dir_bias=%.3f, n=%d",
	        (double)recent_raw_range, (double)dbias, (int)recent_sample_count);

	// Quality check: directional diversity + validation + compute calibration
	float m_inv[4][3];
	if (!magneto_quality_check(ata_recent, recent_norm_sum, recent_sample_count, m_inv)) {
		LOG_INF("Online mag cal: check failed (%d recent samples, dir_bias=%.3f)",
		        (int)recent_sample_count, (double)dbias);
		return false;
	}
	if (!sensor_calibration_get_online_mag_enabled()) {
		LOG_INF("Online mag cal: disabled before apply, skipping update");
		return false;
	}

	float committed[4][3];
	int next_update_count;

	if (has_existing) {
		// Enforce minimum cooldown between updates to avoid frequent VQF mag ref resets.
		// Each update resets VQF's heading reference, causing ~6s of re-establishment.
		uint32_t last_update = (uint32_t)atomic_get(&online_last_update_time);
		if (last_update != 0 &&
		    ONLINE_ELAPSED(now, last_update) < (ONLINE_MIN_UPDATE_INTERVAL_S * 1000U)) {
			return false;
		}

		// Blend trial calibration with existing using EMA.
		// Blending weight is similarity-adaptive: more similar → conservative,
		// more divergent → faster adaptation (possible environment change).
		float blended[4][3];
		if (!magneto_blend_BAinv(blended, existing_cal, m_inv)) {
			LOG_WRN("Online mag cal: blend validation failed, skipping update");
			return false;
		}

		float similarity = magneto_BAinv_similarity(existing_cal, m_inv);

		// Reject candidate if similarity is below threshold — the data
		// is too inconsistent for a meaningful fit.  This guards against
		// mixed-data fits (e.g. when the tracker moves between magnetic
		// environments and the quadrant buffer holds samples from both old
		// and new locations).
		// Rather than blindly trusting a poor fit, let the buffer age out
		// stale samples; the next cycle will fit a consistent dataset with
		// much higher similarity.
		//
		// Exception: during the first ONLINE_MIN_UPDATES cycles we accept
		// even low-sim fits to establish an initial baseline (especially
		// important when booting with a stale NVS calibration).
		if (similarity < 0.85f) {
			if (online_update_count < ONLINE_MIN_UPDATES) {
				LOG_INF("Online mag cal: low sim=%.3f accepted (early bootstrap #%d)",
				        (double)similarity, online_update_count + 1);
			} else if (current_cv < CAL_NORM_GOOD_CV) {
				LOG_WRN("Online mag cal: rejecting candidate (sim=%.3f < 0.85, "
				        "current cv=%.3f is good — possible mixed data)",
				        (double)similarity, (double)current_cv);
				return false;
			} else {
				LOG_WRN("Online mag cal: rejecting candidate (sim=%.3f < 0.85, "
				        "current cv=%.3f — incomplete/dirty buffer?)",
				        (double)similarity, (double)current_cv);
				return false;
			}
		}

		LOG_INF("Online mag cal: blended (#%d, %d samples, dir_bias=%.3f, cur_cv=%.3f, sim=%.3f)",
		        online_update_count + 1,
		        (int)recent_sample_count, (double)dbias, (double)current_cv,
		        (double)similarity);
		memcpy(committed, blended, sizeof(committed));
		next_update_count = online_update_count + 1;
	} else {
		LOG_INF("Online mag cal: first calibration (%d recent samples, dir_bias=%.3f)",
		        (int)recent_sample_count, (double)dbias);

		// First calibration: use candidate directly
		memcpy(committed, m_inv, sizeof(committed));
		next_update_count = 1;
	}

	/*
	 * Publish only if the samples this fit came from are still the current
	 * generation. Anything that cleared or replaced the calibration while the fit
	 * was running wins, and this fit is discarded instead of undoing it.
	 */
	if (!magneto_online_commit_BAinv(committed, snap.gen, now, next_update_count,
	                                 buf_avg_norm)) {
		LOG_INF("Online mag cal: discarded, calibration was cleared during the fit");
		return false;
	}

	memcpy(m_inv, committed, sizeof(m_inv)); // persisted and logged below

	// Reset fusion mag reference so it re-establishes with the new calibration
	sensor_fusion_reset_mag_ref();
	sensor_mag_ref_reset();

	// Reset norm tracking after calibration change
	magneto_online_norm_state_reset();
	sensor_refresh_sensor_ids();

	LOG_INF("Online mag cal applied:");
	for (int i = 0; i < 3; i++) {
		LOG_INF("%.5f %.5f %.5f %.5f",
			(double)m_inv[0][i], (double)m_inv[1][i],
			(double)m_inv[2][i], (double)m_inv[3][i]);
	}

	return true;
}

int sensor_calibration_online_mag_status(float *dir_bias)
{
	if (!sensor_calibration_get_online_mag_enabled()) {
		if (dir_bias) {
			*dir_bias = 1.0f;
		}
		return 0;
	}
	if (dir_bias) {
		*dir_bias = magneto_online_recent_dir_bias();
	}
	return magneto_online_recent_sample_count();
}

// Feed calibrated mag norm for runtime quality tracking.
// Called from sensor.c after applying BAinv calibration.
void sensor_calibration_track_mag_norm(float cal_norm)
{
	if (!sensor_calibration_get_online_mag_enabled()) {
		return;
	}
	if (cal_norm < 1e-6f) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&online_runtime_state_lock);
	if (online_runtime_state.cal_norm_count == 0) {
		online_runtime_state.cal_norm_ema = cal_norm;
		online_runtime_state.cal_norm_var_ema = 0;
	} else {
		float diff = cal_norm - online_runtime_state.cal_norm_ema;
		online_runtime_state.cal_norm_ema += CAL_NORM_EMA_ALPHA * diff;
		online_runtime_state.cal_norm_var_ema += CAL_NORM_EMA_ALPHA
			* (diff * diff - online_runtime_state.cal_norm_var_ema);
	}
	online_runtime_state.cal_norm_count++;
	k_spin_unlock(&online_runtime_state_lock, key);
}

// Get current calibration quality: returns norm CV (std/mean).
// Lower is better. Returns 1.0 if insufficient data.
float sensor_calibration_get_mag_quality(void)
{
	online_runtime_state_t state = magneto_online_runtime_state_snapshot();
	return magneto_online_mag_quality_from_state(&state);
}
