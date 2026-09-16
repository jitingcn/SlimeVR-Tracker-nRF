#include "globals.h"
#include "sensor/sensor.h"
#include "system/system.h"
#include "system/watchdog.h"
#include "util.h"
#include "calibration.h"
#include "mag_common.h"
#include "mag_fit.h"
#include "online_mag.h"

#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(cal_online_mag, LOG_LEVEL_INF);

#define ONLINE_SLOTS 256
#define ONLINE_OCTANTS 8
#define ONLINE_PER_OCTANT 32
#define ONLINE_CELLS 24
#define ONLINE_TTL_MS 60000U
#define ONLINE_INTERVAL_MS 30U
#define ONLINE_SUPPRESS_MS 1500U
#define ONLINE_CHECK_MS 8000U
#define ONLINE_FREEZE_TIMEOUT_MS 250U
#define ONLINE_FIT_TIMEOUT_MS 5000U
#define ONLINE_VALIDATE_MS 3000U
#define ONLINE_PROBATION_MS 10000U
#define ONLINE_EPISODE_TIMEOUT_MS 45000U
#define ELAPSED(now, then) ((uint32_t)((now) - (then)))

struct online_sample {
	float raw[3];
	uint32_t time;
};
/* Exactly one raw pool. Only sensor writes it; fitter borrows it after ACK.
 * count, not timestamp zero, indicates an occupied slot (uptime wraps). */
static struct online_sample pool[ONLINE_SLOTS];
static uint8_t heads[ONLINE_OCTANTS], counts[ONLINE_OCTANTS];
struct direction_metrics {
	float old_sq, new_sq, old_max, new_max;
	float old_dip, new_dip, old_dip_sq, new_dip_sq;
	float norm_sum, old_min;
	uint16_t count, dip_count;
};
static struct direction_metrics directions[ONLINE_CELLS];
static float candidate[4][3], previous[4][3], replacement[4][3];
/* This lock never covers fitting, sleeping, storage, or a pool snapshot. */
static struct k_spinlock online_lock;
/* Runtime-only logging preference; calibration resets must not change it. */
static bool online_debug;
static struct {
	uint32_t generation, served, episode, last_sample, last_check, suppress;
	uint32_t summary_time, norm_count;
	float field, dip, candidate_field, reference_norm, reference_dip;
	float norm_mean, norm_var, dir_bias, center[3], last_dir[3];
	uint16_t recent_count, admitted_since_fit;
	uint8_t updates, phase;
	bool enabled, suspended, fitter, started, replace_pending, ref_pending;
	bool trusted, dip_known, trial, unchanged, candidate_dip_known;
	float validation_norm, validation_dip;
	bool validation_dip_known;
	struct online_mag_diagnostics diagnostics;
} online;

_Static_assert(
	sizeof(pool) + sizeof(heads) + sizeof(counts) + sizeof(directions) + sizeof(candidate) + sizeof(previous)
			+ sizeof(replacement) + sizeof(online) + sizeof(online_lock) + sizeof(online_debug)
		<= 6272,
	"online magnetic state must fit the former two-pool budget"
);

static float dot3(const float a[3], const float b[3])
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static bool transform(const float matrix[4][3], const float raw[3], float unit[3], float *norm)
{
	float v[3] = {raw[0] - matrix[0][0], raw[1] - matrix[0][1], raw[2] - matrix[0][2]};
	for (unsigned i = 0; i < 3; ++i) {
		unit[i] = matrix[1][i] * v[0] + matrix[2][i] * v[1] + matrix[3][i] * v[2];
	}
	*norm = sqrtf(dot3(unit, unit));
	if (!isfinite(*norm) || *norm < 1e-6f) {
		return false;
	}
	for (unsigned i = 0; i < 3; ++i) {
		unit[i] /= *norm;
	}
	return true;
}

static bool has_model(const float matrix[4][3], bool confirmed)
{
	/* Identity is the erased/default calibration, not evidence of a fit.
	 * A zero-bias nonidentity SPD model is nevertheless a valid calibration. */
	for (unsigned row = 0; row < 4; ++row) {
		for (unsigned col = 0; col < 3; ++col) {
			if (matrix[row][col] != ((row == col + 1) ? 1.0f : 0.0f)) {
				return mag_bainv_structurally_ok(matrix, 0.0f);
			}
		}
	}
	return confirmed;
}

static void norm_reset(void)
{
	online.norm_count = 0;
	online.norm_mean = online.norm_var = 0;
}

static void clear_scores_locked(void)
{
	online.diagnostics.old_rms = online.diagnostics.new_rms = 0;
	online.diagnostics.worst_cell_rms = online.diagnostics.max_radial_error = 0;
	online.diagnostics.radial_count = online.diagnostics.dip_count = 0;
	online.diagnostics.radial_cells = online.diagnostics.dip_cells = 0;
	online.diagnostics.radial_poles = online.diagnostics.dip_poles = 0;
	online.diagnostics.score_phase = online.phase;
	online.diagnostics.score_valid = false;
	online.diagnostics.old_dip_sd = online.diagnostics.new_dip_sd = online.diagnostics.dip_delta = 0;
	online.diagnostics.last_gate = ONLINE_MAG_REJECT_NONE;
}

static void cancel_locked(uint32_t now)
{
	if (online.diagnostics.outcome == ONLINE_MAG_NONE) {
		online.diagnostics.outcome = ONLINE_MAG_REJECTED;
		online.diagnostics.rejection = ONLINE_MAG_REJECT_CANCELLED;
	}
	++online.generation;
	online.recent_count = 0;
	online.dir_bias = 1;
	online.suppress = now;
	online.last_check = now;
}

static void reference_locked(float norm, float dip)
{
	online.reference_norm = norm;
	online.reference_dip = dip;
	online.ref_pending = true;
	norm_reset();
}

static void reset_episode_locked(uint32_t now)
{
	memset(counts, 0, sizeof(counts));
	memset(heads, 0, sizeof(heads));
	memset(directions, 0, sizeof(directions));
	memset(online.center, 0, sizeof(online.center));
	memset(online.last_dir, 0, sizeof(online.last_dir));
	online.recent_count = 0;
	online.admitted_since_fit = 0;
	online.dir_bias = 1;
	online.summary_time = online.last_sample = now;
	online.phase = TRAINING;
	online.served = online.generation;
}

/* Internal outcomes retain the moving raw history. Explicit cancellation alone
 * invalidates the generation/discards it, after the fitter releases ownership. */
static void restart_locked(uint32_t now)
{
	if (online.trial) {
		memcpy(magBAinv, previous, sizeof(previous));
		online.trial = false;
		reference_locked(online.dip_known ? online.field : 0, online.dip_known ? online.dip : 0);
	}
	online.phase = TRAINING;
	online.episode = now;
	online.unchanged = false;
}

static const char *gate_name(unsigned gate)
{
	switch (gate) {
	case ONLINE_MAG_REJECT_NONE:
		return "none";
	case ONLINE_MAG_REJECT_FIT:
		return "fit";
	case ONLINE_MAG_REJECT_RADIAL:
		return "radial";
	case ONLINE_MAG_REJECT_DIP:
		return "dip";
	case ONLINE_MAG_REJECT_COVERAGE:
		return "coverage";
	case ONLINE_MAG_REJECT_TIMEOUT:
		return "timeout";
	case ONLINE_MAG_REJECT_CANCELLED:
		return "cancelled";
	case ONLINE_MAG_REJECT_MATRIX:
		return "matrix";
	case ONLINE_MAG_REJECT_SAMPLE:
		return "sample";
	case ONLINE_MAG_REJECT_OVERFLOW:
		return "overflow";
	case ONLINE_MAG_REJECT_NO_BENEFIT:
		return "no-benefit";
	default:
		return "unknown";
	}
}
static void log_snapshot(const char *event, const struct online_mag_diagnostics *snapshot)
{
	if (!sensor_calibration_get_online_mag_debug()) {
		return;
	}
	const struct online_mag_diagnostics d = *snapshot;
	if (d.outcome == ONLINE_MAG_REJECTED) {
		LOG_WRN(
			"Online mag %s reason=%s gate=%s age=%u ms rms=%f/%f cells=%u poles=0x%02x dip_sd=%f/%f delta=%f",
			event,
			gate_name(d.rejection),
			gate_name(d.last_gate),
			d.phase_age_ms,
			(double)d.old_rms,
			(double)d.new_rms,
			d.radial_cells,
			d.radial_poles,
			(double)d.old_dip_sd,
			(double)d.new_dip_sd,
			(double)d.dip_delta
		);
	} else {
		LOG_INF(
			"Online mag %s gate=%s age=%u ms rms=%f/%f cells=%u poles=0x%02x dip_sd=%f/%f delta=%f",
			event,
			gate_name(d.last_gate),
			d.phase_age_ms,
			(double)d.old_rms,
			(double)d.new_rms,
			d.radial_cells,
			d.radial_poles,
			(double)d.old_dip_sd,
			(double)d.new_dip_sd,
			(double)d.dip_delta
		);
	}
}

/* All live matrix publications after startup happen here/on this sensor call. */
static void service_locked(uint32_t now)
{
	if (online.replace_pending) {
		memcpy(magBAinv, replacement, sizeof(replacement));
		online.replace_pending = false;
		online.trial = false;
		reference_locked(0, 0);
	}
	if (online.served != online.generation) {
		if (online.trial) {
			memcpy(magBAinv, previous, sizeof(previous));
			online.trial = false;
			reference_locked(online.dip_known ? online.field : 0, online.dip_known ? online.dip : 0);
		}
		/* Cancellation invalidates the fit immediately, but its borrowed
		 * memory is not reusable until the fitter explicitly releases it. */
		if (!online.fitter) {
			reset_episode_locked(now);
		}
		return;
	}
	if (online.phase == FREEZE_REQUESTED) {
		memcpy(previous, magBAinv, sizeof(previous));
		online.phase = FROZEN;
	} else if (online.phase == VALIDATION_READY) {
		memset(directions, 0, sizeof(directions));
		online.episode = now;
		online.phase = VALIDATING;
		clear_scores_locked();
		online.last_sample = now;
	}
}

void magneto_online_snapshot_BAinv(float out[4][3])
{
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	memcpy(out, magBAinv, sizeof(magBAinv));
	k_spin_unlock(&online_lock, key);
}

void magneto_online_replace_BAinv_and_reset(const float value[4][3])
{
	bool calibrated = has_model(value, false);
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	cancel_locked(k_uptime_get_32());
	memset(&online.diagnostics, 0, sizeof(online.diagnostics));
	online.updates = 0;
	online.trusted = calibrated;
	online.field = 0;
	online.dip_known = false;
	memcpy(replacement, value, sizeof(replacement));
	online.replace_pending = true;
	/* Boot calibration precedes the sensor/fusion consumer. Install now so
	 * startup validation/console sees it, but still enqueue a domain reset:
	 * warm fusion may contain an unconfirmed pre-sleep trial reference. */
	if (!online.started) {
		memcpy(magBAinv, value, sizeof(magBAinv));
		online.replace_pending = false;
		reference_locked(0, 0);
	}
	k_spin_unlock(&online_lock, key);
}

bool magneto_online_take_mag_ref(float *norm, float *dip)
{
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	bool pending = online.ref_pending;
	if (pending) {
		*norm = online.reference_norm;
		*dip = online.reference_dip;
		online.ref_pending = false;
	}
	k_spin_unlock(&online_lock, key);
	return pending;
}

void magneto_online_reset(void)
{
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	cancel_locked(k_uptime_get_32());
	k_spin_unlock(&online_lock, key);
}

void magneto_online_runtime_reset(void)
{
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	cancel_locked(k_uptime_get_32());
	online.updates = 0;
	norm_reset();
	k_spin_unlock(&online_lock, key);
}

void magneto_online_runtime_configure(bool enabled)
{
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	online.enabled = enabled;
	cancel_locked(k_uptime_get_32());
	norm_reset();
	k_spin_unlock(&online_lock, key);
}

void magneto_online_runtime_load_retained(void)
{
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	online.updates = retained->onlineMagState.update_count;
	online.trusted = online.updates > 0 || online.trusted;
	/* Older firmware stored RAW norm here. Do not reinterpret it as a
	 * calibrated field reference. Reacquire from sensor-owned fusion. */
	online.field = 0;
	online.dip_known = false;
	cancel_locked(k_uptime_get_32());
	k_spin_unlock(&online_lock, key);
}

void sensor_calibration_online_mag_prepare_power_down(void)
{
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	online.suspended = true;
	cancel_locked(k_uptime_get_32());
	k_spin_unlock(&online_lock, key);
}

bool sensor_calibration_get_online_mag_debug(void)
{
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	bool enabled = online_debug;
	k_spin_unlock(&online_lock, key);
	return enabled;
}

void sensor_calibration_set_online_mag_debug(bool enabled)
{
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	online_debug = enabled;
	k_spin_unlock(&online_lock, key);
}

bool sensor_calibration_get_online_mag_enabled(void)
{
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	bool enabled = online.enabled;
	k_spin_unlock(&online_lock, key);
	return enabled;
}

void sensor_calibration_set_online_mag_enabled(bool enabled)
{
	uint8_t mode = enabled ? MAG_ONLINE_CALIBRATION_ENABLED : MAG_ONLINE_CALIBRATION_DISABLED;
	/* Repeated configuration must not discard a candidate or roll back a trial. */
	if (sensor_calibration_get_online_mag_enabled() == enabled && retained->mag_online_calibration_mode == mode) {
		return;
	}
	magneto_online_runtime_configure(enabled);
	if (!enabled) {
		sensor_calibration_online_mag_retained_clear();
	}
	sys_write(MAG_ONLINE_CALIBRATION_ID, &retained->mag_online_calibration_mode, &mode, sizeof(mode));
	LOG_INF("Online mag calibration %s (persisted)", enabled ? "enabled" : "disabled");
}

void sensor_calibration_online_mag_retained_save(void)
{
	sys_warm_transaction_begin();
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	retained->onlineMagState.update_count = online.enabled ? online.updates : 0;
	retained->onlineMagState.last_buf_avg_norm = online.enabled ? online.field : 0;
	k_spin_unlock(&online_lock, key);
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

int cal_online_mag_update_count(void)
{
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	int count = online.updates;
	k_spin_unlock(&online_lock, key);
	return count;
}

uint32_t cal_online_mag_norm_count(void)
{
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	uint32_t count = online.norm_count;
	k_spin_unlock(&online_lock, key);
	return count;
}

float magneto_online_min_dir_change_threshold(void)
{
	return 0.015192247f;
}

static bool occupied(unsigned index)
{
	return index % ONLINE_PER_OCTANT < counts[index / ONLINE_PER_OCTANT];
}

/* Sensor-owned bounded summary; no status caller walks or copies raw samples.
 * Refresh min/max from ONLY TTL-valid samples, never lifetime extrema. */
static void summarize(uint32_t now, uint32_t generation, const float center[3])
{
	float lo[3] = {INFINITY, INFINITY, INFINITY};
	float hi[3] = {-INFINITY, -INFINITY, -INFINITY};
	float sum[3] = {0};
	unsigned count = 0;
	for (unsigned i = 0; i < ONLINE_SLOTS; ++i) {
		if (!occupied(i) || ELAPSED(now, pool[i].time) > ONLINE_TTL_MS) {
			continue;
		}
		float v[3], n;
		for (unsigned j = 0; j < 3; ++j) {
			lo[j] = fminf(lo[j], pool[i].raw[j]);
			hi[j] = fmaxf(hi[j], pool[i].raw[j]);
			v[j] = pool[i].raw[j] - center[j];
		}
		n = sqrtf(dot3(v, v));
		if (n > 1e-6f) {
			for (unsigned j = 0; j < 3; ++j) {
				sum[j] += v[j] / n;
			}
		}
		++count;
	}
	float bias = count ? sqrtf(dot3(sum, sum)) / count : 1;
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	if (online.generation == generation) {
		online.recent_count = count;
		online.dir_bias = bias;
		if (count) {
			for (unsigned j = 0; j < 3; ++j) {
				online.center[j] = (lo[j] + hi[j]) * 0.5f;
			}
		}
		online.summary_time = now;
	}
	k_spin_unlock(&online_lock, key);
}

static unsigned cell_of(const float unit[3])
{
	unsigned axis = fabsf(unit[1]) > fabsf(unit[0]) ? 1 : 0;
	if (fabsf(unit[2]) > fabsf(unit[axis])) {
		axis = 2;
	}
	return axis * 8 + (unit[0] < 0) + 2 * (unit[1] < 0) + 4 * (unit[2] < 0);
}

/* Equal directional weight prevents the easiest direction dominating scores.
 * No VQF-clean subset: every eligible future sample contributes radial error;
 * only independent, reliable q6 gravity contributes dip statistics. */
struct evidence {
	float norm, dip, old_dip;
	bool dip_known, old_healthy, old_dip_known, keep_previous;
	struct online_mag_diagnostics diagnostics;
};

static bool metrics_pass(bool trusted, bool allow_unchanged, struct evidence *e)
{
	unsigned poles = 0, dip_poles = 0, dip_bins = 0;
	float norm = 0, old_sq = 0, new_sq = 0, dip_sq = 0, old_dip_sq = 0;
	memset(e, 0, sizeof(*e));
	for (unsigned i = 0; i < ONLINE_CELLS; ++i) {
		const struct direction_metrics *d = &directions[i];
		unsigned pole = (i / 8) * 2 + ((i >> (i / 8)) & 1U);
		if (d->dip_count) {
			++dip_bins;
			e->diagnostics.dip_count += d->dip_count;
			e->dip += d->new_dip / d->dip_count;
			e->old_dip += d->old_dip / d->dip_count;
			dip_sq += d->new_dip_sq / d->dip_count;
			old_dip_sq += d->old_dip_sq / d->dip_count;
			if (d->dip_count >= 4) {
				++e->diagnostics.dip_cells;
				dip_poles |= 1U << pole;
			}
		}
		if (d->count < 4) {
			continue;
		}
		++e->diagnostics.radial_cells;
		e->diagnostics.radial_count += d->count;
		poles |= 1U << pole;
		norm += d->norm_sum / d->count;
		old_sq += d->old_sq / d->count;
		new_sq += d->new_sq / d->count;
	}
	unsigned cells = e->diagnostics.radial_cells, dc = e->diagnostics.dip_cells;
	e->diagnostics.radial_poles = poles;
	e->diagnostics.dip_poles = dip_poles;
	e->diagnostics.score_valid = cells != 0;
	if (cells) {
		e->norm = norm / cells;
		e->diagnostics.new_rms = sqrtf(new_sq / cells);
		e->diagnostics.old_rms
			= trusted && e->norm > 0 ? sqrtf(fmaxf(0, old_sq / cells - e->norm * e->norm)) / e->norm : 1;
	}
	e->old_healthy = trusted && e->diagnostics.old_rms <= 0.05f;
	bool radial_ok = e->diagnostics.new_rms <= 0.05f;
	for (unsigned i = 0; i < ONLINE_CELLS; ++i) {
		const struct direction_metrics *d = &directions[i];
		if (!d->count) {
			continue;
		}
		float cell_rms = sqrtf(d->new_sq / d->count);
		e->diagnostics.worst_cell_rms = fmaxf(e->diagnostics.worst_cell_rms, cell_rms);
		e->diagnostics.max_radial_error = fmaxf(e->diagnostics.max_radial_error, d->new_max);
		radial_ok &= cell_rms <= 0.06f && d->new_max <= 0.18f;
		float old_cell_sq = d->old_sq / d->count - 2 * e->norm * d->norm_sum / d->count + e->norm * e->norm;
		e->old_healthy &= e->norm > 0 && sqrtf(fmaxf(0, old_cell_sq)) / e->norm <= 0.06f
					   && d->old_max / e->norm <= 1.18f && d->old_min / e->norm >= 0.82f;
	}
	if (dip_bins) {
		e->dip /= dip_bins;
		e->old_dip /= dip_bins;
		dip_sq = fmaxf(0, dip_sq / dip_bins - e->dip * e->dip);
		old_dip_sq = fmaxf(0, old_dip_sq / dip_bins - e->old_dip * e->old_dip);
	}
	e->diagnostics.new_dip_sd = sqrtf(dip_sq);
	e->diagnostics.old_dip_sd = sqrtf(old_dip_sq);
	unsigned required_poles = online.phase == PROBATION ? 5 : 6;
	bool dip_coverage
		= dc >= 12 && (unsigned)__builtin_popcount(dip_poles) >= required_poles && e->diagnostics.dip_count >= 96;
	e->dip_known = dip_coverage && dip_sq <= 0.0064f;
	e->old_dip_known = dip_coverage && old_dip_sq <= 0.0064f;
	if (cells < 12 || (unsigned)__builtin_popcount(poles) < required_poles || e->diagnostics.radial_count < 96) {
		e->diagnostics.rejection = ONLINE_MAG_REJECT_COVERAGE;
		return false;
	}
	bool benefit = e->diagnostics.old_rms - e->diagnostics.new_rms >= 0.005f
				&& e->diagnostics.new_rms <= 0.8f * e->diagnostics.old_rms;
	bool unchanged = allow_unchanged && e->old_healthy && (!benefit || !radial_ok);
	/* A useful replacement still needs coherent gravity when available. */
	if (allow_unchanged && e->old_healthy && e->diagnostics.dip_count >= 24 && dip_sq > 0.0064f) {
		unchanged = true;
	}
	e->keep_previous = unchanged;
	if (!unchanged && !radial_ok) {
		e->diagnostics.rejection = ONLINE_MAG_REJECT_RADIAL;
		return false;
	}
	/* Gravity can veto incoherence; its absence cannot veto a sphere fit.
	 * A healthy old sphere can still finish unchanged in a nonuniform field,
	 * but cannot publish an environment dip without broad coherent evidence. */
	if (!unchanged && e->diagnostics.dip_count >= 24 && dip_sq > 0.0064f) {
		e->diagnostics.rejection = ONLINE_MAG_REJECT_DIP;
		return false;
	}
	if (!allow_unchanged && e->old_healthy && !benefit) {
		e->diagnostics.rejection = ONLINE_MAG_REJECT_NO_BENEFIT;
		return false;
	}
	return true;
}

/* Snapshot only scalar policy. Arrays remain sensor-owned/immutable throughout
 * evaluation; a concurrent reset can only invalidate generation, not reuse them. */
struct sample_policy {
	uint32_t generation, episode;
	float field, dip, candidate_field, center[3], last_dir[3];
	uint8_t phase;
	bool trusted, dip_known;
};

static void finish_sample(
	uint32_t now,
	const struct sample_policy *p,
	enum online_mag_rejection reject,
	bool passed,
	const struct evidence *e
)
{
	const char *event = NULL;
	struct online_mag_diagnostics logged;
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	if (online.generation == p->generation && online.phase == p->phase) {
		if (e) {
			int fit_errno = online.diagnostics.fit_errno;
			online.diagnostics = e->diagnostics;
			online.diagnostics.fit_errno = fit_errno;
			online.diagnostics.score_phase = p->phase;
			online.diagnostics.last_gate = e->diagnostics.rejection;
			online.diagnostics.phase_age_ms = ELAPSED(now, p->episode);
		}
		if (reject != ONLINE_MAG_REJECT_NONE) {
			if (!e) {
				clear_scores_locked();
				online.diagnostics.score_phase = p->phase;
			}
			online.diagnostics.outcome = ONLINE_MAG_REJECTED;
			online.diagnostics.rejection = reject;
			online.diagnostics.last_gate = reject;
			event = online.trial ? "rollback/rejected" : "rejected";
			restart_locked(now);
		} else if (passed) {
			if (online.phase == VALIDATING) {
				online.unchanged = e->keep_previous;
				online.validation_norm = e->norm;
				online.validation_dip = online.unchanged ? e->old_dip : e->dip;
				online.validation_dip_known = online.unchanged ? e->old_dip_known : e->dip_known;
				bool environment = online.unchanged && e->old_dip_known
								&& (!online.dip_known || online.field <= 0 || fabsf(e->norm / online.field - 1) > 0.02f
									|| fabsf(e->old_dip - online.dip) > 0.03f);
				if (online.unchanged && !environment) {
					online.diagnostics.outcome = ONLINE_MAG_UNCHANGED;
					event = "unchanged";
					restart_locked(now);
					goto done;
				}
				event = online.unchanged ? "reference confirmation start" : "trial start";
				if (!online.unchanged) {
					memcpy(magBAinv, candidate, sizeof(candidate));
					online.trial = true;
					reference_locked(e->dip_known ? online.candidate_field : 0, e->dip_known ? e->dip : 0);
				}
				logged = online.diagnostics;
				memset(directions, 0, sizeof(directions));
				online.phase = PROBATION;
				clear_scores_locked();
				online.episode = now;
				goto serviced;
			} else if (online.unchanged) {
				bool stable = e->old_healthy && e->old_dip_known && online.validation_dip_known
						   && fabsf(e->old_dip - online.validation_dip) <= 0.04f
						   && fabsf(e->norm / online.validation_norm - 1) <= 0.03f;
				online.diagnostics.outcome = ONLINE_MAG_UNCHANGED;
				if (stable
					&& (!online.dip_known || online.field <= 0 || fabsf(e->norm / online.field - 1) > 0.02f
						|| fabsf(e->old_dip - online.dip) > 0.03f)) {
					online.field = e->norm;
					online.dip = e->old_dip;
					online.dip_known = true;
					reference_locked(e->norm, e->old_dip);
					online.diagnostics.outcome = ONLINE_MAG_ENVIRONMENT;
				}
				event = stable && online.diagnostics.outcome == ONLINE_MAG_ENVIRONMENT
						  ? "reference-only update"
						  : "unchanged (reference unstable)";
				restart_locked(now);
			} else {
				online.candidate_dip_known
					= e->dip_known && online.validation_dip_known && fabsf(e->dip - online.validation_dip) <= 0.04f;
				online.reference_dip = e->dip;
				if (!online.candidate_dip_known) {
					online.diagnostics.last_gate
						= e->diagnostics.dip_delta > 0.04f ? ONLINE_MAG_REJECT_DIP : ONLINE_MAG_REJECT_COVERAGE;
				}
				online.phase = CONFIRMATION_READY;
			}
		}
	}
done:
	logged = online.diagnostics;
serviced:
	service_locked(now);
	k_spin_unlock(&online_lock, key);
	if (event) {
		log_snapshot(event, &logged);
	}
}

static void
validate_sample(const float raw[3], const float up[3], bool up_valid, uint32_t now, const struct sample_policy *p)
{
	float new_unit[3], old_unit[3], new_norm, old_norm = 0;
	if (!transform(candidate, raw, new_unit, &new_norm)
		|| (p->trusted && !transform(previous, raw, old_unit, &old_norm))) {
		finish_sample(now, p, ONLINE_MAG_REJECT_MATRIX, false, NULL);
		return;
	}
	float new_error = fabsf(new_norm / p->candidate_field - 1);
	/* Old geometry is assessed around its OWN mean radius, not 0.5 or VQF. */
	/* A gross environmental transition invalidates the whole episode, never
	 * a selectively clean subset that could make the candidate look good. */
	if (new_error > 0.30f && !online.unchanged) {
		struct evidence e = {0};
		e.diagnostics.max_radial_error = new_error;
		finish_sample(now, p, ONLINE_MAG_REJECT_RADIAL, false, &e);
		return;
	}
	struct direction_metrics *d = &directions[cell_of(new_unit)];
	if (d->count == UINT16_MAX) {
		finish_sample(now, p, ONLINE_MAG_REJECT_OVERFLOW, false, NULL);
		return;
	}
	if (!d->count) {
		d->old_min = INFINITY;
	}
	++d->count;
	d->old_sq += old_norm * old_norm;
	d->new_sq += new_error * new_error;
	d->old_max = fmaxf(d->old_max, old_norm);
	d->new_max = fmaxf(d->new_max, new_error);
	d->norm_sum += old_norm;
	if (p->trusted) {
		d->old_min = fminf(d->old_min, old_norm);
	}
	if (up_valid) {
		float new_dip = -asinf(CLAMP(dot3(new_unit, up), -1.0f, 1.0f));
		float old_dip = p->trusted ? -asinf(CLAMP(dot3(old_unit, up), -1.0f, 1.0f)) : 0;
		++d->dip_count;
		d->old_dip += old_dip;
		d->old_dip_sq += old_dip * old_dip;
		d->new_dip += new_dip;
		d->new_dip_sq += new_dip * new_dip;
	}
	/* Probation rolls back promptly on directional degradation, not only at
	 * its final score/timeout. Four observations reject a persistent bad cell. */
	bool degraded = p->phase == PROBATION && !online.unchanged && d->count >= 4 && d->new_sq / d->count > 0.01f;
	uint32_t minimum = p->phase == PROBATION ? ONLINE_PROBATION_MS : ONLINE_VALIDATE_MS;
	struct evidence e;
	bool scored = ELAPSED(now, p->episode) >= minimum;
	bool passed = (scored || degraded) && metrics_pass(p->trusted, p->phase == VALIDATING || online.unchanged, &e);
	passed &= scored;
	if (p->phase == PROBATION && scored) {
		bool dip_known = online.unchanged ? e.old_dip_known : e.dip_known;
		e.diagnostics.dip_delta = online.validation_dip_known && dip_known
									? fabsf((online.unchanged ? e.old_dip : e.dip) - online.validation_dip)
									: 0;
		/* Environment-only confirmation never prolongs an unstable reference.
		 * Replacement quality remains mandatory even if the old model recovered. */
		if (online.unchanged && e.diagnostics.rejection != ONLINE_MAG_REJECT_COVERAGE) {
			passed = true;
		}
	}
	enum online_mag_rejection terminal = degraded ? ONLINE_MAG_REJECT_RADIAL : ONLINE_MAG_REJECT_NONE;
	if (scored && !passed && e.diagnostics.rejection != ONLINE_MAG_REJECT_COVERAGE) {
		terminal = e.diagnostics.rejection;
	}
	finish_sample(now, p, terminal, passed, scored || degraded ? &e : NULL);
}

void sensor_calibration_online_mag_sample(const float raw[3], const float up[3], bool up_valid)
{
	uint32_t now = k_uptime_get_32();
	struct sample_policy p;
	float matrix[4][3];
	bool summary_due;
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	online.started = true;
	if (online.served != online.generation && online.trial) {
		struct online_mag_diagnostics logged = online.diagnostics;
		logged.outcome = ONLINE_MAG_REJECTED;
		logged.rejection = logged.last_gate = ONLINE_MAG_REJECT_CANCELLED;
		service_locked(now);
		k_spin_unlock(&online_lock, key);
		log_snapshot("cancelled trial retired", &logged);
		key = k_spin_lock(&online_lock);
	} else {
		service_locked(now);
	}
	if (online.served != online.generation || !online.enabled || online.suspended || (magneto_progress & 0x80)) {
		goto out;
	}
	if ((online.phase == VALIDATING || online.phase == PROBATION || online.phase == CONFIRMATION_READY)
		&& ELAPSED(now, online.episode) > ONLINE_EPISODE_TIMEOUT_MS) {
		online.diagnostics.outcome = ONLINE_MAG_REJECTED;
		online.diagnostics.rejection = ONLINE_MAG_REJECT_TIMEOUT;
		online.diagnostics.phase_age_ms = ELAPSED(now, online.episode);
		struct online_mag_diagnostics logged = online.diagnostics;
		restart_locked(now);
		k_spin_unlock(&online_lock, key);
		log_snapshot("timeout/rollback", &logged);
		return;
	}
	if (ELAPSED(now, online.last_sample) < ONLINE_INTERVAL_MS) {
		goto out;
	}
	if (online.phase != TRAINING && online.phase != VALIDATING && online.phase != PROBATION
		&& online.phase != CONFIRMATION_READY) {
		goto out;
	}
	if (online.phase == TRAINING && ELAPSED(now, online.suppress) < ONLINE_SUPPRESS_MS) {
		goto out;
	}
	p = (struct sample_policy){
		.generation = online.generation,
		.episode = online.episode,
		.field = online.field,
		.dip = online.dip,
		.candidate_field = online.candidate_field,
		.phase = online.phase,
		.trusted = online.trusted,
		.dip_known = online.dip_known
	};
	memcpy(p.center, online.center, sizeof(p.center));
	memcpy(p.last_dir, online.last_dir, sizeof(p.last_dir));
	memcpy(matrix, magBAinv, sizeof(matrix));
	summary_due = ELAPSED(now, online.summary_time) >= 1000U;
	online.last_sample = now;
	k_spin_unlock(&online_lock, key);

	/* No IRQ-off floating point loops, inverse trig, or structural checks. */
	if (!isfinite(dot3(raw, raw)) || dot3(raw, raw) < 1e-12f) {
		if (p.phase != TRAINING) {
			finish_sample(now, &p, ONLINE_MAG_REJECT_SAMPLE, false, NULL);
		}
		return;
	}
	if (summary_due) {
		summarize(now, p.generation, p.center);
	}
	bool trusted = p.phase == TRAINING ? has_model(matrix, p.trusted) : p.trusted;
	float dir[3], norm, field = p.field, dip = p.dip;
	bool dip_known = p.dip_known;
	if (p.phase == TRAINING && trusted && field <= 0 && sensor_fusion_get_mag_ref(&field, &dip) && isfinite(field)
		&& field > 0 && isfinite(dip)) {
		dip_known = true;
	}
	/* Collection geometry is independent of the old correction/reference.
	 * A stale hard-iron center must not crowd all new samples into one octant. */
	for (unsigned i = 0; i < 3; ++i) {
		dir[i] = raw[i] - p.center[i];
	}
	norm = sqrtf(dot3(dir, dir));
	if (norm < 1e-6f) {
		goto validate;
	}
	for (unsigned i = 0; i < 3; ++i) {
		dir[i] /= norm;
	}
	if (dot3(dir, p.last_dir) > 1 - magneto_online_min_dir_change_threshold()) {
		goto validate;
	}
	unsigned octant = (dir[0] < 0) + 2 * (dir[1] < 0) + 4 * (dir[2] < 0);
	key = k_spin_lock(&online_lock);
	/* Requesting freeze is not acknowledgement: never write after ACK. */
	if (online.generation == p.generation && online.phase == p.phase
		&& (online.phase == TRAINING || online.phase == VALIDATING || online.phase == PROBATION
			|| online.phase == CONFIRMATION_READY)) {
		if (online.phase == TRAINING) {
			online.trusted = trusted;
		}
		if (online.admitted_since_fit < UINT16_MAX) {
			++online.admitted_since_fit;
		}
		struct online_sample *slot = &pool[octant * ONLINE_PER_OCTANT + heads[octant]];
		memcpy(slot->raw, raw, sizeof(slot->raw));
		slot->time = now;
		heads[octant] = (heads[octant] + 1) % ONLINE_PER_OCTANT;
		if (counts[octant] < ONLINE_PER_OCTANT) {
			++counts[octant];
		}
		memcpy(online.last_dir, dir, sizeof(dir));
		if (online.phase == TRAINING && trusted && isfinite(field) && field > 0) {
			online.field = field;
			online.dip = dip;
			online.dip_known = dip_known;
		}
	}
	k_spin_unlock(&online_lock, key);
validate:
	if (p.phase == VALIDATING || p.phase == PROBATION) {
		validate_sample(raw, up, up_valid, now, &p);
	}
	return;
out:
	k_spin_unlock(&online_lock, key);
}

struct fit_context {
	uint32_t generation, time, started;
};
static bool fit_read(void *opaque, unsigned index, float out[3])
{
	const struct fit_context *ctx = opaque;
	if (index >= ONLINE_SLOTS || !occupied(index) || ELAPSED(ctx->time, pool[index].time) > ONLINE_TTL_MS) {
		return false;
	}
	memcpy(out, pool[index].raw, sizeof(pool[index].raw));
	return true;
}

static bool fit_poll(void *opaque)
{
	const struct fit_context *ctx = opaque;
	watchdog_feed(WDT_CHANNEL_CALIBRATION);
	k_msleep(1);
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	bool valid = online.generation == ctx->generation && online.enabled && !online.suspended
			  && ELAPSED(k_uptime_get_32(), ctx->started) < ONLINE_FIT_TIMEOUT_MS;
	k_spin_unlock(&online_lock, key);
	return valid;
}

static bool confirm(void)
{
	/* Storage lock precedes metadata lock everywhere. Confirmation is the ONLY
	 * online path writing retained matrix or marking MAIN_MAG_BIAS_ID dirty. */
	sys_warm_transaction_begin();
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	bool valid = online.phase == CONFIRMATION_READY && online.trial && online.generation == online.served
			  && online.enabled && !online.suspended
			  && ELAPSED(k_uptime_get_32(), online.episode) <= ONLINE_EPISODE_TIMEOUT_MS;
	if (valid) {
		memcpy(retained->magBAinv, candidate, sizeof(candidate));
		if (online.updates < UINT8_MAX) {
			++online.updates;
		}
		online.field = online.candidate_field;
		online.dip = online.reference_dip;
		online.dip_known = online.candidate_dip_known;
		reference_locked(online.dip_known ? online.field : 0, online.dip_known ? online.dip : 0);
		online.trusted = true;
		online.trial = false;
		retained->onlineMagState.update_count = online.updates;
		retained->onlineMagState.last_buf_avg_norm = online.field;
		online.diagnostics.outcome = ONLINE_MAG_UPDATED;
		online.diagnostics.rejection = ONLINE_MAG_REJECT_NONE;
		restart_locked(k_uptime_get_32());
	} else if (
		online.phase == CONFIRMATION_READY && ELAPSED(k_uptime_get_32(), online.episode) > ONLINE_EPISODE_TIMEOUT_MS
	) {
		online.diagnostics.outcome = ONLINE_MAG_REJECTED;
		online.diagnostics.rejection = ONLINE_MAG_REJECT_TIMEOUT;
		/* Sensor owns rollback; its next sample performs the timed-out restart. */
		online.phase = PROBATION;
	}
	struct online_mag_diagnostics logged = online.diagnostics;
	k_spin_unlock(&online_lock, key);
	if (valid) {
		sys_warm_transaction_mark(MAIN_MAG_BIAS_ID, &retained->magBAinv, sizeof(magBAinv));
	}
	sys_warm_transaction_end(valid);
	if (valid) {
		sensor_refresh_sensor_ids();
		log_snapshot("confirmed retained update (storage queued, not flash completion)", &logged);
	}
	return valid;
}

bool sensor_calibration_online_mag_check(void)
{
	uint32_t now = k_uptime_get_32();
	struct fit_context ctx;
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	if (online.phase == CONFIRMATION_READY) {
		k_spin_unlock(&online_lock, key);
		return confirm();
	}
	if (!online.enabled || online.suspended || online.fitter || online.phase != TRAINING
		|| online.served != online.generation || online.recent_count < MAG_CAL_MIN_SAMPLES
		|| online.admitted_since_fit < 32 || ELAPSED(now, online.last_check) < ONLINE_CHECK_MS
		|| (magneto_progress & 0x80)) {
		k_spin_unlock(&online_lock, key);
		return false;
	}
	online.last_check = now;
	online.admitted_since_fit = 0;
	online.episode = now;
	online.fitter = true;
	online.phase = FREEZE_REQUESTED;
	memset(&online.diagnostics, 0, sizeof(online.diagnostics));
	ctx = (struct fit_context){online.generation, now, now};
	k_spin_unlock(&online_lock, key);
	if (sensor_calibration_get_online_mag_debug()) {
		LOG_INF("Online mag fit start");
	}
	bool frozen = false, trusted = false;
	float field = 0;
	/* Missing/stopped magnetometer cannot strand the calibration thread. */
	while (ELAPSED(k_uptime_get_32(), now) < ONLINE_FREEZE_TIMEOUT_MS && fit_poll(&ctx)) {
		key = k_spin_lock(&online_lock);
		frozen = online.phase == FROZEN && online.generation == ctx.generation;
		if (frozen) {
			trusted = online.trusted;
			field = online.field;
		}
		k_spin_unlock(&online_lock, key);
		if (frozen) {
			break;
		}
	}
	int error = -1;
	float output[4][3];
	struct mag_fit_result result;
	if (frozen) {
		/* The prior initializer uses its current observed radius, independently
		 * of VQF/environment reference and the candidate's normalized radius. */
		if (trusted) {
			float sum = 0;
			unsigned n = 0;
			for (unsigned i = 0; i < ONLINE_SLOTS; ++i) {
				float raw[3], unit[3], norm;
				if (fit_read(&ctx, i, raw) && transform(previous, raw, unit, &norm)) {
					sum += norm;
					++n;
				}
				if ((i & 15U) == 15U && !fit_poll(&ctx)) {
					break;
				}
			}
			if (n) {
				field = sum / n;
			}
		}
		if (fit_poll(&ctx)) {
			error = magneto_robust_fit(
				ONLINE_SLOTS,
				fit_read,
				fit_poll,
				&ctx,
				trusted ? previous : NULL,
				field,
				output,
				&result
			);
		}
		if (error && trusted && fit_poll(&ctx)) {
			error = magneto_robust_fit(ONLINE_SLOTS, fit_read, fit_poll, &ctx, NULL, 0, output, &result);
		}
	}
	key = k_spin_lock(&online_lock);
	online.fitter = false;
	if (online.generation == ctx.generation) {
		online.diagnostics.fit_errno = error;
	}
	if (!error && online.generation == ctx.generation && online.enabled && !online.suspended) {
		memcpy(candidate, output, sizeof(candidate));
		online.candidate_field = result.field_norm;
		online.unchanged = false;
		online.diagnostics.outcome = ONLINE_MAG_NONE;
		online.diagnostics.rejection = ONLINE_MAG_REJECT_NONE;
		online.phase = VALIDATION_READY;
	} else if (online.generation == ctx.generation) {
		online.diagnostics.outcome = ONLINE_MAG_REJECTED;
		online.diagnostics.rejection = ONLINE_MAG_REJECT_FIT;
		online.diagnostics.last_gate = ONLINE_MAG_REJECT_FIT;
		restart_locked(k_uptime_get_32());
	}
	struct online_mag_diagnostics logged = online.diagnostics;
	k_spin_unlock(&online_lock, key);
	if (sensor_calibration_get_online_mag_debug()) {
		LOG_INF("Online mag fit result errno=%d", error);
	}
	if (error) {
		log_snapshot("fit rejected", &logged);
	}
	return false;
}

int sensor_calibration_online_mag_status(float *dir_bias)
{
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	bool valid = online.enabled && online.generation == online.served
			  && ELAPSED(k_uptime_get_32(), online.summary_time) <= ONLINE_TTL_MS;
	if (dir_bias) {
		*dir_bias = valid ? online.dir_bias : 1;
	}
	int count = valid ? online.recent_count : 0;
	k_spin_unlock(&online_lock, key);
	return count;
}

void sensor_calibration_online_mag_diagnostics(struct online_mag_diagnostics *out)
{
	float matrix[4][3];
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	*out = online.diagnostics;
	out->phase = online.phase;
	if (online.phase != TRAINING) {
		out->phase_age_ms = ELAPSED(k_uptime_get_32(), online.episode);
	}
	out->trial = online.trial;
	bool confirmed = online.trusted || online.trial;
	memcpy(matrix, magBAinv, sizeof(matrix));
	k_spin_unlock(&online_lock, key);
	out->has_model = has_model(matrix, confirmed);
}

void sensor_calibration_track_mag_norm(float norm)
{
	if (!isfinite(norm) || norm < 1e-6f) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	if (online.enabled) {
		if (!online.norm_count) {
			online.norm_mean = norm;
		}
		float diff = norm - online.norm_mean;
		online.norm_mean += 0.01f * diff;
		online.norm_var += 0.01f * (diff * diff - online.norm_var);
		++online.norm_count;
	}
	k_spin_unlock(&online_lock, key);
}

float sensor_calibration_get_mag_quality(void)
{
	k_spinlock_key_t key = k_spin_lock(&online_lock);
	float quality
		= online.norm_count >= 100 && online.norm_mean > 1e-6f ? sqrtf(online.norm_var) / online.norm_mean : 1;
	k_spin_unlock(&online_lock, key);
	return quality;
}
