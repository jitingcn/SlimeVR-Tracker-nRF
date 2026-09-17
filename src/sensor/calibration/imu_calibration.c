#include "globals.h"
#include "system/system.h"
#include "util.h"
#include "imu_calibration.h"
#include "connection/tracker_events.h"

#include <errno.h>
#include <math.h>
#include <string.h>
#if CONFIG_CMSIS_DSP
#include <arm_math.h>
#endif
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(imu_calibration, LOG_LEVEL_INF);

static struct k_spinlock coefficient_lock;
K_MUTEX_DEFINE(persistence_lock);
static sensor_imu_calibration_t applied;
static bool loaded;
static bool closing;
static bool consumer_ready;
static bool clearing;
static bool fusion_stale;
static bool fusion_save_pending;
static bool clear_event_pending;

/* One complete transaction: first accepted candidate cannot be overwritten.
 * No wait for the sensor thread (calibration can temporarily suspend it). */
static struct {
	sensor_imu_calibration_t coefficients;
	enum sensor_calibration_effect effect;
	uint16_t operation_id;
	bool bias;
	bool matrix;
	bool persist_gyro;
	bool clear_boot_offset;
	bool persist;
	bool fusion_changed;
} pending;

void sensor_calibration_identity_accel(float matrix[4][3])
{
	memset(matrix, 0, sizeof(applied.accel_matrix));
	for (int i = 0; i < 3; i++) {
		matrix[i + 1][i] = 1.0f;
	}
}

void sensor_calibration_imu_load(void)
{
	k_mutex_lock(&persistence_lock, K_FOREVER);
	k_spinlock_key_t key = k_spin_lock(&coefficient_lock);
	/* Startup calibration thread calls once; a later reload is a no-op. */
	if (loaded) {
		k_spin_unlock(&coefficient_lock, key);
		k_mutex_unlock(&persistence_lock);
		return;
	}
	k_spin_unlock(&coefficient_lock, key);
	sensor_imu_calibration_t initial;
	memcpy(initial.accel_bias, retained->accelBias, sizeof(initial.accel_bias));
	memcpy(initial.gyro_bias, retained->gyroBias, sizeof(initial.gyro_bias));
	memcpy(initial.accel_matrix, retained->accBAinv, sizeof(initial.accel_matrix));
	float zero[3] = {0};
	bool heal_bias = !v_finite(initial.accel_bias, 3) || !v_finite(initial.gyro_bias, 3)
		|| !v_epsilon(initial.accel_bias, zero, 0.5f)
		|| !v_epsilon(initial.gyro_bias, zero, 50.0f);
	if (heal_bias) {
		memset(initial.accel_bias, 0, sizeof(initial.accel_bias));
		memset(initial.gyro_bias, 0, sizeof(initial.gyro_bias));
		sys_write(MAIN_ACCEL_BIAS_ID, &retained->accelBias, initial.accel_bias, sizeof(initial.accel_bias));
		sys_write(MAIN_GYRO_BIAS_ID, &retained->gyroBias, initial.gyro_bias, sizeof(initial.gyro_bias));
	}
#if CONFIG_SENSOR_USE_TCAL
	if (heal_bias) {
		retained->bootCalState.doffset_valid = false;
		memset(retained->bootCalState.doffset, 0, sizeof(retained->bootCalState.doffset));
	}
#endif
	bool heal_matrix = !v_finite(&initial.accel_matrix[0][0], 12);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	float diagonal[3] = {initial.accel_matrix[1][0], initial.accel_matrix[2][1], initial.accel_matrix[3][2]};
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	heal_matrix |= !v_epsilon(initial.accel_matrix[0], zero, 0.5f)
		|| !v_epsilon(diagonal, average, magnitude * 0.1f);
#endif
	if (heal_matrix) {
		sensor_calibration_identity_accel(initial.accel_matrix);
		sys_write(MAIN_ACC_6_BIAS_ID, &retained->accBAinv, initial.accel_matrix, sizeof(initial.accel_matrix));
	}
	if (heal_bias || heal_matrix) {
		retained_update();
		LOG_WRN("Invalid stored IMU calibration cleared before sampling");
	}
	key = k_spin_lock(&coefficient_lock);
	applied = initial;
	/* Startup validation historically preserves the recovered fusion estimate. */
	fusion_stale = false;
	loaded = true;
	k_spin_unlock(&coefficient_lock, key);
	k_mutex_unlock(&persistence_lock);
}

bool sensor_calibration_imu_ready(void)
{
	k_spinlock_key_t key = k_spin_lock(&coefficient_lock);
	bool ready = loaded;
	k_spin_unlock(&coefficient_lock, key);
	return ready;
}

void sensor_calibration_set_consumer_ready(bool ready)
{
	k_spinlock_key_t key = k_spin_lock(&coefficient_lock);
	consumer_ready = ready;
	k_spin_unlock(&coefficient_lock, key);
}

void sensor_calibration_snapshot(sensor_imu_calibration_t *out)
{
	k_spinlock_key_t key = k_spin_lock(&coefficient_lock);
	*out = applied;
	k_spin_unlock(&coefficient_lock, key);
}

static int reserve_candidate(void)
{
	if (closing) {
		return -ESHUTDOWN;
	}
	if (!loaded || !consumer_ready) {
		return -EAGAIN;
	}
	return clearing || pending.effect != SENSOR_CALIBRATION_UNCHANGED || pending.persist ? -EBUSY : 0;
}

static int submit_bias(const float a_bias[3], const float g_bias[3], bool persist_gyro, bool reset,
		       uint16_t operation_id)
{
	float zero[3] = {0};
	if (!a_bias || !g_bias || !v_finite(a_bias, 3) || !v_finite(g_bias, 3)
	    || !v_epsilon(a_bias, zero, 0.5f) || !v_epsilon(g_bias, zero, 50.0f)) {
		return -EINVAL;
	}
	k_spinlock_key_t key = k_spin_lock(&coefficient_lock);
	int err = reserve_candidate();
	if (!err) {
		pending.coefficients = applied;
		memcpy(pending.coefficients.accel_bias, a_bias, sizeof(applied.accel_bias));
		memcpy(pending.coefficients.gyro_bias, g_bias, sizeof(applied.gyro_bias));
		pending.bias = true;
		pending.matrix = false;
		pending.persist_gyro = persist_gyro;
		pending.clear_boot_offset = reset;
		pending.effect = reset ? SENSOR_CALIBRATION_FRAME_CHANGED : SENSOR_CALIBRATION_BIAS_CHANGED;
		pending.operation_id = operation_id;
	}
	k_spin_unlock(&coefficient_lock, key);
	return err;
}

int sensor_calibration_commit_bias(const float a_bias[3], const float g_bias[3], bool persist_gyro,
				   uint16_t operation_id)
{
	return submit_bias(a_bias, g_bias, persist_gyro, false, operation_id);
}

int sensor_calibration_reset_imu(void)
{
	float zero[3] = {0};
	return submit_bias(zero, zero, true, true, 0);
}

static int submit_accel(const float matrix[4][3], enum sensor_calibration_effect effect,
			uint16_t operation_id)
{
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	float zero[3] = {0};
	if (!matrix || !v_finite(&matrix[0][0], 12)) {
		return -EINVAL;
	}
	float diagonal[3] = {matrix[1][0], matrix[2][1], matrix[3][2]};
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	if (!v_epsilon(matrix[0], zero, 0.5f) || !v_epsilon(diagonal, average, magnitude * 0.1f)) {
		return -EINVAL;
	}
	k_spinlock_key_t key = k_spin_lock(&coefficient_lock);
	int err = reserve_candidate();
	if (!err) {
		pending.coefficients = applied;
		memcpy(pending.coefficients.accel_matrix, matrix, sizeof(applied.accel_matrix));
		pending.bias = false;
		pending.matrix = true;
		pending.clear_boot_offset = false;
		pending.effect = effect;
		pending.operation_id = operation_id;
	}
	k_spin_unlock(&coefficient_lock, key);
	return err;
#else
	(void)matrix;
	(void)effect;
	(void)operation_id;
	return -ENOTSUP;
#endif
}

int sensor_calibration_commit_accel(const float matrix[4][3], uint16_t operation_id)
{
	return submit_accel(matrix, SENSOR_CALIBRATION_FRAME_CHANGED, operation_id);
}

int sensor_calibration_reset_accel(void)
{
	float identity[4][3];
	sensor_calibration_identity_accel(identity);
	return submit_accel(identity, SENSOR_CALIBRATION_COEFFICIENTS_CHANGED, 0);
}

/* Called by the sensor at a frame boundary, or by power after suspension.
 * Publish under a short lock; NVS/retained work never holds the spinlock. */
enum sensor_calibration_effect sensor_calibration_apply_pending(void)
{
	k_spinlock_key_t key = k_spin_lock(&coefficient_lock);
	if (clearing) {
		k_spin_unlock(&coefficient_lock, key);
		return SENSOR_CALIBRATION_UNCHANGED;
	}
	enum sensor_calibration_effect effect = pending.effect;
	if (effect == SENSOR_CALIBRATION_UNCHANGED) {
		if (fusion_stale) {
			effect = SENSOR_CALIBRATION_FRAME_CHANGED;
		} else if (fusion_save_pending) {
			fusion_save_pending = false;
			effect = SENSOR_CALIBRATION_SAVE_FUSION;
		}
		k_spin_unlock(&coefficient_lock, key);
		return effect;
	}
	applied = pending.coefficients;
	bool clear_boot_offset = pending.clear_boot_offset;
	fusion_stale |= effect != SENSOR_CALIBRATION_COEFFICIENTS_CHANGED;
	pending.fusion_changed = effect != SENSOR_CALIBRATION_COEFFICIENTS_CHANGED;
	pending.effect = SENSOR_CALIBRATION_UNCHANGED;
#if CONFIG_SENSOR_USE_TCAL
	if (clear_boot_offset) {
		retained->bootCalState.doffset_valid = false;
		memset(retained->bootCalState.doffset, 0, sizeof(retained->bootCalState.doffset));
	}
#else
	(void)clear_boot_offset;
#endif
	pending.persist = true;
	uint16_t operation_id = pending.operation_id;
	cal_event_end(operation_id, CAL_OUTCOME_SUCCESS, CAL_PHASE_APPLIED, CAL_REASON_NONE);
	k_spin_unlock(&coefficient_lock, key);
	if (operation_id) {
		tracker_events_notify();
	}
	return effect;
}

/* Existing calibration worker drains flash. Power may drain synchronously
 * only after stopping the sensor. Neither path waits for the sensor owner. */
void sensor_calibration_persist_pending(void)
{
	k_mutex_lock(&persistence_lock, K_FOREVER);
	k_spinlock_key_t key = k_spin_lock(&coefficient_lock);
	if (!pending.persist) {
		k_spin_unlock(&coefficient_lock, key);
		k_mutex_unlock(&persistence_lock);
		return;
	}
	/* Admission stays closed until persistence completes, so this transaction
	 * remains immutable while the spinlock is released for flash operations. */
	sensor_imu_calibration_t next = pending.coefficients;
	bool bias = pending.bias;
	bool matrix = pending.matrix;
	bool persist_gyro = pending.persist_gyro;
	uint16_t operation_id = pending.operation_id;
	k_spin_unlock(&coefficient_lock, key);
	int err = 0;
	if (bias) {
#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		err = sys_write(MAIN_ACCEL_BIAS_ID, &retained->accelBias, next.accel_bias, sizeof(next.accel_bias));
#endif
		if (persist_gyro) {
			int gyro_err = sys_write(MAIN_GYRO_BIAS_ID, &retained->gyroBias, next.gyro_bias, sizeof(next.gyro_bias));
			if (gyro_err < 0) {
				err = gyro_err;
			}
		}
	}
	if (matrix) {
		err = sys_write(MAIN_ACC_6_BIAS_ID, &retained->accBAinv, next.accel_matrix, sizeof(next.accel_matrix));
	}
	retained_update();
	if (err < 0) {
		LOG_ERR("Calibration applied in RAM; persistence failed: %d", err);
	}
	key = k_spin_lock(&coefficient_lock);
	if (err < 0) {
		cal_event_step(operation_id, CAL_PHASE_STORAGE, CAL_REASON_STORAGE_ERROR);
	}
	pending.persist = false;
	pending.operation_id = 0;
	fusion_save_pending |= pending.fusion_changed;
	k_spin_unlock(&coefficient_lock, key);
	k_mutex_unlock(&persistence_lock);
	if (err < 0 && operation_id) {
		tracker_events_notify();
	}
}

bool sensor_calibration_fusion_stale(void)
{
	k_spinlock_key_t key = k_spin_lock(&coefficient_lock);
	bool stale = fusion_stale || pending.effect == SENSOR_CALIBRATION_BIAS_CHANGED
		|| pending.effect == SENSOR_CALIBRATION_FRAME_CHANGED
		|| (pending.persist && pending.fusion_changed);
	k_spin_unlock(&coefficient_lock, key);
	return stale;
}

void sensor_calibration_fusion_applied(void)
{
	k_spinlock_key_t key = k_spin_lock(&coefficient_lock);
	fusion_stale = false;
	k_spin_unlock(&coefficient_lock, key);
}

void sensor_calibration_prepare_power_down(void)
{
	k_spinlock_key_t key = k_spin_lock(&coefficient_lock);
	closing = true;
	k_spin_unlock(&coefficient_lock, key);
	(void)sensor_calibration_apply_pending();
	sensor_calibration_persist_pending();
	if (sensor_calibration_fusion_stale()) {
		retained->fusion_id = 0;
		retained_update();
	}
}

/* Reset-all owns persistence before taking system storage. A worker already in
 * flash finishes first; queued/applied-but-unwritten transactions are discarded.
 * Application and admission share this gate, so neither can cross the clear.
 * Live coefficients are unchanged, just as for the other reset-all runtime state. */
void sensor_calibration_clear_begin(void)
{
	k_mutex_lock(&persistence_lock, K_FOREVER);
	k_spinlock_key_t key = k_spin_lock(&coefficient_lock);
	clearing = true;
	if (pending.effect != SENSOR_CALIBRATION_UNCHANGED) {
		cal_event_end(pending.operation_id, CAL_OUTCOME_CANCELLED, CAL_PHASE_APPLY_PENDING, CAL_REASON_RESET);
		clear_event_pending = pending.operation_id != 0;
	}
	memset(&pending, 0, sizeof(pending));
	fusion_save_pending = false;
	k_spin_unlock(&coefficient_lock, key);
}

void sensor_calibration_clear_end(void)
{
	k_spinlock_key_t key = k_spin_lock(&coefficient_lock);
	clearing = false;
	bool notify = clear_event_pending;
	clear_event_pending = false;
	k_spin_unlock(&coefficient_lock, key);
	k_mutex_unlock(&persistence_lock);
	if (notify) {
		tracker_events_notify();
	}
}

/* Sample consumers and frame publication share the sensor owner, so no lock
 * or per-sample matrix copy is needed. External readers use snapshot above. */
void sensor_calibration_apply_accel(float a[3])
{
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	apply_BAinv(a, applied.accel_matrix);
#else
	(void)a;
#endif
}

void sensor_calibration_subtract_gyro_bias(float g[3])
{
#if CONFIG_CMSIS_DSP
	arm_sub_f32(g, applied.gyro_bias, g, 3);
#else
	for (int i = 0; i < 3; i++) {
		g[i] -= applied.gyro_bias[i];
	}
#endif
}

void sensor_calibration_gyro_bias(float out[3])
{
	memcpy(out, applied.gyro_bias, sizeof(applied.gyro_bias));
}
