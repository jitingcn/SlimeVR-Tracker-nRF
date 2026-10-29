#ifndef SLIMENRF_TCAL_HEATED_H
#define SLIMENRF_TCAL_HEATED_H

#include <stdbool.h>
#include <stdint.h>

#if CONFIG_SENSOR_TCAL_HEATED
enum tcal_heated_stop_reason {
	TCAL_HEATED_STOP_NONE,
	TCAL_HEATED_STOP_SENSOR_STOP,
	TCAL_HEATED_STOP_POWER_DOWN,
	TCAL_HEATED_STOP_USER,
	TCAL_HEATED_STOP_TIMEOUT,
	TCAL_HEATED_STOP_STALE_TEMP,
	TCAL_HEATED_STOP_OVERTEMP,
	TCAL_HEATED_STOP_RISE_FAST,
	TCAL_HEATED_STOP_HEATER_ERROR,
	TCAL_HEATED_STOP_INSUFFICIENT_COVERAGE,
	TCAL_HEATED_STOP_COMPLETE,
};

int sensor_tcal_heated_start(float target_temp);
int sensor_tcal_heated_stop(void);
/* May run outside the sensor owner. Closes the session and requests an
 * owner-consumed reset; never directly mutates stage or accumulator state.
 * SENSOR_STOP closes readiness; POWER_DOWN also latches terminal admission. */
int sensor_tcal_heated_abort(enum tcal_heated_stop_reason reason);
void sensor_tcal_heated_set_ready(bool ready);
/* Sensor owner only; synchronizes the session and accumulator reset. */
void sensor_tcal_heated_update(bool is_resting);
bool sensor_tcal_heated_busy(void);
void sensor_tcal_heated_report(void);

/* Sensor owner only. True suppresses the ordinary auto accumulator. */
bool sensor_tcal_heated_feed(const float g[3], float temp);
/* Calibration worker only. Acquires storage itself; enter with no gate,
 * model, or storage lock held. Accepted stage slots stay frozen. */
void sensor_tcal_heated_finalize(void);
/* Reset-all calls before taking storage and after releasing it. */
void sensor_tcal_heated_clear_begin(void);
void sensor_tcal_heated_clear_end(void);

/* These wrap the existing calibration request mutex, not a separate lock.
 * Lock order is storage -> gate -> model. Never acquire storage with the
 * gate/model held. Hardware off is bounded; gate owners never wait for sensor. */
void sensor_tcal_heated_lock(void);
void sensor_tcal_heated_unlock(void);
/* All functions below with _locked require the gate held. */
bool sensor_tcal_heated_busy_locked(void);
/* Sensor owner and gate held. Stage slots are immutable once accepted. */
void sensor_tcal_heated_accept_point(int idx, const float bias[3], float temp);
/* Sensor owner and gate held for both queries below. */
bool sensor_tcal_heated_slot_accepted(int idx);
bool sensor_tcal_heated_start_window(float temp);
int sensor_calibration_heated_reserve_locked(void);
void sensor_calibration_heated_release_locked(void);
/* Begin reserves admission and returns without the gate held. Call end only
 * after successful begin; do not hold the gate across storage operations. */
int sensor_calibration_maintenance_begin(void);
void sensor_calibration_maintenance_end(void);
/* Allows replacing a sensitivity worker without taking or clearing its slot.
 * End is required only after a successful begin; no gate is held across I/O. */
int sensor_calibration_sensitivity_maintenance_begin(void);
void sensor_calibration_sensitivity_maintenance_end(void);
bool sensor_calibration_maintenance_active_locked(void);
bool sensor_tcal_heated_resetting_locked(void);
#endif

#endif
