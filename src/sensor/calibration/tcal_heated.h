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
int sensor_tcal_heated_abort(enum tcal_heated_stop_reason reason);
void sensor_tcal_heated_set_ready(bool ready);
void sensor_tcal_heated_update(bool is_resting);
bool sensor_tcal_heated_busy(void);
void sensor_tcal_heated_report(void);

/* Sensor owner only. True suppresses the ordinary auto accumulator. */
bool sensor_tcal_heated_feed(const float g[3], float temp);
/* Existing calibration worker; never called with model/storage locks held. */
void sensor_tcal_heated_finalize(void);
/* Reset-all calls before taking storage and after releasing it. */
void sensor_tcal_heated_clear_begin(void);
void sensor_tcal_heated_clear_end(void);

/* Internal shared request/session gate. Lock order: storage -> gate -> model.
 * Hardware off is bounded; no gate owner waits for the sensor or flash. */
void sensor_tcal_heated_lock(void);
void sensor_tcal_heated_unlock(void);
bool sensor_tcal_heated_busy_locked(void);
void sensor_tcal_heated_accept_point(int idx, const float bias[3], float temp);
/* Sensor owner, gate held; accepted slots are immutable within a session. */
bool sensor_tcal_heated_slot_accepted(int idx);
bool sensor_tcal_heated_start_window(float temp);
int sensor_calibration_heated_reserve_locked(void);
void sensor_calibration_heated_release_locked(void);
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
