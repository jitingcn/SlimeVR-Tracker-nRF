#ifndef SLIMENRF_SYSTEM
#define SLIMENRF_SYSTEM

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "led.h"
#include "power.h"
#include "status.h"

#define RBT_CNT_ID 1
#define PAIRED_ID 2
#define MAIN_ACCEL_BIAS_ID 3
#define MAIN_GYRO_BIAS_ID 4
#define MAIN_MAG_BIAS_ID 5
#define MAIN_GYRO_SENS_ID 6
#define MAIN_ACC_6_BIAS_ID 7

#define BATT_STATS_LAST_RUN_ID 8
#define BATT_STATS_INTERVAL_0 9 // ID 9 to 28 (20 intervals)
#define BATT_STATS_CURVE_ID 29

#define MAIN_SENSOR_DATA_ID 30
#define RF_CHANNEL_ID 31
#define MAG_ENABLED_ID 37
#define TCAL_ENABLED_ID 38
#define MAG_ONLINE_CALIBRATION_ID 39

#define MAG_ONLINE_CALIBRATION_DEFAULT 0
#define MAG_ONLINE_CALIBRATION_ENABLED 1
#define MAG_ONLINE_CALIBRATION_DISABLED 2

#if CONFIG_SENSOR_USE_TCAL
#define MAIN_GYRO_TEMP_ID 32
#define MAIN_GYRO_TCAL_POINTS_ID 33
#define MAIN_GYRO_TCAL_COEFFS_ID 34
#define MAIN_GYRO_TCAL_STATE_ID  35
#define MAIN_GYRO_TCAL_CORRECTION_ID 36
#endif

void configure_sense_pins(void);

/* Complete RESETREAS snapshot captured once at PRE_KERNEL_1, before W1C. */
uint32_t sys_get_reset_reason(void);

uint8_t reboot_counter_read(void);
void reboot_counter_write(uint8_t reboot_counter);

/* Eager retained + NVS write, thread context only.
 * Returns 0 on persistence success, -EIO if NVS cannot initialize, or the
 * negative NVS write error. On failure retained RAM is still updated/sealed.
 */
int sys_write(uint16_t id, void *ptr, const void *data, size_t len);
void sys_write_warm(uint16_t id, void *retained_ptr, const void *data, size_t len);
void sys_warm_transaction_begin(void);
void sys_warm_transaction_mark(uint16_t id, void *retained_ptr, size_t len);
void sys_warm_transaction_end(bool retained_changed);
#if CONFIG_SENSOR_USE_TCAL
void sys_warm_feedback_arm(uint32_t identity); /* Call only with warm transaction held. */
#endif
int sys_flush_warm(void); /* Existing eager flush policy; negative persistence error. */
bool sys_warm_is_dirty(void);
void sys_read(uint16_t id, void *data, size_t len);
/* Caller-confirmed reset; cancels deferred IMU writes before clearing storage.
 * Storage errors are logged; live runtime settings still require a reboot. */
int sys_clear(void); /* Caller confirms destructive clear; 0: cleared, negative: failed. */
void sys_nvs_stats(void);

int set_sensor_clock(bool enable, float rate, float* actual_rate);

/* Boolean GPIO readers report active only for a positive sample. Read errors
 * are inactive, not evidence of a button gesture, dock or charging state. */
bool button_read(void);
bool button_read_filtered(void);

bool dock_read(void);
bool chg_read(void);
bool stby_read(void);
/* Fresh charger GPIO facts; active CHG wins completion conflicts. Inactive
 * CHG requires STBY or the board's explicit charger-full-on-plug heuristic
 * for completion; CHG-only boards remain unknown. Missing/failed evidence
 * returns negative without writing either output. */
int sys_charger_snapshot(bool *charging, bool *charged);

/* 0: power request accepted after a manual release-to-exit 1.8s window;
 * positive: deliberate long-hold cancellation; negative: admission rejected
 * (not a pairing request). Protective/automatic paths never use this window. */
int sys_user_shutdown(void);
/* 0: asynchronous OFF request accepted; negative: admission rejected. */
int sys_command_shutdown(void);
int sys_command_shutdown_request(uint32_t request, uint32_t accepted_event, uint32_t terminal_event);
int sys_enter_dfu(bool ota); /* Accepted handoff is not bootloader readiness. */
void sys_skip_dfu(void);
void sys_reset_mode(uint8_t mode);

#endif
