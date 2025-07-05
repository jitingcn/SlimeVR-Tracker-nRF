#ifndef SLIMENRF_SYSTEM_BATTERY_TRACKER
#define SLIMENRF_SYSTEM_BATTERY_TRACKER

void sys_update_battery_tracker_voltage(int mV, bool plugged);
void sys_update_battery_tracker(int16_t pptt, bool plugged);

int16_t sys_get_calibrated_battery_pptt(int16_t pptt);

int sys_get_battery_mV(void);
int sys_get_valid_battery_mV(void);
int16_t sys_get_valid_battery_pptt(void);
uint64_t sys_get_last_unplugged_time(void);

uint64_t sys_get_battery_runtime_estimate(void);
uint64_t sys_get_battery_runtime_min_estimate(void);
uint64_t sys_get_battery_runtime_max_estimate(void);
uint64_t sys_get_battery_remaining_time_estimate(void);
float sys_get_battery_cycles(void);

float sys_get_battery_calibration_coverage(void);
int16_t sys_get_calibrated_battery_range_min_pptt(void);
int16_t sys_get_calibrated_battery_range_max_pptt(void);

int16_t sys_get_last_cycle_min_pptt(void);
int16_t sys_get_last_cycle_max_pptt(void);
uint64_t sys_get_last_cycle_runtime(void);

void sys_reset_battery_tracker(void);

#endif