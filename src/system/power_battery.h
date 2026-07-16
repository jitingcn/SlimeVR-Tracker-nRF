#ifndef SLIMENRF_SYSTEM_POWER_BATTERY
#define SLIMENRF_SYSTEM_POWER_BATTERY

#include <stdbool.h>
#include <stdint.h>

bool power_battery_pptt_is_valid(int16_t battery_pptt);

/* Debounce charger/USB/vin into device_plugged; may set SYS_STATUS_PLUGGED. */
bool power_battery_update_plugged_state(bool raw_device_plugged, int64_t now_ms);

bool power_battery_plug_signal_settling(bool plug_state_debouncing, int64_t now_ms);

/* Feed SOC filter (when settled), refresh low flag, update battery tracker + calibrated pptt. */
void power_battery_feed_and_track(bool battery_pptt_valid, bool plug_signal_settling,
				  int16_t battery_pptt, bool battery_available, int battery_mV);

bool power_battery_device_plugged(void);
bool power_battery_device_charged(void);
void power_battery_set_charged(bool charged);

int32_t power_battery_average_pptt(void);
int16_t power_battery_current_pptt(void);
int16_t power_battery_calibrated_pptt(void);

bool power_battery_is_low(void);

#endif
