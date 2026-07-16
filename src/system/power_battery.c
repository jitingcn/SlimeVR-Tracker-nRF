#include "globals.h"
#include "battery_tracker.h"
#include "status.h"

#include <string.h>
#include <stdint.h>

#include "power_battery.h"

#define BATTERY_SAMPLES 24
#define BATTERY_PLUG_DEBOUNCE_MS 500
#define BATTERY_PLUG_SETTLE_MS 3000

static int16_t calibrated_battery_pptt = -1;
static int16_t current_battery_pptt = INT16_MIN;
static int32_t hysteresis_pptt = -1;
static int32_t average_pptt = -1;
static int16_t last_pptt[BATTERY_SAMPLES - 1] = {[0 ... BATTERY_SAMPLES - 2] = -1};
static int last_pptt_index = 0;
static uint8_t samples = 0;
static bool battery_low = false;

static bool device_plugged = false;
static bool device_charged = false;
static int64_t last_plug_signal_change_ms = -BATTERY_PLUG_SETTLE_MS;

LOG_MODULE_REGISTER(power_battery, LOG_LEVEL_INF);

bool power_battery_pptt_is_valid(int16_t battery_pptt)
{
	return battery_pptt >= 0 && battery_pptt <= 10000;
}

static void reset_battery_filter(void)
{
	memset(last_pptt, -1, sizeof(last_pptt));
	last_pptt_index = 0;
	samples = 0;
	average_pptt = -1;
	hysteresis_pptt = -1;
}

bool power_battery_update_plugged_state(bool raw_device_plugged, int64_t now_ms)
{
	static bool initialized = false;
	static bool pending_device_plugged = false;
	static int64_t pending_device_plugged_since_ms = 0;

	if (!initialized)
	{
		initialized = true;
		pending_device_plugged = raw_device_plugged;
		pending_device_plugged_since_ms = now_ms;
		device_plugged = raw_device_plugged;
		if (device_plugged)
			set_status(SYS_STATUS_PLUGGED, true);
		last_plug_signal_change_ms = now_ms - BATTERY_PLUG_SETTLE_MS;
		return false;
	}

	if (raw_device_plugged != pending_device_plugged)
	{
		pending_device_plugged = raw_device_plugged;
		pending_device_plugged_since_ms = now_ms;
		last_plug_signal_change_ms = now_ms;
		reset_battery_filter();
	}

	if (pending_device_plugged != device_plugged
		&& now_ms - pending_device_plugged_since_ms >= BATTERY_PLUG_DEBOUNCE_MS)
	{
		device_plugged = pending_device_plugged;
		set_status(SYS_STATUS_PLUGGED, device_plugged);
		last_plug_signal_change_ms = now_ms;
		reset_battery_filter();
	}

	return pending_device_plugged != device_plugged;
}

bool power_battery_plug_signal_settling(bool plug_state_debouncing, int64_t now_ms)
{
	return plug_state_debouncing
		|| now_ms - last_plug_signal_change_ms < BATTERY_PLUG_SETTLE_MS;
}

static bool update_battery(int16_t battery_pptt)
{
	if (!power_battery_pptt_is_valid(battery_pptt))
		return false;

	// Plugged state will cause a sudden change in SOC >10%, so reset the sample array
	if (average_pptt >= 0 && NRFX_ABS(battery_pptt - average_pptt) > 1000)
	{
		if (!device_plugged)
			LOG_INF("Change to battery SOC: %5.2f%% -> %5.2f%%", (double)average_pptt / 100.0, (double)battery_pptt / 100.0);
		memset(last_pptt, -1, sizeof(last_pptt)); // reset array
		samples = 1;
	}

	// Initalize sorted array
	int16_t sorted_pptt[BATTERY_SAMPLES];
	memcpy(sorted_pptt, last_pptt, sizeof(last_pptt));
	sorted_pptt[BATTERY_SAMPLES - 1] = battery_pptt;

	// Now add the last reading to the sample array
	last_pptt[last_pptt_index] = battery_pptt;
	last_pptt_index++;
	last_pptt_index %= BATTERY_SAMPLES - 1;

	// Sort sample array
	for (int i = 1; i < BATTERY_SAMPLES; i++)
	{
		int16_t key = sorted_pptt[i];
		int8_t j = i - 1;
		while (j >= 0 && sorted_pptt[j] > key)
		{
			sorted_pptt[j + 1] = sorted_pptt[j];
			j = j - 1;
		}
		sorted_pptt[j + 1] = key;
	}

	// Average across median 75% of samples
	average_pptt = 0;
	uint8_t valid_samples = 0;
	for (uint8_t i = BATTERY_SAMPLES - (samples - samples / 8); i < (BATTERY_SAMPLES - samples / 8); i++)
	{
		if (sorted_pptt[i] != -1)
		{
			average_pptt += sorted_pptt[i];
			valid_samples++;
		}
	}
	if (valid_samples > 0)
		average_pptt /= valid_samples;
	else
		average_pptt = battery_pptt;

	// Store the average battery level with hysteresis (Effectively 100-10000 -> 1-100%)
	if (average_pptt + 100 < hysteresis_pptt) // Lower bound -100pptt
		hysteresis_pptt = average_pptt + 100;
	else if (average_pptt > hysteresis_pptt) // Upper bound +0pptt
		hysteresis_pptt = average_pptt;

	// 0% to battery tracker will reset it, as >1% to 0% is invalid change
	// Instead, remap 1-100 to 0-100
	current_battery_pptt = (hysteresis_pptt - 100) * 100 / 99;
	return true;
}

bool power_battery_device_plugged(void)
{
	return device_plugged;
}

bool power_battery_device_charged(void)
{
	return device_charged;
}

void power_battery_set_charged(bool charged)
{
	device_charged = charged;
}

int32_t power_battery_average_pptt(void)
{
	return average_pptt;
}

int16_t power_battery_current_pptt(void)
{
	return current_battery_pptt;
}

int16_t power_battery_calibrated_pptt(void)
{
	return calibrated_battery_pptt;
}

bool power_battery_is_low(void)
{
	return battery_low;
}

static void refresh_battery_low(bool battery_available)
{
	bool current_battery_pptt_valid = power_battery_pptt_is_valid(current_battery_pptt);
	if (battery_available && current_battery_pptt_valid && !battery_low && current_battery_pptt < 1000)
		battery_low = true;
	else if (!battery_available || !current_battery_pptt_valid || (battery_low && current_battery_pptt > 1000)) // hysteresis alrerady provided
		battery_low = false;
}

void power_battery_feed_and_track(bool battery_pptt_valid, bool plug_signal_settling,
				  int16_t battery_pptt, bool battery_available, int battery_mV)
{
	// Only feed valid SOC readings into the filter. ADC errors would
	// otherwise look like real 0%/100% jumps and poison the estimate.
	// USB/charger contact bounce also shifts the battery ADC, so wait for
	// the plug signal to settle before accepting the next SOC sample.
	if (battery_pptt_valid && !plug_signal_settling)
	{
		if (samples < BATTERY_SAMPLES)
			samples++;
		update_battery(battery_pptt);
	}

	refresh_battery_low(battery_available);
	bool current_battery_pptt_valid = power_battery_pptt_is_valid(current_battery_pptt);

	sys_update_battery_tracker_voltage(battery_mV, device_plugged || plug_signal_settling);
	if (current_battery_pptt_valid && !plug_signal_settling
		&& (samples == BATTERY_SAMPLES || device_plugged))
		sys_update_battery_tracker(current_battery_pptt, device_plugged);
	if (current_battery_pptt_valid)
		calibrated_battery_pptt = sys_get_calibrated_battery_pptt(current_battery_pptt);
}
