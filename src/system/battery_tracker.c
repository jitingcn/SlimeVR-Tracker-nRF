#include "globals.h"
#include "system/system.h"

#include <zephyr/kernel.h>

#include "battery_tracker.h"

static uint8_t valid_result = 0; // track when data should be recalculated

// #define DEBUG true

LOG_MODULE_REGISTER(battery_tracker, LOG_LEVEL_INF);

struct battery_tracker
{
	int16_t last_max_battery_pptt;
	int16_t last_min_battery_pptt;
	uint64_t last_battery_runtime;
};

struct battery_tracker_interval
{
	uint16_t cycles;
	uint64_t runtime;
	uint64_t runtime_min;
	uint64_t runtime_max;
};

static bool battery_pptt_is_valid(int16_t pptt)
{
	return pptt >= 0 && pptt <= 10000;
}

static void update_statistics(void)
{
	if (retained->max_battery_pptt - retained->min_battery_pptt < 1000)
		return;

	// save last statistics
	struct battery_tracker tracker;
	tracker.last_max_battery_pptt = retained->max_battery_pptt;
	tracker.last_min_battery_pptt = retained->min_battery_pptt;
	tracker.last_battery_runtime = retained->battery_runtime_sum;

	sys_write(BATT_STATS_LAST_RUN_ID, NULL, &tracker, sizeof(tracker));
	LOG_DBG("Discharge: %6.2f%% -> %5.2f%%, %llu us", (double)tracker.last_max_battery_pptt / 100.0, (double)tracker.last_min_battery_pptt / 100.0, k_ticks_to_us_floor64(tracker.last_battery_runtime));
}

static void update_runtime(void)
{
	uint64_t now = k_uptime_ticks();
	retained->battery_runtime_sum += (now - retained->battery_uptime_latest);
	retained->battery_uptime_latest = now;
}

static void reset_tracker(int16_t pptt)
{
	update_runtime(); // update battery_runtime_sum before resetting

	retained->max_battery_pptt = pptt; // reset
	retained->min_battery_pptt = pptt;
	retained->battery_runtime_sum = 0;
	retained->battery_runtime_saved = 0;
	retained->battery_pptt_saved = (pptt + 499) / 500 * 500;
	if (pptt >= 0)
		LOG_INF("Reset battery tracker: start=%.2f%%, saved=%.2f%%, first interval below %.2f%%, valid below %.2f%%",
			(double)pptt / 100.0,
			(double)retained->battery_pptt_saved / 100.0,
			(double)(retained->battery_pptt_saved - 500) / 100.0,
			(double)(pptt - 500) / 100.0);
	else
		LOG_DBG("Reset battery tracker");
}

static void update_interval(int16_t pptt)
{
	update_runtime(); // update battery_runtime_sum before saving

	// Map pptt to a 5% bucket index.
	// Valid NVS interval IDs are 0..18 (i.e. 0-95%). Clamp defensively.
	uint8_t interval_id = (pptt + 499) / 500;
	if (interval_id > 18)
		interval_id = 18;
	uint64_t runtime = retained->battery_runtime_sum - retained->battery_runtime_saved;

	LOG_INF("update_interval: pptt=%.2f%%, interval_id=%u, runtime=%llu ticks (%llu us)",
		(double)pptt / 100.0, interval_id, runtime, k_ticks_to_us_floor64(runtime));

	// Minimum 5 minute of active runtime per 5% interval to be considered valid
	// Note: This only counts active time, not sleep time in WoM mode
	if (runtime < CONFIG_SYS_CLOCK_TICKS_PER_SEC * 60 * 5)
	{
		LOG_ERR("Interval %u: %llu us is too short (min 5 min active time)", interval_id, k_ticks_to_us_floor64(runtime));
		return;
	}

	struct battery_tracker_interval interval = {0};
	sys_read(BATT_STATS_INTERVAL_0 + interval_id, &interval, sizeof(interval));

	// TODO: can use nvs_read_hist
	interval.cycles++;
	interval.runtime += runtime;
	if (runtime < interval.runtime_min || interval.runtime_min == 0)
		interval.runtime_min = runtime;
	if (runtime > interval.runtime_max)
		interval.runtime_max = runtime;
	sys_write(BATT_STATS_INTERVAL_0 + interval_id, NULL, &interval, sizeof(interval));
	valid_result = 0; // invalidate all
	LOG_INF("Interval %u saved: %u cycles, %llu us total (current: %llu us, min: %llu us, max: %llu us)",
		interval_id, interval.cycles, k_ticks_to_us_floor64(interval.runtime),
		k_ticks_to_us_floor64(runtime), k_ticks_to_us_floor64(interval.runtime_min), k_ticks_to_us_floor64(interval.runtime_max));
}

static void update_tracker(int16_t pptt)
{
	if (pptt < retained->min_battery_pptt)
		retained->min_battery_pptt = pptt;

	if (pptt <= retained->max_battery_pptt - 300) // valid pptt
	{
		if (pptt <= retained->battery_pptt_saved - 500) // new interval
		{
			LOG_INF("New interval: %.2f%%", (double)pptt / 100.0);
			if (pptt <= retained->max_battery_pptt - 800) // valid interval
			{
				LOG_INF("Update interval: %.2f%%", (double)pptt / 100.0);
				update_interval(pptt);
			}
			retained->battery_runtime_saved = retained->battery_runtime_sum;
			retained->battery_pptt_saved -= 500;
		}
	}
}

static void update_curve(void)
{
	uint64_t* intervals = (uint64_t*)k_malloc(sizeof(uint64_t) * 19);
	uint64_t curve_runtime = 0;

	int8_t first_valid = -1;
	int8_t last_valid = -1;
	uint8_t valid_intervals = 0;

	// read intervals
	for (uint8_t i = 0; i < 19; i++)
	{
		struct battery_tracker_interval interval = {0};
		sys_read(BATT_STATS_INTERVAL_0 + i, &interval, sizeof(interval));
#if DEBUG
		LOG_DBG("Interval %u: %u cycles, %llu us", i, interval.cycles, k_ticks_to_us_floor64(interval.runtime));
#endif
		if (interval.cycles > 0)
		{
			uint64_t interval_runtime = interval.runtime / interval.cycles;
			intervals[i] = interval_runtime;
			curve_runtime += interval_runtime;
			if (first_valid < 0)
				first_valid = i;
			last_valid = i;
			valid_intervals++;
			// the valid intervals should be continuous, if there is a gap the average is used instead
		}
		else
		{
			intervals[i] = 0;
		}
	}

	if (valid_intervals < 2 || first_valid < 0) // not enough data
	{
		LOG_WRN("Not enough data to calculate discharge curve");
		k_free(intervals);
		return;
	}

	int16_t* curve = (int16_t*)k_malloc(sizeof(int16_t) * 18);
	memset(curve, 0, sizeof(int16_t) * 18);
	uint64_t runtime = 0;

	// curve can be calculated for intervals between first_valid and last_valid
	int16_t curve_start = first_valid * 500;
	int16_t curve_size = (last_valid + 1 - first_valid) * 500;

	// add missing intervals
	uint64_t average_runtime = curve_runtime / valid_intervals;
	curve_runtime += average_runtime * (last_valid + 1 - first_valid - valid_intervals);

	// calculate correct pptt for each interval from its proportional runtime
	for (uint8_t i = first_valid; i < last_valid; i++)
	{
		runtime += intervals[i] ? intervals[i] : average_runtime;
		curve[i] = runtime * curve_size / curve_runtime + curve_start;
#if DEBUG
		LOG_DBG("Map %5.2f%% -> %5.2f%%, %llu us", (i + 1) * 5.0, (double)curve[i] / 100.0, k_ticks_to_us_floor64(intervals[i]));
#endif
	}
	k_free(intervals);

	sys_write(BATT_STATS_CURVE_ID, retained->battery_pptt_curve, curve, sizeof(int16_t) * 18);
	k_free(curve);
	valid_result &= (uint8_t)~8; // invalidate remaining runtime (curve changed)
}

static int16_t apply_curve(int16_t pptt)
{
	if (!battery_pptt_is_valid(pptt))
		return -1;

	uint8_t interval_id = pptt / 500; // above point
	int16_t pb = (interval_id > 0 && interval_id < 19) ? retained->battery_pptt_curve[interval_id - 1] : 0;
	pb = pb ? pb : interval_id * 500;
	int16_t pa = (interval_id < 18) ? retained->battery_pptt_curve[interval_id] : 0;
	pa = pa ? pa : (interval_id + 1) * 500;
	if (pb < 0 || pa < 0 || pb > 10000 || pa > 10500 || pa < pb)
	{
		LOG_ERR("Invalid curve");
		return pptt;
	}
	return (int32_t)(pptt % 500) * (pa - pb) / 500 + pb; // linear interpolation
}

static int last_mV = -1;
static int last_unplugged_mV = -1;
static int16_t last_unplugged_pptt = -1;
static uint64_t last_unplugged_time = 0;
static uint64_t last_unplugged_runtime = 0;

static int16_t last_saved_pptt = -1;
static uint64_t last_saved_time = 0;

void sys_update_battery_tracker_voltage(int mV, bool plugged)
{
	last_mV = mV;
	if (!plugged)
	{
		last_unplugged_mV = mV;
		last_unplugged_time = k_uptime_ticks();
	}
}

// Check if tracker state is valid (properly initialized)
static bool is_tracker_valid(void)
{
	// Tracker is valid if max_battery_pptt is a reasonable battery percentage (0-100%)
	return retained->max_battery_pptt >= 0 && retained->max_battery_pptt <= 10000;
}

void sys_update_battery_tracker(int16_t pptt, bool plugged)
{
	LOG_DBG("sys_update_battery_tracker: pptt=%.2f%%, plugged=%d, tracker_valid=%d",
		(double)pptt / 100.0, plugged, is_tracker_valid());

	if (plugged)
	{
		last_saved_pptt = -1; // reset saved pptt
	}

	// Skip update if pptt is invalid (not yet read from ADC)
	if (!battery_pptt_is_valid(pptt))
	{
		LOG_DBG("Skipping battery tracker update: pptt=%d (invalid)", pptt);
		return;
	}

	if (!plugged)
	{
		if (last_unplugged_pptt != pptt)
			valid_result &= (uint8_t)~8; // invalidate remaining runtime (pptt changed)
		last_unplugged_pptt = pptt;
		last_unplugged_time = k_uptime_ticks();
		last_unplugged_runtime = retained->battery_runtime_sum;
		if (last_saved_pptt == -1)
		{
			last_saved_pptt = pptt;
			last_saved_time = k_uptime_ticks();
		}
	}

	// Handle tracker state transitions
	if (plugged)
	{
		// When plugged in with valid tracker data, save statistics and reset
		if (is_tracker_valid())
		{
			LOG_INF("Tracker reset (charging)");
			update_statistics();
			reset_tracker(-1);
			update_curve(); // recalculate curve for next discharge
		}
		// If not valid, just stay in uninitialized state (will init when unplugged)
	}
	else if (!is_tracker_valid())
	{
		// Unplugged with invalid/uninitialized tracker - initialize it
		LOG_INF("Tracker initialized: %.2f%%", (double)pptt / 100.0);
		reset_tracker(pptt);
	}
	else
	{
		// Normal unplugged operation with valid tracker (pptt already validated >= 0)
		if (pptt < retained->min_battery_pptt - 100) // discharge (caused by long shutdown) event
		{
			LOG_WRN("Unaccounted change to battery SOC: %.2f%% (min) -> %.2f%%", (double)retained->min_battery_pptt / 100.0, (double)pptt / 100.0);
			update_statistics();
			reset_tracker(pptt);
		}
		else if (pptt > retained->min_battery_pptt + 100) // possible charge (should not happen!) event
		{
			LOG_ERR("Abnormal change to battery SOC: %.2f%% (min) -> %.2f%%", (double)retained->min_battery_pptt / 100.0, (double)pptt / 100.0);
			update_statistics();
			reset_tracker(pptt);
			update_curve(); // it is also possible for a device to have no usable charge indicators
		}
		else if (pptt > retained->max_battery_pptt + 100) // charge (should not happen!) event
		{
			LOG_ERR("Abnormal change to battery SOC: %.2f%% (max) -> %.2f%%", (double)retained->max_battery_pptt / 100.0, (double)pptt / 100.0);
			update_statistics();
			reset_tracker(pptt);
		}
		else if (last_saved_pptt != -1 && pptt < last_saved_pptt - 100 && k_uptime_ticks() - last_saved_time <= CONFIG_SYS_CLOCK_TICKS_PER_SEC * 60) // rapid "discharge" (ex. after unplugging)
		{
			uint64_t now = k_uptime_ticks();
			uint64_t delta = k_ticks_to_us_floor64(now - last_saved_time);
			LOG_INF("Rapid SOC change: %.2f%%/min (%.2f%% -> %.2f%% in %llu us)", (double)(pptt - last_saved_pptt) / 100.0 / ((double)delta / 60000000), (double)last_saved_pptt / 100.0, (double)pptt / 100.0, delta);
			update_statistics();
			reset_tracker(pptt);
			last_saved_pptt = pptt; // reset saved pptt
			last_saved_time = now;
		}
		else
		{
			update_tracker(pptt);
			update_runtime();
		}
	}

	retained_update();
}

static int16_t last_pptt = -1;
static int16_t last_calibrated_battery_pptt = -1;

int16_t sys_get_calibrated_battery_pptt(int16_t pptt)
{
	if (!battery_pptt_is_valid(pptt))
		return -1;

	if (pptt == last_pptt)
		return last_calibrated_battery_pptt;
	last_pptt = pptt;
	last_calibrated_battery_pptt = apply_curve(pptt);
	return last_calibrated_battery_pptt;
}

int sys_get_battery_mV(void)
{
	return last_mV;
}

int sys_get_valid_battery_mV(void)
{
	if (last_unplugged_mV > 1500 && last_unplugged_mV <= 6000)
		return last_unplugged_mV;
	return -1;
}

int16_t sys_get_valid_battery_pptt(void)
{
	return last_unplugged_pptt;
}

uint64_t sys_get_last_unplugged_time(void)
{
	return last_unplugged_time;
}

uint64_t sys_get_battery_runtime_estimate(void)
{
	static uint64_t runtime = 0;
	uint8_t valid_intervals = 0;

	if (valid_result & 1)
		return runtime;
	runtime = 0;

	for (uint8_t i = 0; i < 19; i++)
	{
		struct battery_tracker_interval interval = {0};
		sys_read(BATT_STATS_INTERVAL_0 + i, &interval, sizeof(interval));
#if DEBUG
		LOG_DBG("Interval %u: %u cycles, %llu us", i, interval.cycles, k_ticks_to_us_floor64(interval.runtime));
#endif
		if (interval.cycles > 0)
		{
			uint64_t interval_runtime = interval.runtime / interval.cycles;
			runtime += interval_runtime;
			valid_intervals++;
		}
	}

	if (valid_intervals == 0)
		return 0; // no valid intervals

	runtime += runtime * (20 - valid_intervals) / valid_intervals; // extrapolate missing intervals
	LOG_DBG("Estimated runtime %llu us, %u%% coverage", k_ticks_to_us_floor64(runtime), valid_intervals * 100 / 20);

	valid_result |= 1;
	return runtime;
}

uint64_t sys_get_battery_runtime_min_estimate(void)
{
	static uint64_t runtime = 0;
	uint8_t valid_intervals = 0;

	if (valid_result & 2)
		return runtime;
	runtime = 0;

	for (uint8_t i = 0; i < 19; i++)
	{
		struct battery_tracker_interval interval = {0};
		sys_read(BATT_STATS_INTERVAL_0 + i, &interval, sizeof(interval));
#if DEBUG
		LOG_DBG("Interval %u min: %llu us", i, k_ticks_to_us_floor64(interval.runtime_min));
#endif
		if (interval.cycles > 0)
		{
			runtime += interval.runtime_min;
			valid_intervals++;
		}
	}

	if (valid_intervals == 0)
		return 0; // no valid intervals

	runtime += runtime * (20 - valid_intervals) / valid_intervals; // extrapolate missing intervals
	LOG_DBG("Estimated runtime min %llu us, %u%% coverage", k_ticks_to_us_floor64(runtime), valid_intervals * 100 / 20);

	valid_result |= 2;
	return runtime;
}

uint64_t sys_get_battery_runtime_max_estimate(void)
{
	static uint64_t runtime = 0;
	uint8_t valid_intervals = 0;

	if (valid_result & 4)
		return runtime;
	runtime = 0;

	for (uint8_t i = 0; i < 19; i++)
	{
		struct battery_tracker_interval interval = {0};
		sys_read(BATT_STATS_INTERVAL_0 + i, &interval, sizeof(interval));
#if DEBUG
		LOG_DBG("Interval %u max: %llu us", i, k_ticks_to_us_floor64(interval.runtime_max));
#endif
		if (interval.cycles > 0)
		{
			runtime += interval.runtime_max;
			valid_intervals++;
		}
	}

	if (valid_intervals == 0)
		return 0; // no valid intervals

	runtime += runtime * (20 - valid_intervals) / valid_intervals; // extrapolate missing intervals
	LOG_DBG("Estimated runtime max %llu us, %u%% coverage", k_ticks_to_us_floor64(runtime), valid_intervals * 100 / 20);

	valid_result |= 4;
	return runtime;
}

uint64_t sys_get_battery_remaining_time_estimate(void)
{
	static uint64_t result = 0;

	if (valid_result & 8)
		return result;

	if (last_unplugged_runtime <= CONFIG_SYS_CLOCK_TICKS_PER_SEC * 60) // pptt may not be valid yet
		return 0; // no valid pptt

	uint64_t runtime = sys_get_battery_runtime_estimate();
	if (runtime == 0)
		return 0; // no valid runtime

	int16_t pptt = sys_get_valid_battery_pptt();
	if (pptt < 0)
		return 0; // no valid pptt

	pptt = sys_get_calibrated_battery_pptt(pptt);
	if (!battery_pptt_is_valid(pptt))
		return 0;

	result = runtime * (uint64_t)pptt / 10000;

	valid_result |= 8;
	return result;
}

// LSB is 1/20 cycles
uint32_t sys_get_battery_cycles(void)
{
	static uint32_t cycles = 0;

	if (valid_result & 16)
		return cycles;
	cycles = 0;

	for (uint8_t i = 0; i < 19; i++)
	{
		struct battery_tracker_interval interval;
		sys_read(BATT_STATS_INTERVAL_0 + i, &interval, sizeof(interval));
#if DEBUG
		LOG_DBG("Interval %u: %u cycles, %llu us", i, interval.cycles, k_ticks_to_us_floor64(interval.runtime));
#endif
		cycles += interval.cycles;
	}

	valid_result |= 16;
	return cycles;
}

// LSB is 5% intervals
uint8_t sys_get_battery_calibration_coverage(void)
{
	static uint8_t valid_intervals = 0;

	if (valid_result & 32)
		return valid_intervals;
	valid_intervals = 0;

	for (uint8_t i = 0; i < 19; i++)
	{
		struct battery_tracker_interval interval;
		sys_read(BATT_STATS_INTERVAL_0 + i, &interval, sizeof(interval));
		if (interval.cycles > 0)
			valid_intervals++;
	}

	valid_result |= 32;
	return valid_intervals; // maximum coverage is 95%
}

int16_t sys_get_calibrated_battery_range_min_pptt(void)
{
	for (uint8_t i = 0; i < 18; i++)
	{
		if (retained->battery_pptt_curve[i] > 0)
			return i * 500;
	}
	return -1;
}

int16_t sys_get_calibrated_battery_range_max_pptt(void)
{
	for (int8_t i = 17; i >= 0; i--)
	{
		if (retained->battery_pptt_curve[i] > 0)
			return (i + 2) * 500;
	}
	return -1;
}

int16_t sys_get_last_cycle_min_pptt(void)
{
	struct battery_tracker tracker = {0};
	sys_read(BATT_STATS_LAST_RUN_ID, &tracker, sizeof(tracker));
	if (tracker.last_min_battery_pptt < 0)
		return -1;
	return tracker.last_min_battery_pptt;
}

int16_t sys_get_last_cycle_max_pptt(void)
{
	struct battery_tracker tracker = {0};
	sys_read(BATT_STATS_LAST_RUN_ID, &tracker, sizeof(tracker));
	if (tracker.last_max_battery_pptt < 0)
		return -1;
	return tracker.last_max_battery_pptt;
}

uint64_t sys_get_last_cycle_runtime(void)
{
	struct battery_tracker tracker = {0};
	sys_read(BATT_STATS_LAST_RUN_ID, &tracker, sizeof(tracker));
	if (tracker.last_battery_runtime < 0)
		return 0;
	return tracker.last_battery_runtime;
}

void sys_reset_battery_tracker(void)
{
	static bool reset_confirm = false;
	if (!reset_confirm)
	{
		printk("Resetting battery tracker will clear all battery calibration data. Are you sure?\n");
		reset_confirm = true;
		return;
	}
	printk("Resetting battery tracker\n");

	reset_tracker(-1);
	struct battery_tracker tracker = {0};
	sys_write(BATT_STATS_LAST_RUN_ID, NULL, &tracker, sizeof(tracker));
	for (uint8_t i = 0; i < 19; i++)
	{
		struct battery_tracker_interval interval = {0};
		sys_write(BATT_STATS_INTERVAL_0 + i, NULL, &interval, sizeof(interval));
	}
	int16_t* curve = (int16_t*)k_malloc(sizeof(int16_t) * 18);
	memset(curve, 0, sizeof(int16_t) * 18);
	sys_write(BATT_STATS_CURVE_ID, retained->battery_pptt_curve, curve, sizeof(int16_t) * 18); // updates retained
	k_free(curve);
	valid_result = 0; // invalidate all
	reset_confirm = false;
	LOG_INF("Battery tracker reset");
}

void sys_print_battery_tracker_debug(void)
{
	printk("\n=== Battery Tracker Debug ===\n");

	// Check tracker validity
	bool valid = retained->max_battery_pptt >= 0 && retained->max_battery_pptt <= 10000;
	printk("Tracker state: %s\n", valid ? "VALID" : "NOT INITIALIZED");

	printk("\nRetained data:\n");
	if (retained->max_battery_pptt < 0 || retained->max_battery_pptt > 10000)
		printk("  max_battery_pptt: INVALID (%d)\n", retained->max_battery_pptt);
	else
		printk("  max_battery_pptt: %.2f%%\n", (double)retained->max_battery_pptt / 100.0);

	if (retained->min_battery_pptt < 0 || retained->min_battery_pptt > 10000)
		printk("  min_battery_pptt: INVALID (%d)\n", retained->min_battery_pptt);
	else
		printk("  min_battery_pptt: %.2f%%\n", (double)retained->min_battery_pptt / 100.0);

	if (retained->battery_pptt_saved < 0 || retained->battery_pptt_saved > 10500)
		printk("  battery_pptt_saved: INVALID (%d)\n", retained->battery_pptt_saved);
	else
		printk("  battery_pptt_saved: %.2f%%\n", (double)retained->battery_pptt_saved / 100.0);

	printk("  battery_runtime_sum: %llu us\n", k_ticks_to_us_floor64(retained->battery_runtime_sum));
	printk("  battery_runtime_saved: %llu us\n", k_ticks_to_us_floor64(retained->battery_runtime_saved));

	printk("\nCurrent session:\n");
	printk("  last_unplugged_pptt: %.2f%%\n", (double)last_unplugged_pptt / 100.0);

	if (valid)
	{
		// Calculate current interval progress
		uint64_t current_interval_runtime = retained->battery_runtime_sum - retained->battery_runtime_saved;
		printk("  current_interval_runtime: %llu us (need 5 min active time min)\n",
			k_ticks_to_us_floor64(current_interval_runtime));

		// Show what's needed to record next interval
		int16_t next_interval_pptt = retained->battery_pptt_saved - 500;
		int16_t valid_below_pptt = retained->max_battery_pptt - 500;
		printk("  next_interval triggers at: %.2f%% (valid if <= %.2f%%)\n",
			(double)next_interval_pptt / 100.0,
			(double)valid_below_pptt / 100.0);
	}

	printk("\nInterval data from NVS:\n");
	uint8_t valid_count = 0;
	for (uint8_t i = 0; i < 20; i++)
	{
		struct battery_tracker_interval interval = {0};
		sys_read(BATT_STATS_INTERVAL_0 + i, &interval, sizeof(interval));
		if (interval.cycles > 0)
		{
			printk("  Interval %u (%.0f%%-%.0f%%): %u cycles, avg %llu us\n",
				i, (double)(i + 1) * 5.0, (double)i * 5.0,
				interval.cycles, k_ticks_to_us_floor64(interval.runtime / interval.cycles));
			valid_count++;
		}
	}
	if (valid_count == 0)
	{
		printk("  No interval data recorded yet\n");
	}

	printk("\nCalibration curve:\n");
	bool has_curve = false;
	for (uint8_t i = 0; i < 18; i++)
	{
		if (retained->battery_pptt_curve[i] > 0)
		{
			printk("  %.0f%% -> %.2f%%\n", (double)(i + 1) * 5.0, (double)retained->battery_pptt_curve[i] / 100.0);
			has_curve = true;
		}
	}
	if (!has_curve)
	{
		printk("  No calibration curve yet (need to charge after discharge)\n");
	}
	printk("==============================\n");
}
