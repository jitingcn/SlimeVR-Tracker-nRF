#include "globals.h"
#include "system/system.h"
#include "system/battery_tracker.h"
#include "system/test_mode.h"
#include "sensor/sensor.h"
#include "sensor/calibration.h"
#include "sensor/fusion/vqf/vqf.h"
#include "connection/esb.h"
#include "connection/tdma.h"
#include "build_defines.h"
#include "zephyr/sys/printk.h"

#if CONFIG_USB_DEVICE_STACK
#define USB DT_NODELABEL(usbd)
#define USB_EXISTS (DT_NODE_HAS_STATUS(USB, okay) && CONFIG_UART_CONSOLE)
#endif

#if (USB_EXISTS || CONFIG_RTT_CONSOLE) && CONFIG_USE_SLIMENRF_CONSOLE

#if USB_EXISTS
#include <zephyr/console/console.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/drivers/uart.h>
#else
#include "system/rtt_console.h"
#endif
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

static void console_thread(void);
#if USB_EXISTS
static struct k_thread console_thread_id;
static K_THREAD_STACK_DEFINE(console_thread_id_stack, 2048);
#else
K_THREAD_DEFINE(console_thread_id, 2048, console_thread, NULL, NULL, NULL, 6, 0, 0);
#endif

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2
#define NRF5_BOOTLOADER CONFIG_BOARD_HAS_NRF5_BOOTLOADER

#if NRF5_BOOTLOADER
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mag), okay)
#define SENSOR_MAG_EXISTS true
#endif

#if CONFIG_SENSOR_USE_SENS_CALIBRATION
#define SENS_CAL_DEFAULT_REVOLUTIONS CONFIG_SENSOR_SENS_REV
#define SENS_CAL_MAX_REVOLUTIONS     100
#endif

static const char *meows[] = {
	"Mew", "Meww", "Meow", "Meow meow", "Mrrrp", "Mrrf", "Mreow", "Mrrrow", "Mrrr", "Purr",
	"mew", "meww", "meow", "meow meow", "mrrrp", "mrrf", "mreow", "mrrrow", "mrrr", "purr",
};

static const char *meow_punctuations[] = {".", "?", "!", "-", "~", ""};

static const char *meow_suffixes[]
	= {" :3", " :3c", " ;3", " ;3c", " x3", " x3c", " X3", " X3c", " >:3", " >:3c", " >;3", " >;3c", ""};

void console_thread_create(void)
{
#if USB_EXISTS
	k_thread_create(
		&console_thread_id,
		console_thread_id_stack,
		K_THREAD_STACK_SIZEOF(console_thread_id_stack),
		(k_thread_entry_t)console_thread,
		NULL,
		NULL,
		NULL,
		6,
		0,
		K_NO_WAIT
	);
#endif
}

void console_thread_abort(void)
{
#if USB_EXISTS
	k_thread_abort(&console_thread_id);
#endif
}

static void print_board(void)
{
#if USB_EXISTS
	printk(CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT "\n");
#endif
	printk(FW_STRING);
	printk("Repo: %s | Branch: %s | Author: %s\n", FW_GIT_REPO_URL, FW_GIT_BRANCH, FW_GIT_AUTHOR);

	printk("\nBoard: " CONFIG_BOARD "\n");
	printk("SOC: " CONFIG_SOC "\n");
	printk("Target: " CONFIG_BOARD_TARGET "\n");
}

static void print_sensor(void)
{
	printk("IMU: %s\n", (retained->imu_addr & 0x7F) != 0x7F ? sensor_get_sensor_imu_name() : "Not searching");
	if (retained->imu_reg != 0xFF) {
		printk("Interface: %s\n", (retained->imu_reg & 0x80) ? "SPI" : "I2C");
	}
	printk("Address: 0x%02X%02X\n", retained->imu_addr, retained->imu_reg);

	printk(
		"\nMagnetometer: %s (%s)\n",
		(retained->mag_addr & 0x7F) != 0x7F ? sensor_get_sensor_mag_name() : "Not searching",
		sensor_get_mag_enabled() ? "enabled" : "disabled"
	);
	if (retained->mag_reg != 0xFF) {
		const char *mag_interface;
		if (retained->mag_addr & 0x80) {
			// External magnetometer (via IMU I2CM or passthrough)
			if (retained->imu_reg & 0x80) {
				mag_interface = "EXT (SPI IMU I2CM)";
			} else {
				mag_interface = "I2C (passthrough)";
			}
		} else {
			mag_interface = (retained->mag_reg & 0x80) ? "SPI" : "I2C";
		}
		printk("Interface: %s\n", mag_interface);
	}
	printk("Address: 0x%02X%02X\n", retained->mag_addr, retained->mag_reg);

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	printk("\nAccelerometer matrix:\n");
	for (int i = 0; i < 3; i++) {
		printk(
			"%.5f %.5f %.5f %.5f\n",
			(double)retained->accBAinv[0][i],
			(double)retained->accBAinv[1][i],
			(double)retained->accBAinv[2][i],
			(double)retained->accBAinv[3][i]
		);
	}

	// Print calibration quality analysis
	printk("\nAccel calibration:\n");
	printk(
		"  Offset: [%.5f, %.5f, %.5f]\n",
		(double)retained->accBAinv[0][0],
		(double)retained->accBAinv[0][1],
		(double)retained->accBAinv[0][2]
	);
	float diag_x = retained->accBAinv[1][0];
	float diag_y = retained->accBAinv[2][1];
	float diag_z = retained->accBAinv[3][2];
	printk("  Scale: [%.5f, %.5f, %.5f]\n", (double)diag_x, (double)diag_y, (double)diag_z);

#else
	printk(
		"\nAccelerometer bias: %.5f %.5f %.5f\n",
		(double)retained->accelBias[0],
		(double)retained->accelBias[1],
		(double)retained->accelBias[2]
	);
#endif
	printk(
		"Gyroscope bias: %.5f %.5f %.5f\n",
		(double)retained->gyroBias[0],
		(double)retained->gyroBias[1],
		(double)retained->gyroBias[2]
	);
#if CONFIG_SENSOR_USE_TCAL
	// Display the real-time calculated gyro offset
	float current_gyro_offset[3];
	sensor_calibration_get_last_gyro_offset(current_gyro_offset);
	printk(
		"Gyroscope bias tcal (real-time): %.5f %.5f %.5f at %.2f C\n",
		(double)current_gyro_offset[0],
		(double)current_gyro_offset[1],
		(double)current_gyro_offset[2],
		(double)sensor_get_current_imu_temperature()
	);
#endif
	//	printk("Magnetometer bridge offset: %.5f %.5f %.5f\n", (double)retained->magBias[0],
	//(double)retained->magBias[1], (double)retained->magBias[2]);
	printk("Magnetometer matrix:\n");
	for (int i = 0; i < 3; i++) {
		printk(
			"%.5f %.5f %.5f %.5f\n",
			(double)retained->magBAinv[0][i],
			(double)retained->magBAinv[1][i],
			(double)retained->magBAinv[2][i],
			(double)retained->magBAinv[3][i]
		);
	}
	{
		bool mag_has_cal = (retained->magBAinv[0][0] != 0.0f
		                 || retained->magBAinv[0][1] != 0.0f
		                 || retained->magBAinv[0][2] != 0.0f);
		float dir_bias = 0;
		int online_samples = sensor_calibration_online_mag_status(&dir_bias);
		float mag_cv = sensor_calibration_get_mag_quality();
		printk("Mag cal: %s | norm_cv=%.3f | Online: %d samples, dir_bias=%.2f\n",
		       mag_has_cal ? "active" : "none",
		       (double)mag_cv, online_samples, (double)dir_bias);
	}

	printk("\nFusion: %s\n", sensor_get_sensor_fusion_name());

#if CONFIG_SENSOR_RANGE_STATS
	// Display runtime range statistics summary
	const sensor_range_stats_t *stats = sensor_get_range_stats();
	if (stats->initialized) {
		float gyro_peak = 0;
		float accel_peak = 0;
		for (int i = 0; i < 3; i++) {
			float g_peak = fmaxf(fabsf(stats->gyro_min[i]), fabsf(stats->gyro_max[i]));
			float a_peak = fmaxf(fabsf(stats->accel_min[i]), fabsf(stats->accel_max[i]));
			if (g_peak > gyro_peak) gyro_peak = g_peak;
			if (a_peak > accel_peak) accel_peak = a_peak;
		}
		printk("\nRuntime range peaks (this session):\n");
		printk("  Gyro: %.2f deg/s\n", (double)gyro_peak);
		printk("  Accel: %.3f g\n", (double)accel_peak);
		printk("  Samples: %llu (use 'range' for details)\n", stats->sample_count);
	}
#endif // CONFIG_SENSOR_RANGE_STATS
}

static void print_sens_calibration_info(void)
{
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
	// Display Gyro sensitivity
	if (retained) {
		float scale_x = retained->gyroSensScale[0];
		float scale_y = retained->gyroSensScale[1];
		float scale_z = retained->gyroSensScale[2];

		// Calculate the approximate input degrees difference based on the stored scale factor
		// degrees = (1.0 - (1.0 / scale)) * 360.0 * number of revolutions
		float deg_x = (1.0f - (1.0f / scale_x)) * (360.0f * CONFIG_SENSOR_SENS_REV);
		float deg_y = (1.0f - (1.0f / scale_y)) * (360.0f * CONFIG_SENSOR_SENS_REV);
		float deg_z = (1.0f - (1.0f / scale_z)) * (360.0f * CONFIG_SENSOR_SENS_REV);

		printk(
			"Gyroscope sensitivity (degrees diff over %u rev): %.3f %.3f %.3f\n",
			(int)CONFIG_SENSOR_SENS_REV,
			(double)deg_x,
			(double)deg_y,
			(double)deg_z
		);
		printk(
			"Gyroscope sensitivity scale: %.5f %.5f %.5f\n",
			(double)scale_x,
			(double)scale_y,
			(double)scale_z
		);
	} else {
		printk("Gyroscope sensitivity: Retained data unavailable.\n");
	}
#endif
}

static void print_connection(void)
{
	bool paired = retained->paired_addr[0];
	printk(paired ? "Tracker ID: %u\n" : "\nTracker ID: None\n", retained->paired_addr[1]);
	printk("Device address: %012llX\n", *(uint64_t *)NRF_FICR->DEVICEADDR & 0xFFFFFFFFFFFF);
	printk("Receiver address: %012llX\n", (*(uint64_t *)&retained->paired_addr[0] >> 16) & 0xFFFFFFFFFFFF);
	print_sens_calibration_info();
	printk(
		paired ? "Receiver address: %012llX\n" : "Receiver address: None\n",
		(*(uint64_t *)&retained->paired_addr[0] >> 16) & 0xFFFFFFFFFFFF
	);

	// Display RF channel info
	if (retained->rf_channel != 0xFF && retained->rf_channel <= 100) {
		printk("RF Channel: %u (custom)\n", retained->rf_channel);
	} else {
		printk("RF Channel: %u (default)\n", CONFIG_RADIO_RF_CHANNEL);
	}
}

static void print_battery(void)
{
	int battery_mV = sys_get_valid_battery_mV();
	int16_t calibrated_pptt = sys_get_calibrated_battery_pptt(sys_get_valid_battery_pptt());
	uint64_t unplugged_time = sys_get_last_unplugged_time();
	uint64_t remaining = sys_get_battery_remaining_time_estimate();
	uint64_t runtime = sys_get_battery_runtime_estimate();
	if (battery_mV > 0) {
		unplugged_time = k_ticks_to_us_floor64(k_uptime_ticks() - unplugged_time);
		uint32_t hours = unplugged_time / 3600000000;
		unplugged_time %= 3600000000;
		uint8_t minutes = unplugged_time / 60000000;
		if (hours > 0 || minutes > 0) {
			printk("Battery: %.0f%% (Read %uh %umin ago)\n", (double)calibrated_pptt / 100.0, hours, minutes);
		} else {
			printk("Battery: %.0f%%\n", (double)calibrated_pptt / 100.0);
		}
	} else if (unplugged_time == 0) {
		printk("Battery: Waiting for valid reading\n");
	} else {
		printk("Battery: None\n");
	}
	if (remaining > 0) {
		remaining = k_ticks_to_us_floor64(remaining);
		uint32_t hours = remaining / 3600000000;
		remaining %= 3600000000;
		uint8_t minutes = remaining / 60000000;
		printk("Remaining runtime: %uh %umin\n", hours, minutes);
	} else {
		printk("Remaining runtime: Not available\n");
	}
	if (runtime > 0) {
		runtime = k_ticks_to_us_floor64(runtime);
		uint32_t hours = runtime / 3600000000;
		runtime %= 3600000000;
		uint8_t minutes = runtime / 60000000;
		printk("Fully charged runtime: %uh %umin\n", hours, minutes);
	} else {
		printk("Fully charged runtime: Not available\n");
	}
}

static void print_info(void)
{
	print_board();
	printk("\n");
	print_sensor();
	printk("\n");
	print_connection();
	printk("\n");
	print_battery();
}

static void print_uptime(const uint64_t ticks, const char *name)
{
	uint64_t uptime = k_ticks_to_us_floor64(ticks);

	uint32_t hours = uptime / 3600000000;
	uptime %= 3600000000;
	uint8_t minutes = uptime / 60000000;
	uptime %= 60000000;
	uint8_t seconds = uptime / 1000000;
	uptime %= 1000000;
	uint16_t milliseconds = uptime / 1000;
	uint16_t microseconds = uptime % 1000;

	printk("%s: %02u:%02u:%02u.%03u,%03u\n", name, hours, minutes, seconds, milliseconds, microseconds);
}

static void print_battery_tracker(void)
{
	int adc_mV = sys_get_battery_mV();
	printk("ADC: %d mV\n", adc_mV);

	int battery_mV = sys_get_valid_battery_mV();
	int16_t pptt = sys_get_valid_battery_pptt();
	int16_t calibrated_pptt = sys_get_calibrated_battery_pptt(pptt);
	uint64_t unplugged_time = sys_get_last_unplugged_time();
	if (battery_mV > 0) {
		printk(
			"\nBattery: %.2f%% (Raw %.2f%%, %d mV)\n",
			(double)calibrated_pptt / 100.0,
			(double)pptt / 100.0,
			battery_mV
		);
	} else {
		printk("\nBattery: None\n");
	}
	if (unplugged_time > 0) {
		print_uptime(k_uptime_ticks() - unplugged_time, "Last updated");
	} else {
		printk("Last updated: Never\n");
	}

	uint64_t runtime = sys_get_battery_runtime_estimate();
	uint64_t runtime_min = sys_get_battery_runtime_min_estimate();
	uint64_t runtime_max = sys_get_battery_runtime_max_estimate();
	uint64_t remaining = sys_get_battery_remaining_time_estimate();
	if (remaining > 0) {
		print_uptime(remaining, "\nRemaining runtime");
	} else {
		printk("Remaining runtime: Not available\n");
	}
	if (runtime > 0) {
		print_uptime(runtime, "Fully charged runtime");
	} else {
		printk("Fully charged runtime: Not available\n");
	}
	if (runtime_min > 0) {
		print_uptime(runtime_min, "Minimum runtime");
	} else {
		printk("Minimum runtime: Not available\n");
	}
	if (runtime_max > 0) {
		print_uptime(runtime_max, "Maximum runtime");
	} else {
		printk("Maximum runtime: Not available\n");
	}

	int16_t last_min = sys_get_last_cycle_min_pptt();
	int16_t last_max = sys_get_last_cycle_max_pptt();
	int16_t last_calibrated_min = sys_get_calibrated_battery_pptt(last_min);
	int16_t last_calibrated_max = sys_get_calibrated_battery_pptt(last_max);
	uint64_t last_runtime = sys_get_last_cycle_runtime();
	if (last_min >= 0 && last_max >= 0 && last_runtime > 0) {
		printk(
			"\nLast discharge cycle: %.2f%% -> %.2f%% (Raw %.2f%% -> %.2f%%)\n",
			(double)last_calibrated_max / 100.0,
			(double)last_calibrated_min / 100.0,
			(double)last_max / 100.0,
			(double)last_min / 100.0
		);
		print_uptime(last_runtime, "Last cycle runtime");
	} else {
		printk("\nLast cycle: Not available\n");
	}

	uint8_t coverage = sys_get_battery_calibration_coverage() * 5;
	int16_t min = sys_get_calibrated_battery_range_min_pptt();
	int16_t max = sys_get_calibrated_battery_range_max_pptt();
	if (min >= 0 && max >= 0) {
		printk(
			"\nCalibration: %.0f%% - %.0f%% (%.0f%% coverage)\n",
			(double)min / 100.0,
			(double)max / 100.0,
			(double)coverage
		);
	} else {
		printk("\nCalibration: None\n");
	}
	printk("Cycle count: ~%.2f\n", (double)sys_get_battery_cycles() / 20.0);

	// Print debug information
	sys_print_battery_tracker_debug();
}

static void print_meow(void)
{
	int64_t ticks = k_uptime_ticks();

	ticks %= ARRAY_SIZE(meows) * ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes); // silly number generator
	uint8_t meow = ticks / (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	ticks %= (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	uint8_t punctuation = ticks / ARRAY_SIZE(meow_suffixes);
	uint8_t suffix = ticks % ARRAY_SIZE(meow_suffixes);

	printk("%s%s%s\n", meows[meow], meow_punctuations[punctuation], meow_suffixes[suffix]);
}

static void print_help(void)
{
	printk("\n=== Available Commands ===\n\n");
	printk("Device Information:\n");
	printk("  info                       Get device information\n");
	printk("  uptime                     Get device uptime\n");
	printk("  battery                    Get battery information\n");
	printk("\n");
	printk("Sensor Management:\n");
	printk("  scan                       Restart sensor scan\n");
	printk("  calibrate                  Calibrate sensor ZRO\n");
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	printk("  6-side                     Calibrate 6-side accelerometer\n");
#endif
	printk("  mag                        Show magnetometer status\n");
	printk("  mag on|off                 Enable/disable magnetometer\n");
	printk("  mag clear                  Clear magnetometer calibration\n");
	printk("  mag cal                    Start magnetometer calibration\n");
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
	printk("  sens <x>,<y>,<z>           Set gyro sensitivity (deg diff over %u rev)\n", (int)CONFIG_SENSOR_SENS_REV);
	printk("  sens auto <x|y|z> [rev]    Auto-calibrate gyro sensitivity by spinning (default %u rev)\n", SENS_CAL_DEFAULT_REVOLUTIONS);
	printk("  sens reset                 Reset gyro sensitivity calibration\n");
#endif
#if CONFIG_SENSOR_USE_TCAL
	// Update the help string to show the new command set
	printk("  tcal <on|off|status|dump|test temp|remove index|auto on|auto off> Temperature calibration\n");
#endif
	printk("\n");
	printk("Connection:\n");
	printk("  set <address>              Manually set receiver\n");
	printk("  pair                       Enter pairing mode\n");
	printk("  clear                      Clear pairing data\n");
	printk("  tdma <on|off>              Enable/disable TDMA scheduling\n");
	printk("\n");
	printk("RF Channel:\n");
	printk("  channel <1-100>            Set RF channel (saved to NVS)\n");
	printk("    Example: channel 25       Set RF channel to 25\n");
	printk("  clearchannel               Clear RF channel (use default)\n");
	printk("\n");
	printk("System:\n");
	printk("  shutdown                   Power off the device\n");
	printk("  reboot                     Soft reset the device\n");
#if DFU_EXISTS
	printk("  dfu                        Enter DFU bootloader\n");
#if ADAFRUIT_BOOTLOADER
	printk("  dfu ota                    Enter OTA DFU (BLE)\n");
#endif
#endif
	printk("\n");
	printk("Other:\n");
	printk("  ping                       Flash LED (same as remote PING command)\n");
	printk("  meow                       Meow!\n");
	printk("  nvs                        Show NVS usage statistics\n");
	printk("  help                       Show this help message\n");
	printk("  debug [duration]           Start sensor debug mode at FIFO rate (1-60s, default 1s)\n");
	printk("  range                      Show sensor range statistics (min/max values)\n");
	printk("  range reset                Reset sensor range statistics\n");
#if CONFIG_VQF_BENCH
	printk("  vqfbench [iterations]      Benchmark VQF update paths (default 1000)\n");
#endif
	printk("\n");
	printk("Debug Commands:\n");
	printk("  reset zro                  Reset ZRO calibration\n");
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	printk("  reset acc                  Reset accelerometer calibration\n");
#endif
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
	printk("  reset sens                 Reset gyro sensitivity calibration\n");
#endif
#if CONFIG_SENSOR_USE_TCAL
	printk("  reset tcal                 Reset temperature calibration\n");
#endif
	printk("  reset mag                  Reset magnetometer calibration\n");
	printk("  reset bat                  Reset battery tracker\n");
	printk("  reset all                  Clear all settings\n");
	printk("\n");
}

// --- Command Implementations ---

void cmd_sens_set(float x, float y, float z)
{
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
	if (retained) {
		float deg_x = x;
		float deg_y = y;
		float deg_z = z;

		float den_x = 1.0f - (deg_x / (360.0f * CONFIG_SENSOR_SENS_REV));
		float den_y = 1.0f - (deg_y / (360.0f * CONFIG_SENSOR_SENS_REV));
		float den_z = 1.0f - (deg_z / (360.0f * CONFIG_SENSOR_SENS_REV));

		// Prevent division by zero or near-zero
		if (fabsf(den_x) < 1e-6f || fabsf(den_y) < 1e-6f || fabsf(den_z) < 1e-6f) {
			printk("Error: Invalid input degrees leading to division by zero. Calibration not applied.\n");
		} else {
			retained->gyroSensScale[0] = 1.0f / den_x;
			retained->gyroSensScale[1] = 1.0f / den_y;
			retained->gyroSensScale[2] = 1.0f / den_z;
			retained_update();
			sys_write(
				MAIN_GYRO_SENS_ID,
				&retained->gyroSensScale,
				retained->gyroSensScale,
				sizeof(retained->gyroSensScale)
			);
			printk(
				"Gyro sensitivity difference set to: %.3f, %.3f, %.3f\n",
				(double)deg_x,
				(double)deg_y,
				(double)deg_z
			);
		}
	} else {
		printk("Error: Retained data not available.\n");
	}
#else
	printk("Error: Sensitivity calibration not enabled.\n");
#endif
}

void cmd_sens_reset(void)
{
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
	if (retained) {
		printk("Resetting gyro sensitivity calibration.\n");
		retained->gyroSensScale[0] = 1.0f;
		retained->gyroSensScale[1] = 1.0f;
		retained->gyroSensScale[2] = 1.0f;
		retained_update(); // Save changes
		sys_write(
			MAIN_GYRO_SENS_ID,
			&retained->gyroSensScale,
			retained->gyroSensScale,
			sizeof(retained->gyroSensScale)
		);
		printk("Gyro sensitivity reset.\n");
	} else {
		printk("Error: Retained data not available.\n");
	}
#else
	printk("Error: Sensitivity calibration not enabled.\n");
#endif
}

void cmd_sens_auto_request(uint8_t axis, uint16_t revolutions)
{
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
	if (axis >= 3) {
		printk("Error: Invalid sensitivity calibration axis %u.\n", axis);
		return;
	}

	if (revolutions == 0) {
		revolutions = SENS_CAL_DEFAULT_REVOLUTIONS;
	}

	if (revolutions > SENS_CAL_MAX_REVOLUTIONS) {
		printk("Error: Invalid revolutions %u. Use 1 to %u.\n", revolutions, SENS_CAL_MAX_REVOLUTIONS);
		return;
	}

	char axis_char = "XYZ"[axis];
	if (sensor_request_calibration_sens(axis, revolutions) != 0) {
		printk("Error: Calibration busy or parameters invalid.\n");
		return;
	}

	printk("Gyro sensitivity auto-calibration started on %c axis (%u rev).\n", axis_char, revolutions);
	printk("  1. Hold the tracker still until the LED flashes.\n");
	printk("  2. While flashing, spin it %u full turns about the %c axis, then stop.\n", revolutions, axis_char);
#else
	printk("Error: Sensitivity calibration not enabled.\n");
#endif
}

void cmd_sens_auto(const char *axis_str, const char *rev_str)
{
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
	// Axis is a single character; the command parser has already lowercased it.
	if (axis_str == NULL || axis_str[0] == '\0' || axis_str[1] != '\0') {
		printk("Error: Specify a single axis. Use: 'sens auto <x|y|z> [revolutions]'.\n");
		return;
	}

	uint8_t axis;
	switch (axis_str[0]) {
	case 'x':
		axis = 0;
		break;
	case 'y':
		axis = 1;
		break;
	case 'z':
		axis = 2;
		break;
	default:
		printk("Error: Invalid axis '%s'. Use x, y, or z.\n", axis_str);
		return;
	}

	uint16_t revolutions = SENS_CAL_DEFAULT_REVOLUTIONS;
	if (rev_str != NULL) {
		char *endptr;
		long value = strtol(rev_str, &endptr, 10);
		if (*endptr != '\0' || value < 1 || value > SENS_CAL_MAX_REVOLUTIONS) {
			printk("Error: Invalid revolutions '%s'. Use 1 to %u.\n", rev_str, SENS_CAL_MAX_REVOLUTIONS);
			return;
		}
		revolutions = (uint16_t)value;
	}

	cmd_sens_auto_request(axis, revolutions);
#else
	printk("Error: Sensitivity calibration not enabled.\n");
#endif
}

void cmd_reset_zro(void)
{
	sensor_calibration_clear(NULL, NULL, true);
	// Manual command: invalidate fusion to force quaternion recalculation
	sensor_fusion_invalidate();
}

void cmd_reset_acc(void)
{
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	sensor_calibration_clear_6_side(NULL, true);
#else
	printk("Error: 6-side calibration not enabled.\n");
#endif
}

void cmd_reset_tcal(void)
{
#if CONFIG_SENSOR_USE_TCAL
	sensor_tcal_clear();
#else
	printk("Error: Temperature calibration not enabled.\n");
#endif
}

void cmd_reset_bat(void)
{
	sys_reset_battery_tracker();
}

void cmd_fusion_reset(void)
{
	printk("Resetting fusion (invalidating quaternion).\n");
	sensor_fusion_invalidate();
	printk("Fusion reset complete.\n");
}

void cmd_bat_debug(void)
{
	sys_print_battery_tracker_debug();
}

void cmd_ping_start(void)
{
	printk("Ping received! Flashing LED.\n");
	set_led(SYS_LED_PATTERN_ONESHOT_PING, SYS_LED_PRIORITY_HIGHEST);
}

void cmd_shutdown(void)
{
	printk("Shutting down device.\n");
	sys_command_shutdown();
}

static void console_thread(void)
{
#if USB_EXISTS && DFU_EXISTS
	if (button_read()) // button held on usb connect, enter DFU
	{
#if ADAFRUIT_BOOTLOADER
		NRF_POWER->GPREGRET = ADAFRUIT_DFU_MAGIC_UF2_RESET;
		sys_request_system_reboot(false);
#endif
#if NRF5_BOOTLOADER
		gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
	}
#endif

#if USB_EXISTS
	console_getline_init();

	// Wait for any pending log data to be processed
	while (log_data_pending()) {
		k_usleep(1);
	}

	// Wait for USB CDC to be ready by checking DTR (Data Terminal Ready) signal
	// This ensures the terminal is actually connected and ready to receive data
	const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	if (device_is_ready(uart_dev)) {
		uint32_t dtr = 0;
		// Wait up to 5 seconds for DTR to be asserted (terminal connected)
		for (int i = 0; i < 50; i++) {
			if (uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr) == 0 && dtr) {
				break;
			}
			k_msleep(100);
		}
		// Give a bit more time for the terminal to be fully ready
		k_msleep(100);
	}

	printk("*** " CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT " ***\n");
#endif
	printk(FW_STRING);
	printk("Repo: %s | Branch: %s | Author: %s\n", FW_GIT_REPO_URL, FW_GIT_BRANCH, FW_GIT_AUTHOR);

	// Print help on startup
	print_help();

	uint8_t command_info[] = "info";
	uint8_t command_uptime[] = "uptime";
	uint8_t command_shutdown[] = "shutdown";
	uint8_t command_reboot[] = "reboot";
	uint8_t command_battery[] = "battery";
	uint8_t command_scan[] = "scan";
	uint8_t command_calibrate[] = "calibrate";
	uint8_t command_help[] = "help";
	uint8_t command_debug[] = "debug";
	uint8_t command_range[] = "range";
#if CONFIG_VQF_BENCH
	uint8_t command_vqfbench[] = "vqfbench";
#endif

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	uint8_t command_6_side[] = "6-side";
#endif

	uint8_t command_mag[] = "mag";

	uint8_t command_set[] = "set";
	uint8_t command_pair[] = "pair";
	uint8_t command_clear[] = "clear";
	uint8_t command_channel[] = "channel";
	uint8_t command_clearchannel[] = "clearchannel";

#if DFU_EXISTS
	uint8_t command_dfu[] = "dfu";
#endif

	uint8_t command_ping[] = "ping";
	uint8_t command_meow[] = "meow";
	uint8_t command_nvs[] = "nvs";

#if CONFIG_SENSOR_USE_SENS_CALIBRATION
	uint8_t command_sens[] = "sens";
#endif
#if CONFIG_SENSOR_USE_TCAL
	uint8_t command_tcal[] = "tcal";
#endif
	// debug
	uint8_t command_reset[] = "reset";
	uint8_t command_reset_arg_zro[] = "zro";
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	uint8_t command_reset_arg_acc[] = "acc";
#endif
	uint8_t command_reset_arg_mag[] = "mag";
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
	uint8_t command_reset_arg_sens[] = "sens";
#endif
#if CONFIG_SENSOR_USE_TCAL
	uint8_t command_reset_arg_tcal[] = "tcal";
#endif
	uint8_t command_reset_arg_bat[] = "bat";
	uint8_t command_reset_arg_fusion[] = "fusion";
	uint8_t command_reset_arg_all[] = "all";

	while (1) {
#if USB_EXISTS
		uint8_t *line = console_getline();
#else
		uint8_t *line = rtt_console_getline();
#endif
		uint8_t *arg = NULL;
		for (uint8_t *p = line; *p; ++p) {
			*p = tolower(*p);
			if (*p == ' ' && !arg) {
				*p = 0;
				p++;
				*p = tolower(*p);
				if (*p) {
					arg = p;
				}
			}
		}

		if (memcmp(line, command_help, sizeof(command_help)) == 0) {
			print_help();
		} else if (memcmp(line, command_info, sizeof(command_info)) == 0) {
			print_info();
		} else if (memcmp(line, command_uptime, sizeof(command_uptime)) == 0) {
			uint64_t uptime = k_uptime_ticks();
			print_uptime(uptime, "Uptime");
			print_uptime(uptime - retained->uptime_latest + retained->uptime_sum, "Accumulated");
		} else if (memcmp(line, command_shutdown, sizeof(command_shutdown)) == 0) {
			cmd_shutdown();
		} else if (memcmp(line, command_reboot, sizeof(command_reboot)) == 0) {
			sys_request_system_reboot(false);
		} else if (memcmp(line, command_battery, sizeof(command_battery)) == 0) {
			print_battery_tracker();
		} else if (memcmp(line, command_scan, sizeof(command_scan)) == 0) {
			sensor_request_scan(true);
		} else if (memcmp(line, command_calibrate, sizeof(command_calibrate)) == 0) {
			sensor_request_calibration();
		}
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
		else if (memcmp(line, command_sens, sizeof(command_sens)) == 0) {
			// check if there are any arguments at all.
			if (arg == NULL) {
				printk("Error: Missing arguments. Use 'sens <x>,<y>,<z>', 'sens auto <x|y|z> [rev]', or 'sens reset'.\n");
			}
			// check if this is the auto-calibration subcommand
			else if (strncmp((char *)arg, "auto", 4) == 0 && (arg[4] == '\0' || arg[4] == ' ')) {
				strtok((char *)arg, " "); // consume "auto"
				char *axis_str = strtok(NULL, " ");
				char *rev_str = strtok(NULL, " ");
				char *extra_str = strtok(NULL, " ");
				if (extra_str != NULL) {
					printk("Error: Too many arguments. Use: 'sens auto <x|y|z> [revolutions]'.\n");
				} else {
					cmd_sens_auto(axis_str, rev_str);
				}
			}
			// check if the argument is "reset"
			else if (strcmp((char *)arg, "reset") == 0) {
				cmd_sens_reset();
			} else {
				char *token;
				char *endptr;
				int token_count = 0;
				float values[3];

				token = strtok((char *)arg, ",");
				while (token != NULL && token_count < 3) {
					values[token_count] = strtof(token, &endptr);
					if (token == endptr || *endptr != '\0') {
						break; // Invalid float, stop parsing
					}
					token_count++;
					token = strtok(NULL, ",");
				}

				if (token_count == 3 && token == NULL) {
					cmd_sens_set(values[0], values[1], values[2]);
				} else {
					printk("Error: Invalid format. Use: 'sens <x>,<y>,<z>', 'sens auto <x|y|z> [rev]', or 'sens reset'.\n");
					printk("Example: sens 10.5,-2.1,15.0\n");
				}
			}
		}
#endif
#if CONFIG_SENSOR_USE_TCAL
		else if (memcmp(line, command_tcal, sizeof(command_tcal)) == 0) {
			// check if there are any arguments
			if (arg == NULL) {
				printk("Error: Missing argument. Use: tcal <on|off|status|clear|dump|test temp|remove index|check|auto on|auto off|boot [on|off]>\n");
			} else {
				// Tokenize the argument string by space to get the subcommand
				char *subcmd = strtok((char *)arg, " ");

				if (subcmd == NULL) {
					// Handling case where arg might contain only spaces
					printk("Error: Missing argument. Use: tcal <on|off|status|clear|dump|test temp|remove index|check|auto on|auto off|boot [on|off]>\n");
				} else if (strcmp(subcmd, "on") == 0) {
					sensor_tcal_set_enabled(true);
					printk("T-Cal compensation enabled\n");
				} else if (strcmp(subcmd, "off") == 0) {
					sensor_tcal_set_enabled(false);
					printk("T-Cal compensation disabled (using static gyro bias)\n");
				} else if (strcmp(subcmd, "status") == 0) {
					sensor_tcal_status();
					printk("T-Cal compensation: %s\n", sensor_tcal_get_enabled() ? "enabled" : "disabled");
					printk("Auto-calibration: %s\n", sensor_tcal_get_auto_calibration() ? "enabled" : "disabled");
				} else if (strcmp(subcmd, "clear") == 0) {
					cmd_reset_tcal();
				} else if (strcmp(subcmd, "auto") == 0) {
					char *auto_arg = strtok(NULL, " ");
					if (auto_arg == NULL) {
						printk("Error: Missing argument. Use: tcal auto <on|off>\n");
					} else if (strcmp(auto_arg, "on") == 0) {
						sensor_tcal_set_auto_calibration(true);
						printk("T-Cal auto-calibration enabled. Device will auto-calibrate when resting.\n");
						printk("Note: Sleep timeout will be prevented during auto-calibration mode.\n");
					} else if (strcmp(auto_arg, "off") == 0) {
						sensor_tcal_set_auto_calibration(false);
						printk("T-Cal auto-calibration disabled.\n");
					} else {
						printk("Error: Invalid argument '%s'. Use: tcal auto <on|off>\n", auto_arg);
					}
				} else if (strcmp(subcmd, "dump") == 0) {
					if (retained->tempCalState.count == 0) {
						printk("No temperature calibration points have been collected.\n");
						continue;
					}

					printk("Dumping %u collected temperature calibration points:\n", retained->tempCalState.count);
					printk("--------------------------------------------------\n");
					printk("Index | Temp (C) | Bias X   | Bias Y   | Bias Z   |\n");
					printk("--------------------------------------------------\n");

					uint16_t points_printed = 0;
					// Iterate through the entire buffer to find the valid points
					for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
						// A point is valid if its temperature field is not 0.0
						if (retained->tempCalPoints[i].temp != 0.0f) {
							printk(
								" %-4d | %-8.2f | %-8.5f | %-8.5f | %-8.5f\n",
								i,
								(double)retained->tempCalPoints[i].temp,
								(double)retained->tempCalPoints[i].bias[0],
								(double)retained->tempCalPoints[i].bias[1],
								(double)retained->tempCalPoints[i].bias[2]
							);
							points_printed++;
						}

						// Small delay to prevent overwhelming the console output buffer
						if (points_printed % 10 == 0 && points_printed > 0) {
							k_msleep(20);
						}
					}
					printk("--------------------------------------------------\n");
					printk("End of dump. Total points printed: %u\n", points_printed);
				} else if (strcmp(subcmd, "remove") == 0) {
					// Get the next token (the index number)
					char *idx_str = strtok(NULL, " ");

					if (idx_str == NULL) {
						printk("Error: Missing index. Use: tcal remove <index>\n");
					} else {
						char *endptr = NULL;
						long index = strtol(idx_str, &endptr, 10);

						// Check if conversion was successful
						if (endptr == NULL || endptr == idx_str) {
							printk("Error: Invalid index '%s'. Please provide a number.\n", idx_str);
						} else {
							// Skip trailing whitespace
							while (*endptr != '\0' && isspace((unsigned char)*endptr)) {
								endptr++;
							}

							// Check for trailing non-whitespace characters
							if (*endptr != '\0') {
								printk("Error: Invalid characters after index '%s'.\n", idx_str);
							} else {
								sensor_tcal_remove_point((int)index);
							}
						}
					}
				} else if (strcmp(subcmd, "test") == 0) {
					char *temp_str = strtok(NULL, " ");
					if (temp_str == NULL) {
						// Use current temperature if no argument provided
						float current_temp = sensor_get_current_imu_temperature();
						if (isnan(current_temp)) {
							printk("Error: Cannot read current temperature. Please specify temperature: tcal test <temp>\n");
						} else {
							sensor_tcal_test_methods(current_temp);
						}
					} else {
						char *endptr = NULL;
						float test_temp = strtof(temp_str, &endptr);

						if (endptr == temp_str || *endptr != '\0') {
							printk("Error: Invalid temperature '%s'. Use: tcal test <temp>\n", temp_str);
							printk("Example: tcal test 25.5\n");
						} else {
							sensor_tcal_test_methods(test_temp);
						}
					}
				} else if (strcmp(subcmd, "check") == 0) {
					float current_temp = sensor_get_current_imu_temperature();
					if (isnan(current_temp)) {
						printk("Error: Cannot read current temperature.\n");
					} else {
						float closest_temp, distance;
						bool needs_cal = sensor_tcal_is_temp_outside_range(current_temp, &closest_temp, &distance);
						printk("Current temperature: %.2fC\n", (double)current_temp);
						if (!isnan(closest_temp)) {
							printk("Closest calibration point: %.2fC (distance: %.2fC)\n",
								(double)closest_temp, (double)distance);
							float sampling_interval = 1.0f / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE;
							printk("Configured sampling interval: %.2fC\n", (double)sampling_interval);
							if (needs_cal) {
								printk("Status: NEEDS calibration (distance > interval, auto-cal may trigger)\n");
							} else {
								printk("Status: Calibration sufficient (within sampling interval)\n");
							}
						} else {
							printk("Status: No calibration data available (auto-cal will trigger)\n");
						}
					}
				} else if (strcmp(subcmd, "boot") == 0) {
					char *boot_arg = strtok(NULL, " ");
					if (boot_arg == NULL) {
						// Show current boot calibration status
						printk("Boot Calibration Status:\n");
						printk("  Enabled: %s\n", retained->bootCalState.enabled ? "yes" : "no");
						printk("  Completed: %s\n", retained->bootCalState.completed ? "yes" : "no");
						printk("  Attempts: %u\n", retained->bootCalState.attempt_count);
						printk("  D_offset valid: %s\n", retained->bootCalState.doffset_valid ? "yes" : "no");
						if (retained->bootCalState.doffset_valid) {
							printk("  D_offset: [%.5f, %.5f, %.5f] dps\n",
								(double)retained->bootCalState.doffset[0],
								(double)retained->bootCalState.doffset[1],
								(double)retained->bootCalState.doffset[2]);
						}
						printk("\nUsage: tcal boot <on|off>\n");
					} else if (strcmp(boot_arg, "on") == 0) {
						sensor_boot_cal_set_enabled(true);
						printk("Boot calibration enabled. Will calibrate on next boot.\n");
					} else if (strcmp(boot_arg, "off") == 0) {
						sensor_boot_cal_set_enabled(false);
						printk("Boot calibration disabled.\n");
					} else {
						printk("Error: Invalid argument '%s'. Use: tcal boot <on|off>\n", boot_arg);
					}
				} else {
					printk("Error: Invalid argument '%s'. Use: <status|clear|dump|test temp|remove index|check|auto on|auto off|boot on|boot off>\n", subcmd);
				}
			}
		}
#endif
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		else if (memcmp(line, command_6_side, sizeof(command_6_side)) == 0) {
			sensor_request_calibration_6_side();
		}
#endif
		else if (memcmp(line, command_mag, sizeof(command_mag)) == 0) {
			if (arg == NULL) {
				// No argument: show status
				printk("Magnetometer: %s\n", sensor_get_mag_enabled() ? "enabled" : "disabled");
				printk("Hardware: %s\n", sensor_get_sensor_mag_name());
				printk("Magnetometer matrix:\n");
				for (int i = 0; i < 3; i++) {
					printk("%.5f %.5f %.5f %.5f\n",
						(double)retained->magBAinv[0][i],
						(double)retained->magBAinv[1][i],
						(double)retained->magBAinv[2][i],
						(double)retained->magBAinv[3][i]);
				}
				float dir_bias = 0;
				int online_samples = sensor_calibration_online_mag_status(&dir_bias);
				bool mag_has_cal = (retained->magBAinv[0][0] != 0.0f
				                 || retained->magBAinv[0][1] != 0.0f
				                 || retained->magBAinv[0][2] != 0.0f);
				float mag_cv = sensor_calibration_get_mag_quality();
				printk("Calibration: %s (norm_cv=%.3f)\n",
				       mag_has_cal ? "active" : "none", (double)mag_cv);
				printk("Online: %d samples, dir_bias=%.2f\n",
				       online_samples, (double)dir_bias);
			} else {
				char *subcmd = strtok((char *)arg, " ");
				if (strcmp(subcmd, "on") == 0) {
					printk("Enabling magnetometer\n");
					sensor_set_mag_enabled(true);
				} else if (strcmp(subcmd, "off") == 0) {
					printk("Disabling magnetometer\n");
					sensor_set_mag_enabled(false);
				} else if (strcmp(subcmd, "clear") == 0) {
					sensor_calibration_clear_mag(NULL, true);
					printk("Magnetometer calibration cleared\n");
				} else if (strcmp(subcmd, "cal") == 0 || strcmp(subcmd, "calibrate") == 0) {
					sensor_calibration_clear_mag(NULL, true);
					sensor_request_calibration_mag();
					printk("Magnetometer calibration started\n");
				} else {
					printk("Usage: mag [on|off|clear|cal]\n");
				}
			}
		}
		else if (memcmp(line, command_set, sizeof(command_set)) == 0) {
			uint64_t addr = strtoull(arg, NULL, 16);
			uint8_t buf[17];
			snprintk(buf, 17, "%016llx", addr);
			if (addr != 0 && memcmp(buf, arg, 17) == 0) {
				esb_set_pair(addr);
			} else {
				printk("Invalid address\n");
			}
		} else if (memcmp(line, command_pair, sizeof(command_pair)) == 0) {
			esb_reset_pair();
		} else if (memcmp(line, command_clear, sizeof(command_clear)) == 0) {
			esb_clear_pair();
		} else if (memcmp(line, command_channel, sizeof(command_channel)) == 0) {
			if (!arg) {
				printk("Usage: channel <1-100>\n");
				printk("Example: channel 25 - Set RF channel to 25\n");
			} else {
				char *endptr;
				long channel = strtol(arg, &endptr, 10);

				if (*endptr != '\0' || channel < 1 || channel > 100) {
					printk("Invalid channel. Must be a number between 1 and 100.\n");
				} else {
					printk("Setting RF channel to %d\n", (int)channel);
					// Save to retained memory
					retained->rf_channel = (uint8_t)channel;
					retained_update();
					// Save to NVS
					sys_write(
						RF_CHANNEL_ID,
						&retained->rf_channel,
						&retained->rf_channel,
						sizeof(retained->rf_channel)
					);
					printk("RF channel saved to NVS: %u\n", retained->rf_channel);
					// Reinitialize ESB with new channel
					esb_deinitialize();
					k_msleep(10);
					esb_initialize(true); // Channel will be applied inside esb_initialize
					printk("ESB reinitialized with channel %u\n", retained->rf_channel);
				}
			}
		} else if (memcmp(line, command_clearchannel, sizeof(command_clearchannel)) == 0) {
			printk("Clearing RF channel setting (restore default)\n");
			// Clear saved channel (set to 0xFF = use default)
			retained->rf_channel = 0xFF;
			retained_update();
			sys_write(RF_CHANNEL_ID, &retained->rf_channel, &retained->rf_channel, sizeof(retained->rf_channel));
			printk("RF channel cleared, will use default on next boot\n");
			// Reinitialize ESB with default channel
			esb_deinitialize();
			k_msleep(10);
			esb_initialize(true); // Will use default channel since rf_channel is 0xFF
			printk("ESB reinitialized with default channel\n");
		}
#if DFU_EXISTS
		else if (memcmp(line, command_dfu, sizeof(command_dfu)) == 0) {
#if ADAFRUIT_BOOTLOADER
			// Subcommands:
			//   dfu      -> UF2 DFU (USB MSC/CDC)
			//   dfu ota  -> OTA DFU (BLE)
			char *mode = NULL;
			if (arg) {
				mode = strtok((char *)arg, " ");
			}

			if (mode && strcmp(mode, "ota") == 0) {
				printk("Entering OTA DFU (BLE)...\n");
				NRF_POWER->GPREGRET = ADAFRUIT_DFU_MAGIC_OTA_RESET;
			} else if (mode == NULL) {
				printk("Entering UF2 DFU...\n");
				NRF_POWER->GPREGRET = ADAFRUIT_DFU_MAGIC_UF2_RESET;
			} else {
				printk("Error: Unknown DFU mode '%s'. Use: dfu [ota]\n", mode);
				continue;
			}

			k_msleep(100); // Wait for GPREGRET to be written
			sys_request_system_reboot(false);
#endif
#if NRF5_BOOTLOADER
			gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
		}
#endif
		else if (memcmp(line, command_ping, sizeof(command_ping)) == 0) {
			cmd_ping_start();
		}
		else if (memcmp(line, command_nvs, sizeof(command_nvs)) == 0) {
			sys_nvs_stats();
		}
		else if (memcmp(line, command_meow, sizeof(command_meow)) == 0) {
			print_meow();
		} else if (memcmp(line, command_debug, sizeof(command_debug)) == 0) {
			uint32_t duration = 1; // Default 1 second
			if (arg) {
				char *endptr;
				char *arg_char = (char *)arg;
				long dur = strtol(arg_char, &endptr, 10);
				if (endptr != arg_char && *endptr == '\0' && dur >= 1 && dur <= 60) {
					duration = (uint32_t)dur;
				} else {
					printk("Invalid duration (1-60s). Using default 1 seconds.\n");
				}
			}
			sensor_debug_start(duration);
		} else if (memcmp(line, command_range, sizeof(command_range)) == 0) {
#if CONFIG_SENSOR_RANGE_STATS
			if (arg && strcmp((char *)arg, "reset") == 0) {
				sensor_reset_range_stats();
				printk("Sensor range statistics have been reset.\n");
			} else {
				sensor_print_range_stats();
			}
#else
			printk("Sensor range statistics not enabled in configuration.\n");
#endif // CONFIG_SENSOR_RANGE_STATS
#if CONFIG_VQF_BENCH
		} else if (memcmp(line, command_vqfbench, sizeof(command_vqfbench)) == 0) {
			uint32_t iterations = 1000;
			if (arg) {
				char *endptr;
				long parsed = strtol((char *)arg, &endptr, 10);
				if (endptr != (char *)arg && *endptr == '\0' && parsed > 0 && parsed <= 20000) {
					iterations = (uint32_t)parsed;
				} else {
					printk("Invalid iteration count. Using default 1000.\n");
				}
			}
			vqf_run_benchmark(iterations);
#endif // CONFIG_VQF_BENCH
		}	else if (memcmp(line, command_reset, sizeof(command_reset)) == 0) {
			if (arg && memcmp(arg, command_reset_arg_zro, sizeof(command_reset_arg_zro)) == 0) {
				cmd_reset_zro();
			}
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
			else if (arg && memcmp(arg, command_reset_arg_acc, sizeof(command_reset_arg_acc)) == 0) {
				cmd_reset_acc();
			}
#endif
			else if (arg && memcmp(arg, command_reset_arg_mag, sizeof(command_reset_arg_mag)) == 0) {
				sensor_calibration_clear_mag(NULL, true);
			}
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
			else if (arg && memcmp(arg, command_reset_arg_sens, sizeof(command_reset_arg_sens)) == 0) {
				cmd_sens_reset();
			}
#endif
#if CONFIG_SENSOR_USE_TCAL
			else if (arg && memcmp(arg, command_reset_arg_tcal, sizeof(command_reset_arg_tcal)) == 0) {
				cmd_reset_tcal();
			}
#endif
			else if (arg && memcmp(arg, command_reset_arg_bat, sizeof(command_reset_arg_bat)) == 0) {
				cmd_reset_bat();
			} else if (arg && memcmp(arg, command_reset_arg_fusion, sizeof(command_reset_arg_fusion)) == 0) {
				cmd_fusion_reset();
			} else if (arg && memcmp(arg, command_reset_arg_all, sizeof(command_reset_arg_all)) == 0) {
				sys_clear();
			} else {
				printk("Invalid argument\n");
			}
		} else if (memcmp(line, "tdma", 4) == 0) {
			if (arg && strcmp((char *)arg, "on") == 0) {
				tdma_set_enabled(true);
				printk("TDMA enabled\n");
			} else if (arg && strcmp((char *)arg, "off") == 0) {
				tdma_set_enabled(false);
				printk("TDMA disabled\n");
			} else {
				printk("TDMA: %s\n", tdma_is_enabled() ? "enabled" : "disabled");
			}
		} else if (memcmp(line, "test", 4) == 0) {
			if (arg && strcmp((char *)arg, "on") == 0) {
				test_mode_set(true);
				printk("Test mode enabled\n");
			} else if (arg && strcmp((char *)arg, "off") == 0) {
				test_mode_set(false);
				printk("Test mode disabled\n");
			} else {
				printk("Test mode: %s\n", test_mode_get() ? "enabled" : "disabled");
			}
		} else {
			printk("Unknown command\n");
		}
	}
}

#endif
