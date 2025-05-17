#include "globals.h"
#include "system/system.h"
#include "sensor/sensor.h"
#include "sensor/calibration.h"
#include "connection/esb.h"
#include "build_defines.h"

#if CONFIG_USB_DEVICE_STACK
#define USB DT_NODELABEL(usbd)
#define USB_EXISTS (DT_NODE_HAS_STATUS(USB, okay) && CONFIG_UART_CONSOLE)
#endif

#if (USB_EXISTS || CONFIG_RTT_CONSOLE) && CONFIG_USE_SLIMENRF_CONSOLE

#if USB_EXISTS
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/console/console.h>
#include <zephyr/logging/log_ctrl.h>
#else
#include "system/rtt_console.h"
#endif
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/pm/device.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

static void usb_init_thread(void);
K_THREAD_DEFINE(usb_init_thread_id, 256, usb_init_thread, NULL, NULL, NULL, 6, 0, 500); // Wait before enabling USB

static void console_thread(void);
static struct k_thread console_thread_id;
static K_THREAD_STACK_DEFINE(console_thread_id_stack, 512);

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2
#define NRF5_BOOTLOADER CONFIG_BOARD_HAS_NRF5_BOOTLOADER

#if NRF5_BOOTLOADER
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(mag))
#define SENSOR_MAG_EXISTS true
#endif

static const char *meows[] = {
	"Mew",
	"Meww",
	"Meow",
	"Meow meow",
	"Mrrrp",
	"Mrrf",
	"Mreow",
	"Mrrrow",
	"Mrrr",
	"Purr",
	"mew",
	"meww",
	"meow",
	"meow meow",
	"mrrrp",
	"mrrf",
	"mreow",
	"mrrrow",
	"mrrr",
	"purr",
};

static const char *meow_punctuations[] = {
	".",
	"?",
	"!",
	"-",
	"~",
	""
};

static const char *meow_suffixes[] = {
	" :3",
	" :3c",
	" ;3",
	" ;3c",
	" x3",
	" x3c",
	" X3",
	" X3c",
	" >:3",
	" >:3c",
	" >;3",
	" >;3c",
	""
};

static void console_thread_create(void)
{
	k_thread_create(&console_thread_id, console_thread_id_stack, K_THREAD_STACK_SIZEOF(console_thread_id_stack), (k_thread_entry_t)console_thread, NULL, NULL, NULL, 6, 0, K_NO_WAIT);
}

#if USB_EXISTS
static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	const struct log_backend *backend = log_backend_get_by_name("log_backend_uart");
	const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	switch (status)
	{
	case USB_DC_CONNECTED:
		set_status(SYS_STATUS_USB_CONNECTED, true);
		pm_device_action_run(cons, PM_DEVICE_ACTION_RESUME);
		log_backend_enable(backend, backend->cb->ctx, CONFIG_LOG_MAX_LEVEL);
		console_thread_create();
		break;
	case USB_DC_DISCONNECTED:
		set_status(SYS_STATUS_USB_CONNECTED, false);
		k_thread_abort(&console_thread_id);
		log_backend_disable(backend);
		pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}
#endif

static void usb_init_thread(void)
{
#if USB_EXISTS
	usb_enable(status_cb);
#else
	console_thread_create();
#endif
}

static void print_info(void)
{
#if USB_EXISTS
	printk(CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT "\n");
#endif
	printk(FW_STRING);

	printk("\nBoard: " CONFIG_BOARD "\n");
	printk("SOC: " CONFIG_SOC "\n");
	printk("Target: " CONFIG_BOARD_TARGET "\n");

	printk("\nIMU: %s\n", sensor_get_sensor_imu_name());
	if (retained->imu_reg != 0xFF)
		printk("Interface: %s\n", (retained->imu_reg & 0x80) ? "SPI" : "I2C");
	printk("Address: 0x%02X%02X\n", retained->imu_addr, retained->imu_reg);

#if SENSOR_MAG_EXISTS
	printk("\nMagnetometer: %s\n", sensor_get_sensor_mag_name());
	if (retained->mag_reg != 0xFF)
		printk("Interface: %s%s\n", (retained->mag_reg & 0x80) ? "SPI" : "I2C", (retained->mag_addr & 0x80) ? ", external" : "");
	printk("Address: 0x%02X%02X\n", retained->mag_addr, retained->mag_reg);
#endif

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	printk("\nAccelerometer matrix:\n");
	for (int i = 0; i < 3; i++)
		printk("%.5f %.5f %.5f %.5f\n", (double)retained->accBAinv[0][i], (double)retained->accBAinv[1][i], (double)retained->accBAinv[2][i], (double)retained->accBAinv[3][i]);
#else
	printk("\nAccelerometer bias: %.5f %.5f %.5f\n", (double)retained->accelBias[0], (double)retained->accelBias[1], (double)retained->accelBias[2]);
#endif
	printk("Gyroscope bias: %.5f %.5f %.5f\n", (double)retained->gyroBias[0], (double)retained->gyroBias[1], (double)retained->gyroBias[2]);
#if SENSOR_MAG_EXISTS
//	printk("Magnetometer bridge offset: %.5f %.5f %.5f\n", (double)retained->magBias[0], (double)retained->magBias[1], (double)retained->magBias[2]);
	printk("Magnetometer matrix:\n");
	for (int i = 0; i < 3; i++)
		printk("%.5f %.5f %.5f %.5f\n", (double)retained->magBAinv[0][i], (double)retained->magBAinv[1][i], (double)retained->magBAinv[2][i], (double)retained->magBAinv[3][i]);
#endif

	printk("\nFusion: %s\n", sensor_get_sensor_fusion_name());

	printk("\nTracker ID: %u\n", retained->paired_addr[1]);
	printk("Device address: %012llX\n", *(uint64_t *)NRF_FICR->DEVICEADDR & 0xFFFFFFFFFFFF);
	printk("Receiver address: %012llX\n", (*(uint64_t *)&retained->paired_addr[0] >> 16) & 0xFFFFFFFFFFFF);
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
		
				printk("Gyroscope sensitivity (degrees diff over %u rev): %.3f %.3f %.3f\n", (int)CONFIG_SENSOR_SENS_REV, (double)deg_x, (double)deg_y, (double)deg_z);
			} else {
				printk("Gyroscope sensitivity: Retained data unavailable.\n");
		}
#endif		
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

static void console_thread(void)
{
#if DFU_EXISTS
	if (button_read()) // button held on usb connect, enter DFU
	{
#if ADAFRUIT_BOOTLOADER
		NRF_POWER->GPREGRET = 0x57;
		sys_request_system_reboot();
#endif
#if NRF5_BOOTLOADER
		gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
	}
#endif

#if USB_EXISTS
	console_getline_init();
	while (log_data_pending())
		k_usleep(1);
	k_msleep(100);
	printk("*** " CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT " ***\n");
#endif
	printk(FW_STRING);
	printk("info                         Get device information\n");
	printk("uptime                       Get device uptime\n");
	printk("reboot                       Soft reset the device\n");
	printk("calibrate                    Calibrate sensor ZRO\n");
#if CONFIG_SENSOR_USE_SENS_CALIBRATION	
	printk("sens <x>,<y>,<z>             Set gyro sensitivity (deg diff over %u rev)\n", (int)CONFIG_SENSOR_SENS_REV);
	printk("sens reset                   Reset gyro sensitivity calibration\n");
#endif
	uint8_t command_info[] = "info";
	uint8_t command_uptime[] = "uptime";
	uint8_t command_reboot[] = "reboot";
	uint8_t command_calibrate[] = "calibrate";

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	printk("6-side                       Calibrate 6-side accelerometer\n");

	uint8_t command_6_side[] = "6-side";
#endif

#if SENSOR_MAG_EXISTS
	printk("mag                          Clear magnetometer calibration\n");

	uint8_t command_mag[] = "mag";
#endif

	printk("pair                         Clear pairing data\n");

	uint8_t command_pair[] = "pair";

#if DFU_EXISTS
	printk("dfu                          Enter DFU bootloader\n");

	uint8_t command_dfu[] = "dfu";
#endif

	printk("meow                         Meow!\n");

	uint8_t command_meow[] = "meow";
#if CONFIG_SENSOR_USE_SENS_CALIBRATION	
	uint8_t command_sens[] = "sens";
	uint8_t command_sens_reset[] = "sens reset";
#endif

	while (1) {
#if USB_EXISTS
		uint8_t *line = console_getline();
#else
		uint8_t *line = rtt_console_getline();
#endif
		for (uint8_t *p = line; *p; ++p) {
			*p = tolower(*p);
		}

		if (memcmp(line, command_info, sizeof(command_info)) == 0)
		{
			print_info();
		}
		else if (memcmp(line, command_uptime, sizeof(command_uptime)) == 0)
		{
			uint64_t uptime = k_uptime_ticks();
			print_uptime(uptime, "Uptime");
			print_uptime(uptime - retained->uptime_latest + retained->uptime_sum, "Accumulated");
		}
		else if (memcmp(line, command_reboot, sizeof(command_reboot)) == 0)
		{
			sys_request_system_reboot();
		}
		else if (memcmp(line, command_calibrate, sizeof(command_calibrate)) == 0)
		{
			sensor_request_calibration();
		}
#if CONFIG_SENSOR_USE_SENS_CALIBRATION		
			else if (memcmp(line, command_sens_reset, sizeof(command_sens_reset) - 1) == 0)
			{
				if (retained) {
					printk("Resetting gyro sensitivity calibration.\n");
					retained->gyroSensScale[0] = 1.0f;
					retained->gyroSensScale[1] = 1.0f;
					retained->gyroSensScale[2] = 1.0f;
					retained_update(); // Save changes
					printk("Gyro sensitivity reset to 0, 0, 0\n");
				} else {
					printk("Error: Retained data not available.\n");
				}
			}
			else if (strncmp(line, command_sens, sizeof(command_sens) - 1) == 0)
			{
				float deg_x = 0.0f, deg_y = 0.0f, deg_z = 0.0f;
				bool parse_ok = false;
			char *values_str_modifiable = (char *)line + sizeof(command_sens) - 1;

			char *token;
			char *endptr; // For strtof error checking
			int token_count = 0;

			// Get the first token (X value)
			token = strtok(values_str_modifiable, ",");
			if (token != NULL) {
				deg_x = strtof(token, &endptr);
				// Check if conversion was successful (endptr should point to '\0' or the comma)
				if (*endptr == '\0' || *endptr == ',') {
					token_count++;
					// Get the second token (Y value)
					token = strtok(NULL, ",");
					if (token != NULL) {
						deg_y = strtof(token, &endptr);
						if (*endptr == '\0' || *endptr == ',') {
							token_count++;
							// Get the third token (Z value)
							token = strtok(NULL, ","); // Use comma or just end of string
							if (token != NULL) {
								deg_z = strtof(token, &endptr);
								// For the last token, endptr must point to the null terminator
								if (*endptr == '\0') {
									token_count++;
										parse_ok = true;
									}
								}
						}
						}
					}

			if (parse_ok && token_count == 3) {
				
				if (retained) {
					float scale_x, scale_y, scale_z; // Local variables for scale
					float den_x = 1.0f - (deg_x / (360.0f * CONFIG_SENSOR_SENS_REV));
					float den_y = 1.0f - (deg_y / (360.0f * CONFIG_SENSOR_SENS_REV));
					float den_z = 1.0f - (deg_z / (360.0f * CONFIG_SENSOR_SENS_REV));

					// Prevent division by zero or near-zero
					if (fabsf(den_x) < 1e-6f || fabsf(den_y) < 1e-6f || fabsf(den_z) < 1e-6f) {
						printk("Error: Invalid input degrees leading to division by zero. Calibration not applied.\n");
					} else {
						// Calculate the scale factors
						scale_x = 1.0f / den_x;
						scale_y = 1.0f / den_y;
						scale_z = 1.0f / den_z;

						// Assign calculated scales to the retained structure 
						retained->gyroSensScale[0] = scale_x;
						retained->gyroSensScale[1] = scale_y;
						retained->gyroSensScale[2] = scale_z;

						// Save the updated retained data
						retained_update();

						printk("Gyro sensitivity scale set to: %.3f, %.3f, %.3f\n", (double)deg_x, (double)deg_y, (double)deg_z);
					}
				}

					
				} else {
					printk("Error: Invalid format. Use: sens <x>,<y>,<z>\n");
					printk("Example: sens 10.5,-2.1,15.0\n");
				}  
				} else {
					printk("Error: Invalid format. Use: sens <x>,<y>,<z>\n");
					printk("Example: sens 10.5,-2.1,15.0\n");
			}
			}
#endif			
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		else if (memcmp(line, command_6_side, sizeof(command_6_side)) == 0)
		{
			sensor_request_calibration_6_side();
		}
#endif
#if SENSOR_MAG_EXISTS
		else if (memcmp(line, command_mag, sizeof(command_mag)) == 0)
		{
			sensor_calibration_clear_mag(true);
		}
#endif
		else if (memcmp(line, command_pair, sizeof(command_pair)) == 0) 
		{
//			reboot_counter_write(102);
			esb_reset_pair(); // TODO: make not require reboot
			sys_request_system_reboot();
		}
#if DFU_EXISTS
		else if (memcmp(line, command_dfu, sizeof(command_dfu)) == 0)
		{
#if ADAFRUIT_BOOTLOADER
			NRF_POWER->GPREGRET = 0x57;
			sys_request_system_reboot();
#endif
#if NRF5_BOOTLOADER
			gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
		}
#endif
		else if (memcmp(line, command_meow, sizeof(command_meow)) == 0) 
		{
			print_meow();
		}
		else
		{
			printk("Unknown command\n");
		}
	}
}

#endif