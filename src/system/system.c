#include "globals.h"
#include "test_mode.h"
#include "sensor/sensor.h"
#include "sensor/calibration.h"
#include "connection/connection.h"
#include "connection/esb.h"
#include "system/esb_ota.h"
#include "watchdog.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <hal/nrf_gpio.h>

#include "system.h"
#include "build_defines.h"

static struct nvs_fs fs;

#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)
#define NVS_PARTITION_SIZE FIXED_PARTITION_SIZE(NVS_PARTITION)

LOG_MODULE_REGISTER(system, LOG_LEVEL_INF);

#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
#define BUTTON_EXISTS true
static void button_thread(void);
K_THREAD_DEFINE(
	button_thread_id,
	1024,
	button_thread,
	NULL,
	NULL,
	NULL,
	6,
	0,
	0
); // TODO: stack increased because of reboot request (to 512) and sensor scan (to 1024)
#else
#pragma message "Button GPIO does not exist"
#endif

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define CLKOUT_NODE DT_NODELABEL(pwmclock)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dock_gpios)
#define DOCK_EXISTS true
static const struct gpio_dt_spec dock = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, dock_gpios);
#else
#pragma message "Dock sense GPIO does not exist"
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, chg_gpios)
#define CHG_EXISTS true
static const struct gpio_dt_spec chg = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, chg_gpios);
#else
#pragma message "Charge sense GPIO does not exist"
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, stby_gpios)
#define STBY_EXISTS true
static const struct gpio_dt_spec stby = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, stby_gpios);
#else
#pragma message "Standby sense GPIO does not exist"
#endif

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, clk_gpios)
#define CLK_EN_EXISTS true
static const struct gpio_dt_spec clk_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, clk_gpios);
static const struct pwm_dt_spec clk_out = {0};
#elif DT_NODE_HAS_PROP(CLKOUT_NODE, pwms)
#define CLK_OUT_EXISTS true
static const struct pwm_dt_spec clk_out = PWM_DT_SPEC_GET(CLKOUT_NODE);
#else
#pragma message "Clock enable GPIO or clock PWM out does not exist"
static const struct pwm_dt_spec clk_out = {0};
#endif

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2
#define NRF5_BOOTLOADER CONFIG_BOARD_HAS_NRF5_BOOTLOADER

#if NRF5_BOOTLOADER
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
#endif

void configure_sense_pins(void)
{
	// Configure dock sense
	bool docked = dock_read();
#if DOCK_EXISTS
	if (docked) {
		nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL); // Still works
		nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_HIGH);
	} else {
		nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_PULLUP); // Still works
		nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_LOW);
	}
	LOG_INF("Configured dock sense");
#endif
	// Configure chgstat sense
	if (!docked) {
		bool ignore_charge_wake = IGNORE_CHARGE_WAKE_ON_VBUS && vbus_read();
		if (ignore_charge_wake) {
			LOG_INF("Skipped charge wake sense while VBUS is present");
		}
#if CHG_EXISTS
		if (!ignore_charge_wake) {
			nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chg_gpios), NRF_GPIO_PIN_PULLUP);
			nrf_gpio_cfg_sense_set(
				NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chg_gpios),
				chg_read() ? NRF_GPIO_PIN_SENSE_HIGH : NRF_GPIO_PIN_SENSE_LOW
			);
			LOG_INF("Configured chg sense");
		}
#endif
#if STBY_EXISTS
		if (!ignore_charge_wake) {
			nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, stby_gpios), NRF_GPIO_PIN_PULLUP);
			nrf_gpio_cfg_sense_set(
				NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, stby_gpios),
				stby_read() ? NRF_GPIO_PIN_SENSE_HIGH : NRF_GPIO_PIN_SENSE_LOW
			);
			LOG_INF("Configured stby sense");
		}
#endif
	}
	// Configure sw0 sense
#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured sw0 sense");
#endif
}

static bool nvs_init = false;

static inline bool sys_nvs_init(void)
{
	if (nvs_init) {
		return true;
	}
	struct flash_pages_info info;
	fs.flash_device = NVS_PARTITION_DEVICE;
	fs.offset = NVS_PARTITION_OFFSET; // starting at NVS_PARTITION_OFFSET
	if (flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info)) {
		LOG_ERR("Failed to get page info");
		return false;
	}
	fs.sector_size = info.size; // sector_size equal to the pagesize
	fs.sector_count = 6U;       // 6 sectors
	int err = nvs_mount(&fs);
	if (err) {
		LOG_ERR("Failed to mount NVS, error: %d", err);
		return false;
	}
	nvs_init = true;
	return true;
}

static bool ram_retention_valid = false;

static int sys_retained_init(void)
{
#ifdef NRF_RESET
	bool reset_pin_reset = NRF_RESET->RESETREAS & 0x01;
#else
	bool reset_pin_reset = NRF_POWER->RESETREAS & 0x01;
#endif
	// on most nrf, reset by pin reset will clear retained
	if (!reset_pin_reset) { // if reset reason is not by pin reset, system automatically trusts retained state
		ram_retention_valid = true;
	}
	// All contents of NVS was stored in RAM to not need initializing NVS often
	if (!retained_validate()) // Check ram retention
	{
		LOG_WRN("Invalidated RAM");
		if (!sys_nvs_init()) {
			LOG_ERR("NVS init failed during retained init, cannot restore data from flash");
			// Don't try to read from NVS if it failed to init
			// The data will be zero/uninitialized but we can't recover it
			retained_update();
			return 0;
		}
		// read from nvs to retained
		sys_read(PAIRED_ID, &retained->paired_addr, sizeof(retained->paired_addr));
		sys_read(MAIN_SENSOR_DATA_ID, &retained->sensor_data, sizeof(retained->sensor_data));
		sys_read(MAIN_ACCEL_BIAS_ID, &retained->accelBias, sizeof(retained->accelBias));
		sys_read(MAIN_GYRO_BIAS_ID, &retained->gyroBias, sizeof(retained->gyroBias));
		sys_read(MAIN_MAG_BIAS_ID, &retained->magBAinv, sizeof(retained->magBAinv));
		sys_read(MAIN_ACC_6_BIAS_ID, &retained->accBAinv, sizeof(retained->accBAinv));
		sys_read(BATT_STATS_CURVE_ID, &retained->battery_pptt_curve, sizeof(retained->battery_pptt_curve));
		sys_read(MAIN_GYRO_SENS_ID, &retained->gyroSensScale, sizeof(retained->gyroSensScale));
		// If gyroSensScale was never set in NVS (all zeros), restore default values
		if (retained->gyroSensScale[0] == 0.0f &&
		    retained->gyroSensScale[1] == 0.0f &&
		    retained->gyroSensScale[2] == 0.0f) {
			retained->gyroSensScale[0] = 1.0f;
			retained->gyroSensScale[1] = 1.0f;
			retained->gyroSensScale[2] = 1.0f;
		}
#if CONFIG_SENSOR_USE_TCAL
		sys_read(MAIN_GYRO_TEMP_ID, &retained->gyroTemp, sizeof(retained->gyroTemp));
		sys_read(MAIN_GYRO_TCAL_POINTS_ID, &retained->tempCalPoints, sizeof(retained->tempCalPoints));
		sys_read(MAIN_GYRO_TCAL_COEFFS_ID, &retained->tempCalCoeffs, sizeof(retained->tempCalCoeffs));
		// tempCalCorrectionOffset is retained for compatibility only; no longer used.
		sys_read(MAIN_GYRO_TCAL_STATE_ID, &retained->tempCalState, sizeof(retained->tempCalState));
#endif
		sys_read(RF_CHANNEL_ID, &retained->rf_channel, sizeof(retained->rf_channel));
		sys_read(MAG_ENABLED_ID, &retained->mag_enabled, sizeof(retained->mag_enabled));
		retained_update();
	} else {
		LOG_INF("Validated RAM");
		ram_retention_valid = true;
		// Still need to init NVS for later sys_read/sys_write calls (e.g., battery_tracker)
		sys_nvs_init();
	}
	return 0;
}

SYS_INIT(sys_retained_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// read from retained
uint8_t reboot_counter_read(void)
{
	if (!ram_retention_valid) // system cannot trust retained state, read from nvs
	{
		sys_nvs_init();
		nvs_read(&fs, RBT_CNT_ID, &retained->reboot_counter, sizeof(retained->reboot_counter));
		retained_update();
	}
	return retained->reboot_counter;
}

// write to retained
void reboot_counter_write(uint8_t reboot_counter)
{
	retained->reboot_counter = reboot_counter;
	if (!ram_retention_valid) // system cannot trust retained state, write to nvs
	{
		sys_nvs_init();
		nvs_write(&fs, RBT_CNT_ID, &retained->reboot_counter, sizeof(retained->reboot_counter));
	}
	retained_update();
}

// write to retained and nvs
void sys_write(uint16_t id, void *retained_ptr, const void *data, size_t len)
{
	if (!sys_nvs_init()) {
		LOG_ERR("sys_write: NVS init failed, cannot write ID %d", id);
		if (retained_ptr) {
			memcpy(retained_ptr, data, len);
			retained_update();
		}
		return;
	}
	if (retained_ptr) {
		memcpy(retained_ptr, data, len);
	}
	int err = nvs_write(&fs, id, data, len);
	if (err < 0) {
		LOG_ERR("Failed to write to NVS, error: %d", err);
		return;
	}
	if (retained_ptr) {
		retained_update();
	}
}

void sys_read(uint16_t id, void *data, size_t len)
{
	memset(data, 0, len);

	if (!sys_nvs_init()) {
		LOG_ERR("sys_read: NVS init failed, cannot read ID %d", id);
		return;
	}
	int err = nvs_read(&fs, id, data, len);
	if (err < 0) {
		if (err == -ENOENT) // suppress ENOENT
		{
			LOG_DBG("No entry exists for ID %d, read data set to zero", id);
		} else {
			LOG_ERR("Failed to read from NVS, error: %d", err);
			LOG_WRN("Read data set to zero");
		}
		return;
	}
	if ((size_t)err < len) {
		LOG_WRN("Short NVS read for ID %d: got %d bytes, expected %zu", id, err, len);
	}
}

void sys_clear(void)
{

	static bool reset_confirm = false;
	if (!reset_confirm) {
		printk(
			"Resetting NVS and retained will clear all pairing, sensor calibration data, and battery calibration data. "
			"Are you sure?\n"
		);
		reset_confirm = true;
		return;
	}
	printk("Resetting NVS and retained\n");

	sys_nvs_init();
	memset(retained, 0, sizeof(*retained));
	nvs_clear(&fs);
	nvs_init = false;
	reset_confirm = false;

	// Re-initialize fields that need non-zero default values
	retained->gyroSensScale[0] = 1.0f;
	retained->gyroSensScale[1] = 1.0f;
	retained->gyroSensScale[2] = 1.0f;
	retained->build_timestamp = BUILD_TIMESTAMP;
	retained_update();

	LOG_INF("NVS and retained reset");
}

void sys_nvs_stats(void)
{
	if (!sys_nvs_init()) {
		printk("NVS init failed\n");
		return;
	}

	printk("Storage partition: %u bytes\n", NVS_PARTITION_SIZE);
	printk(
		"Allocated NVS: %u * %u = %u bytes\n",
		fs.sector_size,
		fs.sector_count,
		fs.sector_size * fs.sector_count
	);
	printk(
		"NVS free: %d bytes, max item: %d bytes\n",
		nvs_calc_free_space(&fs),
		nvs_sector_max_data_size(&fs)
	);
}

// return 0 if clock applied, -1 if failed (because there is no clk_en or clk_out)
int set_sensor_clock(bool enable, float rate, float *actual_rate)
{
	*actual_rate = 0;
#if CLK_EN_EXISTS
	int ret = gpio_pin_set_dt(&clk_en, enable);
	if (ret) {
		LOG_ERR("CLK_EN GPIO set to %d failed (ret=%d)", enable, ret);
		return ret;
	}
	LOG_INF("CLK_EN GPIO set to %d", enable);
	if (enable) {
		k_msleep(2); // allow external oscillator to stabilize
		*actual_rate = 32768;
	}
	return 0;
#endif
	if (!device_is_ready(clk_out.dev)) {
		if (enable) {
			LOG_WRN("Clock output device not ready");
		}
		return -1;
	}
	int err = pwm_set_dt(&clk_out, PWM_HZ(rate), enable ? PWM_HZ(rate * 2) : 0);
	if (err) {
		LOG_ERR("PWM clock output set failed (err=%d)", err);
		return err;
	}
	if (enable) {
		*actual_rate = rate;
	}
	return 0;
}

#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static int64_t press_time = 0;
static int64_t last_press_duration = 0;

static void button_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	bool pressed = button_read();
	int64_t current_time = k_uptime_get();
	if (press_time && !pressed && current_time - press_time > 50) { // debounce
		last_press_duration = current_time - press_time;
	} else if (press_time && pressed) { // unusual press event on button already pressed
		return;
	}
	press_time = pressed ? current_time : 0;
}

static struct gpio_callback button_cb_data;

static int sys_button_init(void)
{
	gpio_pin_configure_dt(&button0, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&button_cb_data, button_interrupt_handler, BIT(button0.pin));
	gpio_add_callback(button0.port, &button_cb_data);
	return 0;
}

SYS_INIT(sys_button_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif

bool button_read(void)
{
#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
	return gpio_pin_get_dt(&button0);
#else
	return false;
#endif
}

#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
static void button_thread(void)
{
	int num_presses = 0;
	int64_t last_press = 0;

	/* Register button thread with watchdog */
	watchdog_register_thread(WDT_CHANNEL_BUTTON, 0);

	while (1) {
		if (press_time && k_uptime_get() - press_time > 50) // debounce
		{
			if (!get_status(SYS_STATUS_BUTTON_PRESSED)) {
				set_status(SYS_STATUS_BUTTON_PRESSED, true);
			}
			set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_HIGHEST);
		}
		if (last_press_duration > 50) // debounce
		{
			if (!get_status(SYS_STATUS_BUTTON_PRESSED)) {
				set_status(SYS_STATUS_BUTTON_PRESSED, true);
			}
			num_presses++;
			LOG_INF("Button pressed %d times", num_presses);
			last_press_duration = 0;
			last_press = k_uptime_get();
			set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_HIGHEST);
		}
		/* Block all button actions during OTA (active or suppressed) */
		bool ota_busy = esb_ota_is_active() || connection_get_ota_suppressed();
		if (last_press && k_uptime_get() - last_press > 1000) {
			LOG_INF("Button was pressed %d times", num_presses);
			last_press = 0;
			if (ota_busy) {
				LOG_INF("Button action blocked by OTA");
				set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
			} else if (num_presses == 1) {
				if (test_mode_get()) {
					LOG_INF("Button reboot blocked by test mode");
				} else {
					sys_request_system_reboot(false);
				}
			}
#if CONFIG_USER_EXTRA_ACTIONS // TODO: extra actions are default until server can send commands to trackers
			if (!ota_busy) {
				sys_reset_mode(num_presses - 1);
			}
#endif
			num_presses = 0;
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
			set_status(SYS_STATUS_BUTTON_PRESSED, false);
		}
		if (press_time && k_uptime_get() - press_time > 1000 && button_read()) // Button is being held
		{
			if (ota_busy) {
				LOG_INF("Button hold blocked by OTA");
				press_time = 0;
				set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
				set_status(SYS_STATUS_BUTTON_PRESSED, false);
			} else if (sys_user_shutdown()) // held for 5 seconds, reset pairing
			{
				LOG_INF("Pairing requested");
				esb_reset_pair();
				press_time = 0;
				set_status(SYS_STATUS_BUTTON_PRESSED, false); // TODO: is needed?
			}
			else // shutting down or rebooting
			{
				k_thread_abort(button_thread_id);
			}
		}

		/* Feed watchdog at end of each loop iteration */
		watchdog_feed(WDT_CHANNEL_BUTTON);

		k_msleep(20);
	}
}
#endif

static int sys_gpio_init(void)
{
#if DOCK_EXISTS // configure if exists
	gpio_pin_configure_dt(&dock, GPIO_INPUT);
#endif
#if CHG_EXISTS
	gpio_pin_configure_dt(&chg, GPIO_INPUT);
#endif
#if STBY_EXISTS
	gpio_pin_configure_dt(&stby, GPIO_INPUT);
#endif
#if CLK_EN_EXISTS
	gpio_pin_configure_dt(&clk_en, GPIO_OUTPUT);
#endif
#if DCDC_EN_EXISTS
	gpio_pin_configure_dt(&dcdc_en, GPIO_OUTPUT);
#endif
#if LDO_EN_EXISTS
	gpio_pin_configure_dt(&ldo_en, GPIO_OUTPUT);
#endif
	return 0;
}

SYS_INIT(sys_gpio_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

bool dock_read(void)
{
#if DOCK_EXISTS
	return gpio_pin_get_dt(&dock);
#else
	return false;
#endif
}

bool chg_read(void)
{
#if CHG_EXISTS
	return gpio_pin_get_dt(&chg);
#else
	return false;
#endif
}

bool stby_read(void)
{
#if STBY_EXISTS
	return gpio_pin_get_dt(&stby);
#else
	return false;
#endif
}

int sys_user_shutdown(void)
{
	int64_t start_time = k_uptime_get();
#if USER_SHUTDOWN_ENABLED
	LOG_INF("User shutdown requested");
	reboot_counter_write(0);
	set_led(SYS_LED_PATTERN_ONESHOT_POWEROFF, SYS_LED_PRIORITY_HIGHEST);
#endif
	k_msleep(1500);
	if (button_read()) // If alternate button is available and still pressed, wait for the user to stop pressing the
					   // button
	{
		set_led(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_HIGHEST);
		bool led_on = false;
		while (button_read()) {
			if (!led_on && k_uptime_get() - start_time > 500) // long pattern starts with led on, so delay pattern a bit
			{
				set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_HIGHEST);
				led_on = 1;
			}
			if (k_uptime_get() - start_time > 4000) // held for over 5 seconds, cancel shutdown
			{
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
				return 1;
			}
			k_msleep(1);
		}
		set_led(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_HIGHEST);
	}
#if USER_SHUTDOWN_ENABLED
	sys_request_system_off(false);
#else
	sys_request_system_reboot(false);
#endif
	return 0;
}

void sys_command_shutdown(void)
{
	LOG_INF("Command shutdown requested");
	reboot_counter_write(0);
	set_led(SYS_LED_PATTERN_ONESHOT_POWEROFF, SYS_LED_PRIORITY_HIGHEST);
	k_msleep(1500);
	sys_request_system_off(false);
}

void sys_reset_mode(uint8_t mode)
{
	switch (mode) {
#if CONFIG_USER_EXTRA_ACTIONS
	case 1:
		LOG_INF("IMU calibration requested");
		sensor_request_calibration();
		break;
#endif
	case 2: // Reset mode pairing reset
		LOG_INF("Pairing reset requested");
		esb_reset_pair();
		break;
#if DFU_EXISTS // Using DFU bootloader
#if !defined(CONFIG_BOARD_STYRIA_MINI_UF2)
	case 3:
	case 4: // Reset mode DFU
#else
	case 5:
	case 6: // Reset mode DFU
#endif
		LOG_INF("DFU requested");
#if ADAFRUIT_BOOTLOADER
		NRF_POWER->GPREGRET = ADAFRUIT_DFU_MAGIC_UF2_RESET;
		sys_request_system_reboot(false);
#endif
#if NRF5_BOOTLOADER
		gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
		break;
	case 7:
	case 8: // Reset mode DFU OTA
		LOG_INF("DFU OTA requested");
#if ADAFRUIT_BOOTLOADER
		NRF_POWER->GPREGRET = ADAFRUIT_DFU_MAGIC_OTA_RESET;
		sys_request_system_reboot(false);
#endif
#if NRF5_BOOTLOADER
		gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
#endif
	default:
		break;
	}
}
