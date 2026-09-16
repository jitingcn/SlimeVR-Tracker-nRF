#include "globals.h"
#include "system/system.h"
#include "system/battery_tracker.h"
#include "system/test_mode.h"
#include "sensor/sensor.h"
#include "sensor/calibration/calibration.h"
#include "sensor/calibration/online_mag.h"
#if CONFIG_VQF_BENCH
#include "sensor/fusion/vqf/vqf.h"
#endif
#include "connection/esb.h"
#include "connection/connection.h"
#include "connection/channel_control.h"
#include "connection/tdma.h"
#if defined(CONFIG_TDMA_DIAGNOSTICS)
#include "connection/radio_capture.h"
#endif
#include "build_defines.h"
#include "parse_args.h"
#include "zephyr/sys/printk.h"
#if CONFIG_THREAD_ANALYZER
#include <zephyr/debug/thread_analyzer.h>
#endif

#define USB_EXISTS 0
#if CONFIG_USB_DEVICE_STACK_NEXT
#undef USB_EXISTS
#define USB DT_NODELABEL(zephyr_udc0)
#define USB_EXISTS (DT_NODE_HAS_STATUS(USB, okay) && CONFIG_UART_CONSOLE && \
	DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart))
#endif

/*
 * Interactive console over the chosen console device. UART input is parsed
 * by the private IRQ line editor below; the worker stays blocked on its queue
 * while USB is disconnected.
 */
#define UART_CONSOLE_EXISTS \
	(!USB_EXISTS && CONFIG_UART_CONSOLE && \
	 DT_NODE_HAS_STATUS(DT_CHOSEN(zephyr_console), okay))

#if (USB_EXISTS || UART_CONSOLE_EXISTS || CONFIG_RTT_CONSOLE) && CONFIG_USE_SLIMENRF_CONSOLE

#if USB_EXISTS || UART_CONSOLE_EXISTS
#include <zephyr/drivers/uart.h>
#else
#include "system/rtt_console.h"
#endif
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>

#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

static void console_thread(void);

#if USB_EXISTS || UART_CONSOLE_EXISTS
#include <zephyr/device.h>
#include <errno.h>

static const struct device *const console_uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
#if USB_EXISTS
static struct k_thread console_thread_id;
static K_THREAD_STACK_DEFINE(console_thread_stack, 1536);
static bool console_thread_started;
#endif
static void console_uart_irq(const struct device *dev, void *user_data);

#define CONSOLE_LINE_QUEUE_DEPTH 4
#define CONSOLE_ECHO_BUFFER_SIZE 512
#define CONSOLE_INPUT_DRAIN_MAX 4096
#define CONSOLE_LINE_MAX_LEN CONFIG_CONSOLE_INPUT_MAX_LINE_LEN

BUILD_ASSERT(CONSOLE_LINE_MAX_LEN >= 2, "Console line buffer must hold an empty line");

struct console_line_message {
	uint32_t epoch;
	char line[CONSOLE_LINE_MAX_LEN];
};

enum console_escape_state {
	CONSOLE_ESCAPE_NONE,
	CONSOLE_ESCAPE_START,
	CONSOLE_ESCAPE_CSI,
	CONSOLE_ESCAPE_SS3,
};

struct console_input_state {
	struct k_spinlock lock;
	bool initialized;
	bool active;
	bool overflow;
	bool last_was_cr;
	enum console_escape_state escape_state;
	bool escape_has_value;
	bool escape_ignore_value;
	uint16_t escape_value;
	uint16_t cursor;
	uint16_t tail;
	uint32_t epoch;
	char line[CONSOLE_LINE_MAX_LEN];
	uint8_t echo[CONSOLE_ECHO_BUFFER_SIZE];
	uint16_t echo_head;
	uint16_t echo_tail;
};
K_MSGQ_DEFINE(console_line_msgq, sizeof(struct console_line_message), CONSOLE_LINE_QUEUE_DEPTH, 4);

static struct console_input_state console_input;
static bool console_echo_has_data_locked(void)
{
	return console_input.echo_head != console_input.echo_tail;
}

static void console_echo_put_locked(uint8_t byte)
{
	uint16_t next = (uint16_t)((console_input.echo_head + 1U) % CONSOLE_ECHO_BUFFER_SIZE);

	if (next == console_input.echo_tail) {
		return;
	}

	console_input.echo[console_input.echo_head] = byte;
	console_input.echo_head = next;
}

static void console_echo_text_locked(const char *text)
{
	while (*text != '\0') {
		console_echo_put_locked((uint8_t)*text++);
	}
}

static void console_echo_cursor_locked(uint8_t direction, uint16_t count)
{
	if (count == 0U) {
		return;
	}

	console_echo_put_locked(0x1b);
	console_echo_put_locked('[');
	if (count >= 100U) {
		console_echo_put_locked((uint8_t)('0' + count / 100U));
		count %= 100U;
		console_echo_put_locked((uint8_t)('0' + count / 10U));
		console_echo_put_locked((uint8_t)('0' + count % 10U));
	} else if (count >= 10U) {
		console_echo_put_locked((uint8_t)('0' + count / 10U));
		console_echo_put_locked((uint8_t)('0' + count % 10U));
	} else {
		console_echo_put_locked((uint8_t)('0' + count));
	}
	console_echo_put_locked(direction);
}

static void console_reset_line_locked(void)
{
	console_input.overflow = false;
	console_input.last_was_cr = false;
	console_input.escape_state = CONSOLE_ESCAPE_NONE;
	console_input.escape_has_value = false;
	console_input.escape_ignore_value = false;
	console_input.escape_value = 0;
	console_input.cursor = 0;
	console_input.tail = 0;
}

static void console_drop_queued_lines_locked(void)
{
	struct console_line_message dropped;

	while (k_msgq_get(&console_line_msgq, &dropped, K_NO_WAIT) == 0) {
	}
}

static void console_finish_line_locked(void)
{
	struct console_line_message message = {0};
	uint16_t length = (uint16_t)(console_input.cursor + console_input.tail);

	if (!console_input.overflow) {
		message.epoch = console_input.epoch;
		memcpy(message.line, console_input.line, length);
		message.line[length] = '\0';
		if (k_msgq_put(&console_line_msgq, &message, K_NO_WAIT) != 0) {
			console_echo_put_locked('\a');
		}
	}

	console_echo_text_locked("\r\n");
	console_reset_line_locked();
}

static void console_insert_char_locked(uint8_t byte)
{
	uint16_t length = (uint16_t)(console_input.cursor + console_input.tail);

	if (length >= CONSOLE_LINE_MAX_LEN - 1U) {
		if (!console_input.overflow) {
			console_input.overflow = true;
			console_echo_put_locked('\a');
		}
		return;
	}

	for (uint16_t i = length; i > console_input.cursor; i--) {
		console_input.line[i] = console_input.line[i - 1U];
	}
	console_input.line[console_input.cursor++] = (char)byte;

	console_echo_put_locked(byte);
	if (console_input.tail != 0U) {
		console_echo_text_locked("\x1b[s");
		for (uint16_t i = console_input.cursor; i < console_input.cursor + console_input.tail; i++) {
			console_echo_put_locked((uint8_t)console_input.line[i]);
		}
		console_echo_text_locked("\x1b[u");
	}
}

static void console_backspace_locked(void)
{
	uint16_t length;

	if (console_input.cursor == 0U) {
		return;
	}

	length = (uint16_t)(console_input.cursor + console_input.tail);
	console_input.cursor--;
	for (uint16_t i = console_input.cursor; i + 1U < length; i++) {
		console_input.line[i] = console_input.line[i + 1U];
	}

	console_echo_put_locked('\b');
	if (console_input.tail == 0U) {
		console_echo_text_locked(" \b");
	} else {
		console_echo_text_locked("\x1b[s");
		for (uint16_t i = console_input.cursor; i < console_input.cursor + console_input.tail; i++) {
			console_echo_put_locked((uint8_t)console_input.line[i]);
		}
		console_echo_text_locked(" \x1b[u");
	}
}

static void console_delete_locked(void)
{
	uint16_t length;

	if (console_input.tail == 0U) {
		return;
	}

	length = (uint16_t)(console_input.cursor + console_input.tail);
	for (uint16_t i = console_input.cursor; i + 1U < length; i++) {
		console_input.line[i] = console_input.line[i + 1U];
	}
	console_input.tail--;

	console_echo_cursor_locked('C', 1);
	console_echo_put_locked('\b');
	if (console_input.tail == 0U) {
		console_echo_text_locked(" \b");
	} else {
		console_echo_text_locked("\x1b[s");
		for (uint16_t i = console_input.cursor; i < console_input.cursor + console_input.tail; i++) {
			console_echo_put_locked((uint8_t)console_input.line[i]);
		}
		console_echo_text_locked(" \x1b[u");
	}
}

static void console_move_home_locked(void)
{
	console_echo_cursor_locked('D', console_input.cursor);
	console_input.tail = (uint16_t)(console_input.tail + console_input.cursor);
	console_input.cursor = 0;
}

static void console_move_end_locked(void)
{
	console_echo_cursor_locked('C', console_input.tail);
	console_input.cursor = (uint16_t)(console_input.cursor + console_input.tail);
	console_input.tail = 0;
}

static void console_apply_escape_locked(uint8_t final)
{
	uint16_t count
		= console_input.escape_has_value && console_input.escape_value != 0U ? console_input.escape_value : 1U;

	switch (final) {
	case 'D':
		count = MIN(count, console_input.cursor);
		console_input.cursor -= count;
		console_input.tail = (uint16_t)(console_input.tail + count);
		console_echo_cursor_locked('D', count);
		break;
	case 'C':
		count = MIN(count, console_input.tail);
		console_input.cursor = (uint16_t)(console_input.cursor + count);
		console_input.tail -= count;
		console_echo_cursor_locked('C', count);
		break;
	case 'H':
		console_move_home_locked();
		break;
	case 'F':
		console_move_end_locked();
		break;
	case '~':
		if (console_input.escape_value == 3U) {
			console_delete_locked();
		} else if (console_input.escape_value == 1U || console_input.escape_value == 7U) {
			console_move_home_locked();
		} else if (console_input.escape_value == 4U || console_input.escape_value == 8U) {
			console_move_end_locked();
		}
		break;
	default:
		break;
	}
}

static bool console_handle_escape_locked(uint8_t byte)
{
	switch (console_input.escape_state) {
	case CONSOLE_ESCAPE_START:
		if (byte == '[') {
			console_input.escape_state = CONSOLE_ESCAPE_CSI;
			console_input.escape_has_value = false;
			console_input.escape_ignore_value = false;
			console_input.escape_value = 0;
		} else if (byte == 'O') {
			console_input.escape_state = CONSOLE_ESCAPE_SS3;
		} else {
			console_input.escape_state = CONSOLE_ESCAPE_NONE;
		}
		return true;
	case CONSOLE_ESCAPE_CSI:
		if (byte >= 0x30 && byte <= 0x3f) {
			if (byte >= '0' && byte <= '9') {
				if (!console_input.escape_ignore_value) {
					console_input.escape_has_value = true;
					if (console_input.escape_value < 999U) {
						console_input.escape_value
							= (uint16_t)MIN(999U, console_input.escape_value * 10U + (uint16_t)(byte - '0'));
					}
				}
			} else if (byte == ';') {
				console_input.escape_ignore_value = true;
			}
			return true;
		}
		if (byte >= 0x20 && byte <= 0x2f) {
			return true;
		}
		if (byte >= 0x40 && byte <= 0x7e) {
			console_apply_escape_locked(byte);
		}
		console_input.escape_state = CONSOLE_ESCAPE_NONE;
		return true;
	case CONSOLE_ESCAPE_SS3:
		if (byte >= 0x20 && byte <= 0x2f) {
			return true;
		}
		if (byte >= 0x40 && byte <= 0x7e) {
			console_apply_escape_locked(byte);
		}
		console_input.escape_state = CONSOLE_ESCAPE_NONE;
		return true;
	case CONSOLE_ESCAPE_NONE:
	default:
		return false;
	}
}

static void console_input_byte_locked(uint8_t byte)
{
	if (byte == '\n' && console_input.last_was_cr) {
		console_input.last_was_cr = false;
		return;
	}

	if (byte == '\r' || byte == '\n') {
		console_finish_line_locked();
		console_input.last_was_cr = byte == '\r';
		return;
	}

	console_input.last_was_cr = false;
	if (console_input.overflow) {
		return;
	}

	if (console_input.escape_state != CONSOLE_ESCAPE_NONE) {
		(void)console_handle_escape_locked(byte);
		return;
	}

	if (byte == 0x1b) {
		console_input.escape_state = CONSOLE_ESCAPE_START;
		console_input.escape_has_value = false;
		console_input.escape_ignore_value = false;
		console_input.escape_value = 0;
		return;
	}

	switch (byte) {
	case 0x08:
	case 0x7f:
		console_backspace_locked();
		break;
	case '\t':
		break;
	default:
		if (isprint((unsigned char)byte) != 0) {
			console_insert_char_locked(byte);
		}
		break;
	}
}

static void console_echo_flush(const struct device *dev)
{
	while (true) {
		int ready = uart_irq_tx_ready(dev);
		if (ready <= 0) {
			return;
		}

		k_spinlock_key_t key = k_spin_lock(&console_input.lock);
		if (!console_echo_has_data_locked()) {
			k_spin_unlock(&console_input.lock, key);
			uart_irq_tx_disable(dev);
			return;
		}

		uint16_t head = console_input.echo_head;
		uint16_t tail = console_input.echo_tail;
		uint16_t contiguous = head > tail ? (uint16_t)(head - tail) : (uint16_t)(CONSOLE_ECHO_BUFFER_SIZE - tail);
		int count = (int)contiguous;
		int sent = uart_fifo_fill(dev, &console_input.echo[tail], count);
		if (sent > 0) {
			console_input.echo_tail = (uint16_t)((tail + (uint16_t)sent) % CONSOLE_ECHO_BUFFER_SIZE);
		}
		bool empty = !console_echo_has_data_locked();
		k_spin_unlock(&console_input.lock, key);

		if (sent <= 0) {
			return;
		}
		if (empty) {
			uart_irq_tx_disable(dev);
			return;
		}
	}
}

static void console_uart_irq(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (dev != console_uart_dev) {
		return;
	}

	while (uart_irq_update(dev) > 0 && uart_irq_is_pending(dev) > 0) {
		if (uart_irq_rx_ready(dev) > 0) {
			do {
				uint8_t byte;
				k_spinlock_key_t key = k_spin_lock(&console_input.lock);
				int received = uart_fifo_read(dev, &byte, 1);

				if (received > 0 && console_input.initialized && console_input.active) {
					console_input_byte_locked(byte);
				}
				k_spin_unlock(&console_input.lock, key);

				if (received <= 0) {
					break;
				}
			} while (uart_irq_rx_ready(dev) > 0);
		}

		if (uart_irq_tx_ready(dev) > 0) {
			console_echo_flush(dev);
		}
	}

	k_spinlock_key_t key = k_spin_lock(&console_input.lock);
	bool echo_pending = console_echo_has_data_locked();
	k_spin_unlock(&console_input.lock, key);
	if (echo_pending) {
		uart_irq_tx_enable(dev);
	} else {
		uart_irq_tx_disable(dev);
	}
}

static void console_drain_uart_locked(void)
{
	unsigned char byte;

	for (size_t i = 0; i < CONSOLE_INPUT_DRAIN_MAX; i++) {
		if (uart_poll_in(console_uart_dev, &byte) != 0) {
			break;
		}
	}
}

static int console_input_install(void)
{
	k_spinlock_key_t key = k_spin_lock(&console_input.lock);
	bool initialized = console_input.initialized;
	k_spin_unlock(&console_input.lock, key);
	if (initialized) {
		return 0;
	}

	if (!device_is_ready(console_uart_dev)) {
		LOG_ERR("Console UART is not ready");
		return -ENODEV;
	}

	uart_irq_rx_disable(console_uart_dev);
	uart_irq_tx_disable(console_uart_dev);
	key = k_spin_lock(&console_input.lock);
	console_drain_uart_locked();
	k_spin_unlock(&console_input.lock, key);

	int ret = uart_irq_callback_user_data_set(console_uart_dev, console_uart_irq, NULL);
	if (ret != 0) {
		LOG_ERR("Failed to install console UART input callback: %d", ret);
		return ret;
	}

	key = k_spin_lock(&console_input.lock);
	console_input.initialized = true;
	k_spin_unlock(&console_input.lock, key);
	uart_irq_rx_enable(console_uart_dev);
	return 0;
}

static bool console_line_is_current(uint32_t epoch)
{
	k_spinlock_key_t key = k_spin_lock(&console_input.lock);
	/* Admission survives DTR close; only a hard lifecycle loss retires it. */
	bool current = console_input.epoch == epoch;
	k_spin_unlock(&console_input.lock, key);
	return current;
}

#endif

#if !USB_EXISTS && (UART_CONSOLE_EXISTS || CONFIG_RTT_CONSOLE)
K_THREAD_DEFINE(console_thread_id, 2048, console_thread, NULL, NULL, NULL, CONSOLE_THREAD_PRIORITY, 0, 0);
#endif

#define DFU_EXISTS (CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER || CONFIG_BOOTLOADER_MCUBOOT)
#define ADAFRUIT_BOOTLOADER (CONFIG_BUILD_OUTPUT_UF2 && !CONFIG_BOOTLOADER_MCUBOOT)

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mag), okay)
#define SENSOR_MAG_EXISTS true
#endif

#if CONFIG_SENSOR_USE_SENS_CALIBRATION
#define SENS_CAL_DEFAULT_REVOLUTIONS CONFIG_SENSOR_SENS_REV
#define SENS_CAL_MAX_REVOLUTIONS     100
#endif

#define CONSOLE_BUTTON_EXISTS DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios)

static const char *meows[] = {
	"Mew", "Meww", "Meow", "Meow meow", "Mrrrp", "Mrrf", "Mreow", "Mrrrow", "Mrrr", "Purr",
	"mew", "meww", "meow", "meow meow", "mrrrp", "mrrf", "mreow", "mrrrow", "mrrr", "purr",
};

static const char *meow_punctuations[] = {".", "?", "!", "-", "~", ""};

static const char *meow_suffixes[]
	= {" :3", " :3c", " ;3", " ;3c", " x3", " x3c", " X3", " X3c", " >:3", " >:3c", " >;3", " >;3c", ""};

int console_serial_start(void)
{
#if USB_EXISTS || UART_CONSOLE_EXISTS
#if USB_EXISTS
	bool create_thread = false;
#endif

	int ret = console_input_install();
	if (ret != 0) {
		return ret;
	}

	k_spinlock_key_t key = k_spin_lock(&console_input.lock);
	if (console_input.active) {
		k_spin_unlock(&console_input.lock, key);
		return 0;
	}

	uart_irq_rx_disable(console_uart_dev);
	uart_irq_tx_disable(console_uart_dev);
	console_drain_uart_locked();
	console_input.active = true;
	console_reset_line_locked();
	console_input.echo_head = 0;
	console_input.echo_tail = 0;
#if USB_EXISTS
	if (!console_thread_started) {
		console_thread_started = true;
		create_thread = true;
	}
#endif
	uart_irq_rx_enable(console_uart_dev);
	k_spin_unlock(&console_input.lock, key);

#if USB_EXISTS
	if (create_thread) {
		k_thread_create(
			&console_thread_id,
			console_thread_stack,
			K_THREAD_STACK_SIZEOF(console_thread_stack),
			(k_thread_entry_t)console_thread,
			NULL,
			NULL,
			NULL,
			CONSOLE_THREAD_PRIORITY,
			0,
			K_NO_WAIT
		);
	}
#endif
	printk("*** " CONFIG_SLIMEVR_USB_DEVICE_MANUFACTURER " " CONFIG_SLIMEVR_USB_DEVICE_PRODUCT " ***\n");
	printk(FW_STRING);
	printk("Repo: %s | Branch: %s\n", FW_GIT_REPO_URL, FW_GIT_BRANCH);
	printk("Type 'help' to show available commands.\n");
	return 0;
#else
	return 0;
#endif
}

/* USB callers serialize transitions; IRQ/editor state uses its own lock. */
static void console_serial_end(bool invalidate)
{
#if USB_EXISTS || UART_CONSOLE_EXISTS
	k_spinlock_key_t key = k_spin_lock(&console_input.lock);
	console_input.active = false;
	if (invalidate) {
		console_input.epoch++;
		console_drop_queued_lines_locked();
	}
	console_reset_line_locked();
	console_input.echo_head = 0;
	console_input.echo_tail = 0;
	if (console_input.initialized) {
		uart_irq_tx_disable(console_uart_dev);
		uart_irq_rx_disable(console_uart_dev);
		console_drain_uart_locked();
	}
	k_spin_unlock(&console_input.lock, key);
#else
	(void)invalidate;
#endif
}

void console_serial_close(void)
{
	console_serial_end(false);
}

void console_serial_stop(void)
{
	console_serial_end(true);
}

static void print_board(void)
{
#if USB_EXISTS
	printk(CONFIG_SLIMEVR_USB_DEVICE_MANUFACTURER " " CONFIG_SLIMEVR_USB_DEVICE_PRODUCT "\n");
#endif
	printk(FW_STRING);
	printk("Repo: %s | Branch: %s\n", FW_GIT_REPO_URL, FW_GIT_BRANCH);

	printk("\nBoard: " CONFIG_BOARD "\n");
	printk("SOC: " CONFIG_SOC "\n");
	printk("Target: " CONFIG_BOARD_TARGET "\n");
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

static void print_sensor_identity(void)
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
}

static void print_odr_summary_line(void)
{
	float gyro_hz = sensor_get_gyro_odr();
	float accel_hz = sensor_get_accel_odr();
	float mag_hz = sensor_get_mag_odr();
	float fusion_hz = sensor_get_fusion_rate();

	if (mag_hz > 0.0f) {
		printk(
			"\nODR: gyro %.1fHz / accel %.1fHz / mag %.1fHz | fusion %.1fHz\n",
			(double)gyro_hz,
			(double)accel_hz,
			(double)mag_hz,
			(double)fusion_hz
		);
	} else {
		printk(
			"\nODR: gyro %.1fHz / accel %.1fHz / mag n/a | fusion %.1fHz\n",
			(double)gyro_hz,
			(double)accel_hz,
			(double)fusion_hz
		);
	}
}

static void print_mag_calibration_status(void)
{
	struct online_mag_diagnostics status;
	sensor_calibration_online_mag_diagnostics(&status);
	printk("Online mag debug: %s (runtime only)\n", sensor_calibration_get_online_mag_debug() ? "on" : "off");
	printk(
		"Calibration: %s (live, norm_cv=%.3f)\n",
		status.trial ? "trial" : (status.has_model ? "active" : "none"),
		(double)sensor_calibration_get_mag_quality()
	);
	if (!sensor_calibration_get_online_mag_enabled()) {
		printk("Online: disabled\n");
		return;
	}
	float dir_bias;
	int samples = sensor_calibration_online_mag_status(&dir_bias);
	static const char *const phases[] = {
		[TRAINING] = "collecting",
		[FREEZE_REQUESTED] = "freeze-requested",
		[FROZEN] = "fitting",
		[VALIDATION_READY] = "validation-ready",
		[VALIDATING] = "validating",
		[PROBATION] = "probation",
		[CONFIRMATION_READY] = "confirmation-ready",
	};
	static const char *const outcomes[] = {
		[ONLINE_MAG_NONE] = "pending",
		[ONLINE_MAG_UNCHANGED] = "unchanged",
		[ONLINE_MAG_ENVIRONMENT] = "environment-reference",
		[ONLINE_MAG_UPDATED] = "calibration-updated",
		[ONLINE_MAG_REJECTED] = "rejected",
	};
	static const char *const rejections[] = {
		[ONLINE_MAG_REJECT_NONE] = "none",
		[ONLINE_MAG_REJECT_FIT] = "fit",
		[ONLINE_MAG_REJECT_RADIAL] = "radial",
		[ONLINE_MAG_REJECT_DIP] = "dip",
		[ONLINE_MAG_REJECT_COVERAGE] = "coverage",
		[ONLINE_MAG_REJECT_TIMEOUT] = "timeout",
		[ONLINE_MAG_REJECT_CANCELLED] = "cancelled",
		[ONLINE_MAG_REJECT_MATRIX] = "matrix",
		[ONLINE_MAG_REJECT_SAMPLE] = "sample",
		[ONLINE_MAG_REJECT_OVERFLOW] = "overflow",
		[ONLINE_MAG_REJECT_NO_BENEFIT] = "no-benefit",
	};
	printk("Online: enabled, %d training samples, dir_bias=%.2f\n", samples, (double)dir_bias);
	printk(
		"  Phase: %s; last result: %s; rejection: %s; fit_errno=%d\n",
		status.phase < ARRAY_SIZE(phases) ? phases[status.phase] : "unknown",
		status.outcome < ARRAY_SIZE(outcomes) ? outcomes[status.outcome] : "unknown",
		status.rejection < ARRAY_SIZE(rejections) ? rejections[status.rejection] : "unknown",
		status.fit_errno
	);
	printk("  Last gate: %s\n", status.last_gate < ARRAY_SIZE(rejections) ? rejections[status.last_gate] : "unknown");
	if (status.score_valid) {
		printk(
			"  Evidence (%s, %u ms): radial samples=%u cells=%u; gravity samples=%u cells=%u; old_rms=%.4f "
			"new_rms=%.4f\n",
			status.score_phase < ARRAY_SIZE(phases) ? phases[status.score_phase] : "unknown",
			(unsigned)status.phase_age_ms,
			status.radial_count,
			status.radial_cells,
			status.dip_count,
			status.dip_cells,
			(double)status.old_rms,
			(double)status.new_rms
		);
		printk(
			"  Poles: radial=0x%02x gravity=0x%02x (all=0x3f); worst_cell_rms=%.4f max_error=%.4f\n",
			status.radial_poles,
			status.dip_poles,
			(double)status.worst_cell_rms,
			(double)status.max_radial_error
		);
		printk(
			"  Dip: old_sd=%.2f deg new_sd=%.2f deg window_delta=%.2f deg\n",
			(double)(status.old_dip_sd * 57.2957795f),
			(double)(status.new_dip_sd * 57.2957795f),
			(double)(status.dip_delta * 57.2957795f)
		);
	} else {
		printk(
			"  Evidence: no scored holdout; holdout_age=%u ms triggering_error=%.4f\n",
			(unsigned)status.phase_age_ms,
			(double)status.max_radial_error
		);
	}
}

static void print_sensor_summary(void)
{
	sensor_imu_calibration_t calibration;
	sensor_calibration_snapshot(&calibration);
	print_sensor_identity();
	print_odr_summary_line();

	printk(
		"Gyroscope bias: %.5f %.5f %.5f\n",
		(double)calibration.gyro_bias[0],
		(double)calibration.gyro_bias[1],
		(double)calibration.gyro_bias[2]
	);
#if CONFIG_SENSOR_USE_TCAL
	float current_gyro_offset[3];
	sensor_calibration_get_last_gyro_offset(current_gyro_offset);
	printk(
		"T-Cal: flag=%s apply=%s points=%u\n",
		sensor_tcal_get_enabled() ? "on" : "off",
		sensor_tcal_get_apply_mode_name(),
		retained->tempCalState.count
	);
	printk(
		"Gyroscope bias tcal (real-time): %.5f %.5f %.5f at %.2f C\n",
		(double)current_gyro_offset[0],
		(double)current_gyro_offset[1],
		(double)current_gyro_offset[2],
		(double)sensor_get_current_imu_temperature()
	);
#endif
	print_sens_calibration_info();
	printk("\nFusion: %s\n", sensor_get_sensor_fusion_name());
}

static void print_sensor_detail(void)
{
	sensor_imu_calibration_t calibration;
	sensor_calibration_snapshot(&calibration);
	printk("=== Sensor detail ===\n");
	printk(
		"IMU: %s | Mag: %s (%s)\n",
		(retained->imu_addr & 0x7F) != 0x7F ? sensor_get_sensor_imu_name() : "Not searching",
		(retained->mag_addr & 0x7F) != 0x7F ? sensor_get_sensor_mag_name() : "Not searching",
		sensor_get_mag_enabled() ? "enabled" : "disabled"
	);

	float mag_hz = sensor_get_mag_odr();
	float mag_feed_hz = sensor_get_mag_feed_hz();
	float work_time_ms = sensor_get_processing_work_time_ms();
	printk("\nRates:\n");
	printk("  Gyro ODR:    %.2f Hz\n", (double)sensor_get_gyro_odr());
	printk("  Accel ODR:   %.2f Hz\n", (double)sensor_get_accel_odr());
	if (mag_hz > 0.0f) {
		printk("  Mag ODR:     %.2f Hz\n", (double)mag_hz);
	} else {
		printk("  Mag ODR:     n/a\n");
	}
	if (mag_feed_hz > 0.0f) {
		printk("  Mag feed:    ~%.1f Hz\n", (double)mag_feed_hz);
	} else if (mag_hz > 0.0f && sensor_get_mag_enabled()) {
		printk("  Mag feed:    measuring...\n");
	}
	printk("  Fusion rate: %.2f Hz\n", (double)sensor_get_fusion_rate());
#if CONFIG_SENSOR_GYRO_OVERSAMPLING > 1 || CONFIG_SENSOR_ACCEL_OVERSAMPLING > 1
	printk(
		"  Oversample:  gyro %dx, accel %dx\n",
		CONFIG_SENSOR_GYRO_OVERSAMPLING,
		CONFIG_SENSOR_ACCEL_OVERSAMPLING
	);
#endif
	if (work_time_ms > 0.0f) {
		printk("  Work time:   ~%.1f ms/loop\n", (double)work_time_ms);
	} else {
		printk("  Work time:   n/a\n");
	}
	printk(
		"  Test mode:   %s (target=%u, effective=%u TPS)\n",
		test_mode_get() ? "enabled" : "disabled",
		test_mode_get_target_tps(),
		test_mode_effective_tps()
	);

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	printk("\nAccelerometer matrix:\n");
	for (int i = 0; i < 3; i++) {
		printk(
			"%.5f %.5f %.5f %.5f\n",
			(double)calibration.accel_matrix[0][i],
			(double)calibration.accel_matrix[1][i],
			(double)calibration.accel_matrix[2][i],
			(double)calibration.accel_matrix[3][i]
		);
	}

	printk("\nAccel calibration:\n");
	printk(
		"  Offset: [%.5f, %.5f, %.5f]\n",
		(double)calibration.accel_matrix[0][0],
		(double)calibration.accel_matrix[0][1],
		(double)calibration.accel_matrix[0][2]
	);
	float diag_x = calibration.accel_matrix[1][0];
	float diag_y = calibration.accel_matrix[2][1];
	float diag_z = calibration.accel_matrix[3][2];
	printk("  Scale: [%.5f, %.5f, %.5f]\n", (double)diag_x, (double)diag_y, (double)diag_z);
#else
	printk(
		"\nAccelerometer bias: %.5f %.5f %.5f\n",
		(double)calibration.accel_bias[0],
		(double)calibration.accel_bias[1],
		(double)calibration.accel_bias[2]
	);
#endif
	printk("Magnetometer matrix (stored):\n");
	for (int i = 0; i < 3; i++) {
		printk(
			"%.5f %.5f %.5f %.5f\n",
			(double)retained->magBAinv[0][i],
			(double)retained->magBAinv[1][i],
			(double)retained->magBAinv[2][i],
			(double)retained->magBAinv[3][i]
		);
	}
	print_mag_calibration_status();

#if CONFIG_SENSOR_RANGE_STATS
	const sensor_range_stats_t *stats = sensor_get_range_stats();
	if (stats->initialized) {
		float gyro_peak = 0;
		float accel_peak = 0;
		for (int i = 0; i < 3; i++) {
			float g_peak = fmaxf(fabsf(stats->gyro_min[i]), fabsf(stats->gyro_max[i]));
			float a_peak = fmaxf(fabsf(stats->accel_min[i]), fabsf(stats->accel_max[i]));
			if (g_peak > gyro_peak) {
				gyro_peak = g_peak;
			}
			if (a_peak > accel_peak) {
				accel_peak = a_peak;
			}
		}
		printk("\nRuntime range peaks (this session):\n");
		printk("  Gyro: %.2f deg/s\n", (double)gyro_peak);
		printk("  Accel: %.3f g\n", (double)accel_peak);
		printk("  Samples: %llu (use 'range' for details)\n", stats->sample_count);
	}
#endif // CONFIG_SENSOR_RANGE_STATS
}

static void print_connection(void)
{
	bool paired = retained->paired_addr[0];
	printk(paired ? "Tracker ID: %u\n" : "\nTracker ID: None\n", retained->paired_addr[1]);
	printk("Device address: %012llX\n", *(uint64_t *)NRF_FICR->DEVICEADDR & 0xFFFFFFFFFFFF);
	printk(
		paired ? "Receiver address: %012llX\n" : "Receiver address: None\n",
		(*(uint64_t *)&retained->paired_addr[0] >> 16) & 0xFFFFFFFFFFFF
	);

	// Display RF channel info (stored value is encoded)
	uint8_t rf_ch = esb_rf_channel_decode(retained->rf_channel);
	if (rf_ch != ESB_RF_CHANNEL_DEFAULT) {
		printk("RF Channel: %u (custom)\n", rf_ch);
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
	print_sensor_summary();
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

static void print_button_help(void)
{
	printk("Button Functions (current build):\n");
#if CONSOLE_BUTTON_EXISTS
	printk("  Short press (1x):          Reboot (blocked in test mode)\n");
#if CONFIG_USER_EXTRA_ACTIONS
	printk("  Quick press (2x):          Calibrate sensor ZRO\n");
	printk("  Quick press (3x):          Reset active pairing\n");
#if DFU_EXISTS
#if defined(CONFIG_BOARD_STYRIA_MINI_UF2)
	printk("  Quick press (6x/7x):       Enter DFU bootloader\n");
#else
	printk("  Quick press (4x/5x):       Enter DFU bootloader\n");
#endif
#if ADAFRUIT_BOOTLOADER
	printk("  Quick press (8x/9x):       Enter OTA DFU (BLE) if flashed SD (softdevice) firmware\n");
#else
	printk("  Quick press (8x/9x):       Enter DFU bootloader\n");
#endif
#endif
#if USER_SHUTDOWN_ENABLED
	printk("  Hold (~1s):                Power off; keep holding ~5s to cancel\n");
#else
	printk("  Hold (~1s):                Reboot; keep holding ~5s to cancel\n");
#endif
#else
#if USER_SHUTDOWN_ENABLED
	printk("  Hold (~1s):                Power off; keep holding ~5s to reset pairing\n");
#else
	printk("  Hold (~1s):                Reboot; keep holding ~5s to reset pairing\n");
#endif
#endif
#if USB_EXISTS && DFU_EXISTS
	printk("  Hold while USB connects:   Enter DFU bootloader\n");
#endif
	printk("  During OTA:                Button actions are blocked\n");
#else
	printk("  No sw0 button is defined for this board\n");
#endif
	printk("\n");
}

static void print_help(void)
{
	printk("\n=== Available Commands ===\n\n");
	printk("Device Information:\n");
	printk("  info                       Get device information\n");
	printk("  sensor                     Get sensor rates and calibration detail\n");
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
	printk("  mag auto on|off     Enable/disable online magnetometer calibration\n");
	printk("  mag debug on|off           Toggle online calibration logs (runtime only)\n");
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
	printk("  radio <on|off>             Stop/restart ESB radio (diagnostic A/B for IMU noise)\n");
	printk("\n");
	printk("  channel <0-100>            Set RF channel (saved to NVS)\n");
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
#if CONFIG_THREAD_ANALYZER
	printk("  stack                      Print thread and ISR stack high-water usage\n");
#endif
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
	print_button_help();
}

// --- Command Implementations ---

#if CONFIG_SENSOR_USE_SENS_CALIBRATION
static void cmd_sens_set(float x, float y, float z)
{
	const float degrees[3] = {x, y, z};
	int err = sensor_calibration_set_sensitivity(degrees);
	if (err) {
		printk("Error: Sensitivity update failed: %d (RAM may already be updated).\n", err);
	} else {
		printk("Gyro sensitivity difference set to: %.3f, %.3f, %.3f\n",
			(double)x, (double)y, (double)z);
	}
}

static void cmd_sens_reset(void)
{
	int err = sensor_calibration_reset_sensitivity();
	if (err) {
		printk("Error: Sensitivity reset failed: %d (RAM may already be updated).\n", err);
	} else {
		printk("Gyro sensitivity reset.\n");
	}
}

static void cmd_sens_auto(const char *axis_str, const char *rev_str)
{
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

	int err = sensor_request_calibration_sens(axis, revolutions);
	if (err) {
		printk("Error: Calibration request rejected: %d.\n", err);
		return;
	}
	char axis_char = "XYZ"[axis];
	printk("Gyro sensitivity auto-calibration started on %c axis (%u rev).\n", axis_char, revolutions);
	printk("  1. Hold the tracker still until the LED flashes.\n");
	printk("  2. While flashing, spin it %u full turns about the %c axis, then stop.\n", revolutions, axis_char);
}
#endif

static void cmd_reset_zro(void)
{
	int err = sensor_calibration_reset_imu();
	if (err) {
		printk("Error: IMU calibration reset rejected: %d.\n", err);
	}
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static void cmd_reset_acc(void)
{
	int err = sensor_calibration_reset_accel();
	if (err) {
		printk("Error: Accelerometer calibration reset rejected: %d.\n", err);
	}
}
#endif

#if CONFIG_SENSOR_USE_TCAL
static void cmd_reset_tcal(void)
{
	sensor_tcal_clear();
}
#endif

static void cmd_reset_bat(void)
{
	sys_reset_battery_tracker();
}

static void cmd_fusion_reset(void)
{
	printk("Resetting fusion (invalidating quaternion).\n");
	sensor_request_fusion_reset();
	printk("Fusion reset requested.\n");
}

static void cmd_ping_start(void)
{
	printk("Ping received! Flashing LED.\n");
	set_led(SYS_LED_PATTERN_ONESHOT_PING, SYS_LED_PRIORITY_HIGHEST);
}

static void cmd_shutdown(void)
{
	int err = sys_command_shutdown();
	if (err < 0) {
		printk("Shutdown request rejected: %d\n", err);
	} else {
		printk("Shutdown request accepted.\n");
	}
}

static inline void strtolower(char *str)
{
	for (int i = 0; str[i] != '\0'; i++) {
		str[i] = (char)tolower((unsigned char)str[i]);
	}
}

typedef void (*console_cmd_fn)(size_t argc, char **argv);

struct console_cmd {
	const char *name;
	console_cmd_fn fn;
};

static void console_cmd_help(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	print_help();
}

static void console_cmd_info(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	print_info();
}

static void console_cmd_sensor(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	print_sensor_detail();
}

static void console_cmd_uptime(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	uint64_t uptime = k_uptime_ticks();
	print_uptime(uptime, "Uptime");
	print_uptime(uptime - retained->uptime_latest + retained->uptime_sum, "Accumulated");
}

static void console_cmd_shutdown(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	cmd_shutdown();
}

static void console_cmd_reboot(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	int err = sys_request_system_reboot();
	if (err) {
		printk("Error: Reboot request rejected: %d.\n", err);
	}
}

static void console_cmd_battery(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	print_battery_tracker();
}

static void console_cmd_scan(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	sensor_request_scan(true);
}

static void console_cmd_calibrate(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	sensor_request_calibration();
}

#if CONFIG_SENSOR_USE_SENS_CALIBRATION
static void console_cmd_sens(size_t argc, char **argv)
{
	char *arg = argc > 1 ? argv[1] : NULL;
	char *arg2 = argc > 2 ? argv[2] : NULL;
	char *arg3 = argc > 3 ? argv[3] : NULL;

	// check if there are any arguments at all.
	if (arg == NULL) {
		printk("Error: Missing arguments. Use 'sens <x>,<y>,<z>', 'sens auto <x|y|z> [rev]', or 'sens reset'.\n");
	}
	// check if this is the auto-calibration subcommand
	else if (strcmp(arg, "auto") == 0) {
		if (argc > 4) {
			printk("Error: Too many arguments. Use: 'sens auto <x|y|z> [revolutions]'.\n");
		} else {
			cmd_sens_auto(arg2, arg3);
		}
	}
	// check if the argument is "reset"
	else if (strcmp(arg, "reset") == 0) {
		cmd_sens_reset();
	} else {
		char *token;
		char *endptr;
		int token_count = 0;
		float values[3];

		token = strtok(arg, ",");
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
static void console_cmd_tcal(size_t argc, char **argv)
{
	char *arg = argc > 1 ? argv[1] : NULL;
	char *arg2 = argc > 2 ? argv[2] : NULL;

	// check if there are any arguments
	if (arg == NULL) {
		printk("Error: Missing argument. Use: tcal <on|off|status|clear|dump|test temp|remove index|check|auto on|auto off|boot [on|off]>\n");
	} else {
		char *subcmd = arg;

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
			printk(
				"T-Cal compensation: %s | apply=%s\n",
				sensor_tcal_get_enabled() ? "enabled" : "disabled",
				sensor_tcal_get_apply_mode_name()
			);
			printk("Auto-calibration: %s\n", sensor_tcal_get_auto_calibration() ? "enabled" : "disabled");
		} else if (strcmp(subcmd, "clear") == 0) {
			cmd_reset_tcal();
		} else if (strcmp(subcmd, "auto") == 0) {
			char *auto_arg = arg2;
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
				return;
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
			char *idx_str = arg2;

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
			char *temp_str = arg2;
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
				float closest_temp, distance_c;
				bool needs_cal = sensor_tcal_needs_nearby_point(current_temp, &closest_temp, &distance_c);
				printk("Current temperature: %.2fC\n", (double)current_temp);
				if (!isnan(closest_temp)) {
					printk("Closest calibration point: %.2fC (distance: %.2fC)\n",
						(double)closest_temp, (double)distance_c);
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
			char *boot_arg = arg2;
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
static void console_cmd_6_side(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	sensor_request_calibration_6_side();
}
#endif

static void console_cmd_mag(size_t argc, char **argv)
{
	char *arg = argc > 1 ? argv[1] : NULL;
	char *arg2 = argc > 2 ? argv[2] : NULL;
	char *arg3 = argc > 3 ? argv[3] : NULL;

	if (arg == NULL) {
		// No argument: show status
		printk("Magnetometer: %s\n", sensor_get_mag_enabled() ? "enabled" : "disabled");
		printk("Hardware: %s\n", sensor_get_sensor_mag_name());
		printk("Magnetometer matrix (stored):\n");
		for (int i = 0; i < 3; i++) {
			printk(
				"%.5f %.5f %.5f %.5f\n",
				(double)retained->magBAinv[0][i],
				(double)retained->magBAinv[1][i],
				(double)retained->magBAinv[2][i],
				(double)retained->magBAinv[3][i]
			);
		}
		print_mag_calibration_status();
	} else {
		char *subcmd = arg;
		if (subcmd == NULL) {
			printk("Usage: mag [on|off|clear|cal|auto <on|off>|debug <on|off>]\n");
		} else if (strcmp(subcmd, "on") == 0) {
			printk("Enabling magnetometer\n");
			sensor_set_mag_enabled(true);
		} else if (strcmp(subcmd, "off") == 0) {
			printk("Disabling magnetometer\n");
			sensor_set_mag_enabled(false);
		} else if (strcmp(subcmd, "auto") == 0 || strcmp(subcmd, "online") == 0) {
			char *state = arg2;
			char *extra = arg3;

			if (state == NULL || extra != NULL) {
				printk("Usage: mag %s <on|off>\n", subcmd);
			} else if (strcmp(state, "on") == 0) {
				sensor_calibration_set_online_mag_enabled(true);
				printk("Online magnetometer calibration enabled\n");
			} else if (strcmp(state, "off") == 0) {
				sensor_calibration_set_online_mag_enabled(false);
				printk("Online magnetometer calibration disabled\n");
			} else {
				printk("Usage: mag %s <on|off>\n", subcmd);
			}
		} else if (strcmp(subcmd, "debug") == 0) {
			if (argc != 3 || (strcmp(arg2, "on") != 0 && strcmp(arg2, "off") != 0)) {
				printk("Usage: mag debug <on|off>\n");
			} else {
				sensor_calibration_set_online_mag_debug(strcmp(arg2, "on") == 0);
				printk(
					"Online mag debug: %s (runtime only)\n",
					sensor_calibration_get_online_mag_debug() ? "on" : "off"
				);
			}
		} else if (strcmp(subcmd, "clear") == 0) {
			sensor_calibration_clear_mag(NULL, true);
			printk("Magnetometer calibration cleared\n");
		} else if (strcmp(subcmd, "cal") == 0 || strcmp(subcmd, "calibrate") == 0) {
			sensor_calibration_clear_mag(NULL, true);
			sensor_request_calibration_mag();
			printk("Magnetometer calibration started\n");
		} else {
			printk("Usage: mag [on|off|clear|cal|auto <on|off>|debug <on|off>]\n");
		}
	}
}

static void console_cmd_set(size_t argc, char **argv)
{
	char *arg = argc > 1 ? argv[1] : NULL;

	if (argc != 2) {
		printk("Invalid number of arguments\n");
		return;
	}
	uint64_t addr = parse_u64(arg, 16);
	char buf[17];
	snprintk(buf, 17, "%016llx", addr);
	if (addr != 0 && strcmp(buf, arg) == 0) {
		esb_set_pair(addr);
	} else {
		printk("Invalid address\n");
	}
}

static void console_cmd_pair(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	esb_reset_pair();
}

static void console_cmd_clear(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	esb_clear_pair();
}

static void console_cmd_channel(size_t argc, char **argv)
{
	char *arg = argc > 1 ? argv[1] : NULL;

	if (!arg) {
		printk("Usage: channel <0-100>\n");
		printk("Example: channel 25 - Set RF channel to 25\n");
	} else {
		char *endptr;
		long channel = strtol(arg, &endptr, 10);

		if (endptr == arg || *endptr != '\0' || channel < 0 || channel > 100) {
			printk("Invalid channel. Must be a number between 0 and 100.\n");
		} else {
			int err = channel_control_set((int)channel);
			if (err) {
				printk("Error: Channel update failed: %d (RAM/radio may already be updated).\n", err);
			} else {
				printk("RF channel saved to NVS: %d\n", (int)channel);
				printk("ESB reinitialized with channel %d\n", (int)channel);
			}
		}
	}
}

static void console_cmd_clearchannel(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	printk("Clearing RF channel setting (restore default)\n");
	int err = channel_control_reset();
	if (err) {
		printk("Error: Channel reset failed: %d (RAM/radio may already be updated).\n", err);
	} else {
		printk("ESB reinitialized with default channel\n");
	}
}

static void console_cmd_radio(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	char *arg = argc > 1 ? argv[1] : NULL;

	if (!arg) {
		printk("Usage: radio <on|off>\n");
		printk("Example: radio off - stop ESB radio for IMU noise A/B test\n");
		return;
	}

	if (strcmp(arg, "off") == 0) {
		esb_deinitialize();
		printk("ESB radio disabled; sensor loop keeps running (diagnostic only)\n");
	} else if (strcmp(arg, "on") == 0) {
		if (esb_reinitialize()) {
			printk("Error: ESB reinitialize failed\n");
		} else {
			printk("ESB radio reinitialized\n");
		}
	} else {
		printk("Invalid radio argument: %s (use on/off)\n", arg);
	}
}

#if DFU_EXISTS
static void console_cmd_dfu(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	char *arg = argc > 1 ? argv[1] : NULL;
	bool ota = false;

#if ADAFRUIT_BOOTLOADER
	// Subcommands:
	//   dfu      -> UF2 DFU (USB MSC/CDC)
	//   dfu ota  -> OTA DFU (BLE)
	char *mode = arg;

	if (mode && strcmp(mode, "ota") == 0) {
		ota = true;
		printk("Entering OTA DFU (BLE)...\n");
	} else if (mode == NULL) {
		printk("Entering UF2 DFU...\n");
	} else {
		printk("Error: Unknown DFU mode '%s'. Use: dfu [ota]\n", mode);
		return;
	}

#else
	if (arg != NULL) {
		printk("Error: This bootloader does not support a DFU mode argument\n");
		return;
	}
#endif

	printk("Entering DFU bootloader...\n");
	sys_enter_dfu(ota);
}
#endif

static void console_cmd_ping(size_t argc, char **argv)
{
	if (argc > 1 && strcmp(argv[1], "stats") == 0) {
		connection_print_ping_stats();
		return;
	}
	cmd_ping_start();
}

static void console_cmd_nvs(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	sys_nvs_stats();
}

#if CONFIG_THREAD_ANALYZER
static void console_cmd_stack(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	printk("Thread/ISR stack high-water usage:\n");
	thread_analyzer_print(0);
}
#endif

static void console_cmd_meow(size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	print_meow();
}

static void console_cmd_debug(size_t argc, char **argv)
{
	char *arg = argc > 1 ? argv[1] : NULL;

	uint32_t duration = 1; // Default 1 second
	if (arg) {
		char *endptr;
		long dur = strtol(arg, &endptr, 10);
		if (endptr != arg && *endptr == '\0' && dur >= 1 && dur <= 60) {
			duration = (uint32_t)dur;
		} else {
			printk("Invalid duration (1-60s). Using default 1 seconds.\n");
		}
	}
	sensor_debug_start(duration);
}

static void console_cmd_range(size_t argc, char **argv)
{
	char *arg = argc > 1 ? argv[1] : NULL;

#if CONFIG_SENSOR_RANGE_STATS
	if (arg && strcmp(arg, "reset") == 0) {
		sensor_reset_range_stats();
		printk("Sensor range statistics have been reset.\n");
	} else {
		sensor_print_range_stats();
	}
#else
	ARG_UNUSED(arg);
	printk("Sensor range statistics not enabled in configuration.\n");
#endif // CONFIG_SENSOR_RANGE_STATS
}

#if CONFIG_VQF_BENCH
static void console_cmd_vqfbench(size_t argc, char **argv)
{
	char *arg = argc > 1 ? argv[1] : NULL;

	uint32_t iterations = 1000;
	if (arg) {
		char *endptr;
		long parsed = strtol(arg, &endptr, 10);
		if (endptr != arg && *endptr == '\0' && parsed > 0 && parsed <= 20000) {
			iterations = (uint32_t)parsed;
		} else {
			printk("Invalid iteration count. Using default 1000.\n");
		}
	}
	vqf_run_benchmark(iterations);
}
#endif // CONFIG_VQF_BENCH

static void console_cmd_reset(size_t argc, char **argv)
{
	char *arg = argc > 1 ? argv[1] : NULL;

	if (arg && strcmp(arg, "zro") == 0) {
		cmd_reset_zro();
	}
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	else if (arg && strcmp(arg, "acc") == 0) {
		cmd_reset_acc();
	}
#endif
	else if (arg && strcmp(arg, "mag") == 0) {
		sensor_calibration_clear_mag(NULL, true);
	}
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
	else if (arg && strcmp(arg, "sens") == 0) {
		cmd_sens_reset();
	}
#endif
#if CONFIG_SENSOR_USE_TCAL
	else if (arg && strcmp(arg, "tcal") == 0) {
		cmd_reset_tcal();
	}
#endif
	else if (arg && strcmp(arg, "bat") == 0) {
		cmd_reset_bat();
	} else if (arg && strcmp(arg, "fusion") == 0) {
		cmd_fusion_reset();
	} else if (arg && strcmp(arg, "all") == 0) {
		sys_clear();
	} else {
		printk("Invalid argument\n");
	}
}

static void console_cmd_tdma(size_t argc, char **argv)
{
	char *arg = argc > 1 ? argv[1] : NULL;

	if (arg && strcmp(arg, "on") == 0) {
		tdma_set_enabled(true);
		printk("TDMA enabled\n");
	} else if (arg && strcmp(arg, "off") == 0) {
		tdma_set_enabled(false);
		printk("TDMA disabled\n");
	} else if (arg && strcmp(arg, "capture") == 0) {
#if defined(CONFIG_TDMA_DIAGNOSTICS)
		char *state = argc > 2 ? argv[2] : NULL;
		if (state && strcmp(state, "on") == 0) {
			radio_capture_set_enabled(true);
			printk("RADIO capture enabled\n");
		} else if (state && strcmp(state, "off") == 0) {
			radio_capture_set_enabled(false);
			printk("RADIO capture disabled\n");
		} else {
			printk("RADIO capture: %s\n", radio_capture_is_enabled() ? "enabled" : "disabled");
			radio_capture_print_stats();
		}
#else
		printk("tdma capture requires CONFIG_TDMA_DIAGNOSTICS=y\n");
#endif
	} else if (arg && strcmp(arg, "stats") == 0) {
#if defined(CONFIG_TDMA_DIAGNOSTICS)
		tdma_print_stats();
#else
		printk("tdma stats requires CONFIG_TDMA_DIAGNOSTICS=y\n");
#endif
	} else {
		printk("TDMA: %s\n", tdma_is_enabled() ? "enabled" : "disabled");
	}
}

static void console_cmd_test(size_t argc, char **argv)
{
	char *arg = argc > 1 ? argv[1] : NULL;

	if (arg && strcmp(arg, "on") == 0) {
		test_mode_set(true);
		printk("Test mode enabled\n");
	} else if (arg && strcmp(arg, "off") == 0) {
		test_mode_set(false);
		printk("Test mode disabled\n");
	} else {
		printk("Test mode: %s\n", test_mode_get() ? "enabled" : "disabled");
	}
}

static const struct console_cmd console_cmds[] = {
	{"help", console_cmd_help},
	{"info", console_cmd_info},
	{"sensor", console_cmd_sensor},
	{"uptime", console_cmd_uptime},
	{"shutdown", console_cmd_shutdown},
	{"reboot", console_cmd_reboot},
	{"battery", console_cmd_battery},
	{"scan", console_cmd_scan},
	{"calibrate", console_cmd_calibrate},
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
	{"sens", console_cmd_sens},
#endif
#if CONFIG_SENSOR_USE_TCAL
	{"tcal", console_cmd_tcal},
#endif
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	{"6-side", console_cmd_6_side},
#endif
	{"mag", console_cmd_mag},
	{"set", console_cmd_set},
	{"pair", console_cmd_pair},
	{"clear", console_cmd_clear},
	{"channel", console_cmd_channel},
	{"clearchannel", console_cmd_clearchannel},
	{"radio", console_cmd_radio},
#if DFU_EXISTS
	{"dfu", console_cmd_dfu},
#endif
	{"ping", console_cmd_ping},
	{"nvs", console_cmd_nvs},
	{"meow", console_cmd_meow},
	{"debug", console_cmd_debug},
#if CONFIG_THREAD_ANALYZER
	{"stack", console_cmd_stack},
#endif
	{"range", console_cmd_range},
#if CONFIG_VQF_BENCH
	{"vqfbench", console_cmd_vqfbench},
#endif
	{"reset", console_cmd_reset},
	{"tdma", console_cmd_tdma},
	{"test", console_cmd_test},
};

static void console_thread(void)
{
#if UART_CONSOLE_EXISTS
	if (console_serial_start() != 0) {
		return;
	}
#elif !USB_EXISTS
	printk(FW_STRING);
	printk("Repo: %s | Branch: %s\n", FW_GIT_REPO_URL, FW_GIT_BRANCH);
	printk("Type 'help' to show available commands.\n");
#endif

	while (1) {
#if USB_EXISTS || UART_CONSOLE_EXISTS
		struct console_line_message message;
		if (k_msgq_get(&console_line_msgq, &message, K_FOREVER) != 0 || !console_line_is_current(message.epoch)) {
			continue;
		}
		char *line = message.line;
#else
		char *line = rtt_console_getline();
#endif
		char *argv[8] = {NULL};
		size_t argc = parse_args(line, argv, ARRAY_SIZE(argv));
		if (argc == 0) {
			continue;
		}
		for (size_t i = 0; i < argc; i++) {
			strtolower(argv[i]);
		}

		bool found = false;
		for (size_t i = 0; i < ARRAY_SIZE(console_cmds); i++) {
			if (strcmp(argv[0], console_cmds[i].name) == 0) {
				console_cmds[i].fn(argc, argv);
				found = true;
				break;
			}
		}
		if (!found) {
			printk("Unknown command\n");
		}
	}
}

#endif
