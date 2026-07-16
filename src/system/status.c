#include "globals.h"
#include "connection/connection.h"

#include <zephyr/kernel.h>

#include "status.h"
#include "led.h"

static int status_state = 0;

#define STATUS_LED_ERROR_MASK \
	(SYS_STATUS_SENSOR_ERROR | SYS_STATUS_CONNECTION_ERROR | SYS_STATUS_SYSTEM_ERROR)

static K_SEM_DEFINE(status_wake_sem, 0, 1);

LOG_MODULE_REGISTER(status, LOG_LEVEL_INF);

static void status_thread(void);
K_THREAD_DEFINE(status_thread_id, 256, status_thread, NULL, NULL, NULL, 6, 0, 0);

void set_status(enum sys_status status, bool set) {
	int before = status_state & STATUS_LED_ERROR_MASK;
	if (set) {
		status_state |= status;
		switch (status)
		{
		case SYS_STATUS_SENSOR_ERROR:
			LOG_ERR("Sensor communication error");
			break;
		case SYS_STATUS_CONNECTION_ERROR:
			LOG_WRN("Connection error");
			break;
		case SYS_STATUS_SYSTEM_ERROR:
			LOG_ERR("General error");
			break;
		case SYS_STATUS_USB_CONNECTED:
			LOG_INF("USB connected");
			break;
		case SYS_STATUS_PLUGGED:
			LOG_INF("Charger plugged");
			break;
		case SYS_STATUS_CALIBRATION_RUNNING:
			LOG_INF("Calibration running");
			break;
		case SYS_STATUS_BUTTON_PRESSED:
			LOG_INF("Button pressed");
			break;
		default:
			break;
		}
	} else {
		status_state &= ~status;
		LOG_INF("Cleared status: %d", status);
	}
	int after = status_state & STATUS_LED_ERROR_MASK;
	if (before != after) {
		k_sem_give(&status_wake_sem);
	}
	connection_update_status(status_state);
	LOG_INF("Status: %d", status_state);
}

int get_status(enum sys_status status)
{
	return status_state & status;
}

static void status_thread(void)
{
	while (1) // cycle through errors
	{
		int status = status_state & STATUS_LED_ERROR_MASK;
		if (status & SYS_STATUS_SENSOR_ERROR) {
			set_led(SYS_LED_PATTERN_ERROR_A, SYS_LED_PRIORITY_STATUS);
			(void)k_sem_take(&status_wake_sem, K_MSEC(5000));
		}
		if (status & SYS_STATUS_CONNECTION_ERROR) {
			set_led(SYS_LED_PATTERN_ERROR_B, SYS_LED_PRIORITY_STATUS);
			(void)k_sem_take(&status_wake_sem, K_MSEC(5000));
		}
		if (status & SYS_STATUS_SYSTEM_ERROR) {
			set_led(SYS_LED_PATTERN_ERROR_C, SYS_LED_PRIORITY_STATUS);
			(void)k_sem_take(&status_wake_sem, K_MSEC(5000));
		}
		if (!status) {
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_STATUS);
			(void)k_sem_take(&status_wake_sem, K_FOREVER);
		}
	}
}

bool status_ready(void)  // true if no important statuses are active
{
	return (status_state & ~SYS_STATUS_CONNECTION_ERROR)
		== 0;  // connection error is temporary, not critical
}
