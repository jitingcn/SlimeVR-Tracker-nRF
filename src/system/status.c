#include "globals.h"
#include "connection/connection.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "status.h"
#include "led.h"

static atomic_t status_state;
static struct k_spinlock status_lock;
static enum sys_sensor_fault status_sensor_fault;

#define STATUS_LED_ERROR_MASK (SYS_STATUS_SENSOR_ERROR | SYS_STATUS_SYSTEM_ERROR)

static K_SEM_DEFINE(status_wake_sem, 0, 1);

LOG_MODULE_REGISTER(status, LOG_LEVEL_INF);

static void status_thread(void);
K_THREAD_DEFINE(status_thread_id, CONFIG_STATUS_THREAD_STACK_SIZE, status_thread, NULL, NULL, NULL, STATUS_THREAD_PRIORITY, 0, 0);

static void status_update(enum sys_status status, bool set, enum sys_sensor_fault fault, bool replace_fault)
{
	k_spinlock_key_t key = k_spin_lock(&status_lock);
	int before = atomic_get(&status_state) & STATUS_LED_ERROR_MASK;
	enum sys_sensor_fault before_fault = status_sensor_fault;
	if (status & SYS_STATUS_SENSOR_ERROR) {
		if (!set) {
			status_sensor_fault = SYS_SENSOR_FAULT_NONE;
		} else if (replace_fault || status_sensor_fault == SYS_SENSOR_FAULT_NONE) {
			status_sensor_fault = fault;
		}
	}
	if (set) {
		atomic_or(&status_state, status);
	} else {
		atomic_and(&status_state, ~status);
	}
	int snapshot = atomic_get(&status_state);
	bool fault_changed = before_fault != status_sensor_fault;
	/* Leaf callback: only scalar stores and pure status mapping, no locks or
	 * callbacks. Publish under the state lock so an older writer cannot win.
	 * Keep logging and LED wakeups outside this IRQ-safe critical section. */
	connection_update_status(snapshot);
	k_spin_unlock(&status_lock, key);

	if (set) {
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
		case SYS_STATUS_SERIAL_ACTIVE:
			LOG_INF("Serial connected");
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
		LOG_INF("Cleared status: %d", status);
	}
	int after = snapshot & STATUS_LED_ERROR_MASK;
	if (before != after || fault_changed) {
		k_sem_give(&status_wake_sem);
	}
	LOG_INF("Status: %d", snapshot);
}

void set_status(enum sys_status status, bool set)
{
	status_update(status, set, SYS_SENSOR_FAULT_OTHER, false);
}

void set_sensor_fault(enum sys_sensor_fault fault)
{
	status_update(SYS_STATUS_SENSOR_ERROR, fault != SYS_SENSOR_FAULT_NONE, fault, true);
}

void set_sensor_fault_if_unset(enum sys_sensor_fault fault)
{
	if (fault != SYS_SENSOR_FAULT_NONE) {
		status_update(SYS_STATUS_SENSOR_ERROR, true, fault, false);
	}
}

int get_status(enum sys_status status)
{
	return atomic_get(&status_state) & status;
}

static void status_publish_faults(void)
{
	k_spinlock_key_t key = k_spin_lock(&status_lock);
	int status = atomic_get(&status_state) & STATUS_LED_ERROR_MASK;
	enum sys_sensor_fault sensor_fault = status_sensor_fault;
	k_spin_unlock(&status_lock, key);

	enum led_fault_kind fault = LED_FAULT_NONE;
	if ((status & SYS_STATUS_SENSOR_ERROR) != 0) {
		fault = sensor_fault == SYS_SENSOR_FAULT_MISSING ? LED_FAULT_SENSOR_MISSING : LED_FAULT_SENSOR;
	}
	led_fault_publish(LED_OWNER_SENSOR, fault, 0);
	led_fault_publish(LED_OWNER_SYSTEM, (status & SYS_STATUS_SYSTEM_ERROR) != 0 ? LED_FAULT_SYSTEM : LED_FAULT_NONE, 0);
}

static void status_thread(void)
{
	while (1) {
		status_publish_faults();
		(void)k_sem_take(&status_wake_sem, K_FOREVER);
	}
}

bool status_ready(void)  // true if no important statuses are active
{
	return (atomic_get(&status_state) & ~SYS_STATUS_CONNECTION_ERROR)
		== 0;  // connection error is temporary, not critical
}
