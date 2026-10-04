#ifndef SLIMENRF_SYSTEM_STATUS
#define SLIMENRF_SYSTEM_STATUS

enum sys_status {
	SYS_STATUS_SENSOR_ERROR = 1,
	SYS_STATUS_CONNECTION_ERROR = 2,
	SYS_STATUS_SYSTEM_ERROR = 4,
	SYS_STATUS_USB_CONNECTED = 8,
	SYS_STATUS_PLUGGED = 16,
	SYS_STATUS_CALIBRATION_RUNNING = 32,
	SYS_STATUS_BUTTON_PRESSED = 64,
	SYS_STATUS_SERIAL_ACTIVE = 128,

	SYS_STATUS_WARN = SYS_STATUS_CONNECTION_ERROR,
	SYS_STATUS_ERROR = SYS_STATUS_SENSOR_ERROR | SYS_STATUS_SYSTEM_ERROR,
	SYS_STATUS_ALL = 255,
};

/* Local presentation cause; SYS_STATUS_SENSOR_ERROR remains the sole host bit. */
enum sys_sensor_fault {
	SYS_SENSOR_FAULT_NONE,
	SYS_SENSOR_FAULT_MISSING,
	SYS_SENSOR_FAULT_OTHER,
};

void set_status(enum sys_status status, bool set);

/* Exact cause updates and successful-scan clearing are one status transition. */
void set_sensor_fault(enum sys_sensor_fault fault);

/* A generic initialization failure must not replace a specific scan failure. */
void set_sensor_fault_if_unset(enum sys_sensor_fault fault);

int get_status(enum sys_status status);

bool status_ready(void);

#endif