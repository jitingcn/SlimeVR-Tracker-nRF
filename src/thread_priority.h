/* Negative priority is cooperative
 Non-negative priority is preemptible (such as during any k_sleep)
 Higher value is lower priority
*/
#define CONSOLE_THREAD_PRIORITY 6
#define HID_THREAD_PRIORITY 6
#define USB_INIT_THREAD_PRIORITY 6
#define CONNECTION_THREAD_PRIORITY 4
#define ESB_THREAD_PRIORITY 5
#define CLOCKS_START_THREAD_PRIORITY 5
#define CLOCKS_STOP_THREAD_PRIORITY 5
#define CLOCK_INIT_THREAD_PRIORITY 8
#define CALIBRATION_THREAD_PRIORITY 8
#define SENSOR_REQUEST_SCAN_THREAD_PRIORITY 7
#define SENSOR_SCAN_THREAD_PRIORITY 7
#define SENSOR_LOOP_THREAD_PRIORITY 7
#define LED_THREAD_PRIORITY 6
#define DISABLE_DFU_THREAD_PRIORITY 6
/* Below the sensor loop (7): the battery poll bitbangs the nPM1300 over
 * gpio-i2c (busy-wait), which would otherwise stall the sensor loop. */
#define POWER_THREAD_PRIORITY 9
#define STATUS_THREAD_PRIORITY 6
#define BUTTON_THREAD_PRIORITY 6
#define VQF_BENCH_THREAD_PRIORITY 8
