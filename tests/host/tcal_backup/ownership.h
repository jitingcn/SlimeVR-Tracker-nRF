#pragma once
/* Hardware/event leaves only. Ownership, request admission, saver and reset
 * generations are compiled verbatim from production below. */
/* Collector constants and ownership are extracted from the actual sources. */
#define MAX(a,b) ((a)>(b)?(a):(b))
static atomic_t tcal_auto_calibration_enabled;
static float sensor_get_gyro_odr(void) { return 100.0f; }
static bool sensor_peek_accel(float a[3]) {
    a[0] = a[1] = 0.0f; a[2] = 1.0f;
    return true;
}
K_MUTEX_DEFINE(calibration_request_lock);
static int requested_calibration;
static unsigned requested_operation, requested_generation;
static uint8_t magneto_progress;
static bool mag_cal_led_pending;
static atomic_t calibration_kind;
struct led_token { unsigned session; };
static struct led_token requested_feedback;
#define CAL_REASON_BUSY 1
#define CAL_EVENT_ORIGIN_AUTO 128
#define TCAL_DIR_RISING 1
#define TCAL_DIR_FALLING 2
#define TCAL_HYSTERESIS_EMA_RISING 0.7f
#define TCAL_HYSTERESIS_EMA_FALLING 0.15f
#define TCAL_HYSTERESIS_EMA_UNKNOWN 0.5f
#define TCAL_SAVE_SIGNIFICANCE_THRESHOLD 0.002f
static unsigned warm_marks;
static void sys_write_warm(uint16_t id, const void *src, void *dst, size_t size) {
    (void)id; (void)src; (void)dst; (void)size;
    assert(warm_transaction && !lock_depth); warm_marks++;
}
static uint8_t calibration_request_kind(int id) { return id > 0 && id < 8 ? 1 : 0; }
static void sensor_calibration_samples_end(void) {}
static int calibration_led_owner(int id) { return id; }
static void cal_event_reject(int kind, int reason) { (void)kind; (void)reason; }
static unsigned cal_event_accept(int kind) { (void)kind; return 1; }
static struct led_token calibration_led_accept(int id) { (void)id; return (struct led_token){1}; }
static uint32_t calibration_next_generation(void) { return requested_generation + 1; }
static void tracker_events_notify(void) {}
static void calibration_signal_wake(void) {}

static int flush_error, nvs_clear_error;
static int sys_flush_warm(void) { assert(warm_transaction); return flush_error; }
static void sensor_calibration_clear_begin(void) {}
static void sensor_calibration_clear_end(void) {}
static void sensor_calibration_online_mag_cancel_pending(void) {}
static int sys_wom_timeout_change_begin(void) { return 0; }
static void sys_wom_timeout_change_end(bool changed) { (void)changed; }
static int fs, sys_storage_lock, warm_tcal_armed, warm_tcal_receipt_count, warm_dirty_count;
static bool nvs_init;
static atomic_t wom_timeout_delay_ms;
#define WOM_TIMEOUT_DEFAULT_MS 1000
#define BUILD_TIMESTAMP 42
#define LED_OWNER_SYSTEM 2
#define LED_FAILED 1
#define LED_SUCCESS 2
static void retained_reset(void) {
    memset(retained, 0, sizeof(*retained));
    for (unsigned i = 0; i < 3; i++) retained->gyroSensScale[i] = 1.0f;
    retained->build_timestamp = BUILD_TIMESTAMP;
    retained->tcal_enabled = true;
}
static int led_request_id(void) { return 1; }
static int led_event_id(void) { return 1; }
static void led_request_event(int owner, int request, int event, int result) {
    (void)owner; (void)request; (void)event;
    if (result == LED_REJECTED) console_rejected++;
}
static bool sys_nvs_init(void) { return true; }
static int nvs_clear(int *store) {
    (void)store;
    if (nvs_clear_error) return nvs_clear_error;
    memset(&flash_data, 0, sizeof(flash_data));
    return 0;
}
static void system_storage_lock(void *lock, int timeout) {
    (void)lock; (void)timeout;
    sys_warm_transaction_begin();
}
static void system_storage_unlock(void *lock) {
    (void)lock;
    sys_warm_transaction_end(false);
}
