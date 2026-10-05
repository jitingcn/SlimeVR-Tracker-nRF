#pragma once
/* Hardware/event leaves only. Ownership, request admission, saver and reset
 * generations are compiled verbatim from production below. */
#define TCAL_ACCUM_ROBUST_BLOCKS 5
K_MUTEX_DEFINE(calibration_request_lock);
static int requested_calibration, requested_storage_error;
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
