#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdatomic.h>
#include "system/wom_timeout.h"

#define CONFIG_USE_ACTIVE_TIMEOUT 1
#define CONFIG_USE_IMU_WAKE_UP 1
#define DEFAULT_MINUTES ((CONFIG_ACTIVE_TIMEOUT_DELAY % 60000 == 0 && \
    CONFIG_ACTIVE_TIMEOUT_DELAY >= 60000 && \
    CONFIG_ACTIVE_TIMEOUT_DELAY / 60000 <= WOM_TIMEOUT_MAX_MINUTES) ? \
    CONFIG_ACTIVE_TIMEOUT_DELAY / 60000 : 0)
#define CONFIG_ACTIVE_TIMEOUT_REPEAT_WAKE_COUNT 3
#define CONFIG_ACTIVE_TIMEOUT_REPEAT_WAKE_DELAY 90000
#define CONFIG_ACTIVE_TIMEOUT_IDLE_WAKE_DELAY 90000
#define CONFIG_ACTIVE_TIMEOUT_MEANINGFUL_MOTION_MS 1000
#define SENSOR_ACTIVITY_STARTUP_GUARD_MS 1000
#define WOM_TIMEOUT_MINUTES_ID 40
#define K_FOREVER 0
#define ARG_UNUSED(x) (void)(x)
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define LOG_INF(...) ((void)0)
#define LOG_ERR(...) ((void)0)
static char last_output[256];
static void printk(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf(last_output, sizeof(last_output), format, args);
    va_end(args);
}
#define BUILD_TIMESTAMP 123
#define LED_OWNER_SYSTEM 0
#define LED_FAILED 1
#define LED_SUCCESS 2
#define TRACKER_EVENT_KIND_POWER 0
#define POWER_WOM_CANCELLED 0
#define POWER_WOM_FORCED 1
#define POWER_WOM_NORMAL 2
#define SYS_POWER_REQ_SYSTEM_OFF 3
#define SYS_POWER_REQ_REBOOT 4
#define POWER_OTA_REBOOT_NONE 0

typedef atomic_int atomic_t;
#define atomic_get(p) atomic_load(p)
#define atomic_set(p,v) atomic_store(p,v)
#define ATOMIC_INIT(v) ATOMIC_VAR_INIT(v)
struct mutex { unsigned depth; };
static struct mutex sys_storage_lock, power_plan_lock;
static void k_mutex_lock(struct mutex *m, int timeout) { (void)timeout; ++m->depth; }
static void k_mutex_unlock(struct mutex *m) { assert(m->depth); --m->depth; }
typedef int k_spinlock_key_t;
static int k_spin_lock(int *lock) { return 0; }
static void k_spin_unlock(int *lock, int key) {}
static struct { int lock, request, ota_reboot; bool physical_started; } power_requests;
static bool wom_planned, wom_announced, wom_force, wom_normal_timeout, wom_manual;
static int64_t wom_deadline, wom_commit_at;
static unsigned cancellations;
static bool power_request_cancel_wom(void *mailbox) {
    if (power_requests.physical_started) return false;
    cancellations++; return true;
}
static int64_t k_uptime_get(void) { return 100000; }
static void tracker_event_notice(int kind, int phase, int detail) {}
static void tracker_events_notify(void) {}
static void sys_power_policy_lock(void);
static void sys_power_policy_unlock(void);
static int sys_wom_timeout_change_begin(void);
static void sys_wom_timeout_change_end(bool changed);

static int fs;
static bool nvs_init, storage_available = true, stored_present;
static uint16_t stored;
static int read_result = 2, write_result = 2, clear_result, delete_result;
static unsigned writes, deletes;
static int64_t expected_delete_delay;
static bool sys_nvs_init(void) { return storage_available; }
static int nvs_read(void *filesystem, unsigned id, void *data, size_t length) {
    assert(sys_storage_lock.depth && id == WOM_TIMEOUT_MINUTES_ID);
    if (!stored_present) return -ENOENT;
    memcpy(data, &stored, length); return read_result;
}
static int nvs_write(void *filesystem, unsigned id, const void *data, size_t length) {
    assert(power_plan_lock.depth && sys_storage_lock.depth && id == WOM_TIMEOUT_MINUTES_ID);
    writes++;
    if (write_result == 0 || write_result == (int)length) {
        memcpy(&stored, data, length); stored_present = true;
    }
    return write_result;
}
static int nvs_delete(void *filesystem, unsigned id) {
    assert(power_plan_lock.depth && sys_storage_lock.depth && id == WOM_TIMEOUT_MINUTES_ID);
    assert(wom_timeout_get_delay_ms() == expected_delete_delay);
    deletes++;
    if (delete_result) return delete_result;
    if (!stored_present) return 0;
    stored_present = false;
    return 0;
}
static int nvs_clear(void *filesystem) {
    assert(power_plan_lock.depth && sys_storage_lock.depth);
    if (!clear_result) stored_present = false;
    return clear_result;
}
static struct { float gyroSensScale[3]; unsigned build_timestamp; uint8_t wom_idle_wake_streak; } retained_data;
static __typeof__(retained_data) *retained = &retained_data;
static unsigned warm_dirty_count;
static void retained_update(void) {}
static void retained_reset(void) {
    memset(retained, 0, sizeof(*retained));
    for (unsigned i = 0; i < 3; i++) retained->gyroSensScale[i] = 1.0f;
    retained->build_timestamp = BUILD_TIMESTAMP;
}
static void sensor_calibration_clear_begin(void) { assert(!power_plan_lock.depth); }
static void sensor_calibration_clear_end(void) { assert(!power_plan_lock.depth); }
static void sensor_calibration_online_mag_cancel_pending(void) {}
static unsigned led_request_id(void) { return 1; }
static unsigned led_event_id(void) { return 1; }
static void led_request_event(int owner, unsigned request, unsigned event, int state) {}
#if CONFIG_DYNAMIC_ACTIVE_TIMEOUT
static bool sensor_session_woke_from_wom, sensor_session_meaningful_motion;
static int sensor_session_activity_score;
static bool sensor_activity_score_update(int *score, float speed, float accel, int64_t now, int guard, int duration) {
    return speed > 100;
}
#endif
static unsigned rejected;
static void console_reject(void) { rejected++; }
static bool console_require_args(size_t argc, size_t low, size_t high) {
    if (argc >= low && argc <= high) return true;
    console_reject(); return false;
}
static unsigned enter_calls;
static int enter_result;
static int sys_request_WOM_now(void) { enter_calls++; return enter_result; }
#include "production.inc"

static void command(const char *value) {
    char *args[] = {"wom", "timeout", (char *)value};
    console_cmd_wom(value ? 3 : 2, args);
}

int main(void) {
    (void)wom_deadline;
    (void)wom_commit_at;
    {
    char *enter[] = {"wom", "enter", "extra", "extra"};
    unsigned old_rejected = rejected;
    console_cmd_wom(3, enter);
    console_cmd_wom(4, enter);
    assert(enter_calls == 0 && rejected == old_rejected + 2);
    console_cmd_wom(2, enter);
    assert(enter_calls == 1 && rejected == old_rejected + 2);
    enter_result = -EBUSY;
    console_cmd_wom(2, enter);
    assert(enter_calls == 2 && rejected == old_rejected + 3);
    enter_result = -ENOTSUP;
    console_cmd_wom(2, enter);
    assert(enter_calls == 3 && rejected == old_rejected + 4);
    assert(writes == 0 && deletes == 0);
    rejected = old_rejected;
    }
    wom_timeout_load();
    assert(wom_timeout_get_delay_ms() == CONFIG_ACTIVE_TIMEOUT_DELAY);
#if CONFIG_SLEEP_ON_ACTIVE_TIMEOUT
    assert(wom_timeout_get_minutes() == DEFAULT_MINUTES);
    command(NULL);
    assert(writes == 0 && deletes == 0 && wom_timeout_get_delay_ms() == CONFIG_ACTIVE_TIMEOUT_DELAY);
    if (DEFAULT_MINUTES == 0) {
        char exact[32];
        snprintf(exact, sizeof(exact), "%d ms", CONFIG_ACTIVE_TIMEOUT_DELAY);
        assert(strstr(last_output, "not representable") && strstr(last_output, exact));
    }
    expected_delete_delay = CONFIG_ACTIVE_TIMEOUT_DELAY;
    assert(wom_timeout_set_minutes(0) == 0 && !stored_present && deletes == 1);
    assert(wom_timeout_get_delay_ms() == CONFIG_ACTIVE_TIMEOUT_DELAY);
    assert(wom_timeout_set_minutes(35792) == -EINVAL);
    assert(wom_timeout_set_minutes(UINT16_MAX) == -EINVAL);
    assert(writes == 0);
    assert(wom_timeout_set_minutes(1) == 0 && wom_timeout_get_delay_ms() == 60000);
    assert(wom_timeout_set_minutes(35791) == 0 && wom_timeout_get_delay_ms() == INT64_C(2147460000));
    atomic_set(&wom_timeout_delay_ms, 600000);
    wom_timeout_load(); /* cold boot and WOM wake both use this exact loader */
    assert(wom_timeout_get_minutes() == 35791);
    stored = 0; wom_timeout_load(); assert(wom_timeout_get_minutes() == DEFAULT_MINUTES);
    stored = 35792; wom_timeout_load(); assert(wom_timeout_get_minutes() == DEFAULT_MINUTES);
    stored = 30; read_result = 1; wom_timeout_load(); assert(wom_timeout_get_minutes() == DEFAULT_MINUTES);
    read_result = -EIO; wom_timeout_load(); assert(wom_timeout_get_minutes() == DEFAULT_MINUTES);
    read_result = 2; wom_timeout_load(); assert(wom_timeout_get_minutes() == 30);

    wom_planned = wom_announced = wom_normal_timeout = true;
    write_result = -ENOSPC;
    assert(wom_timeout_set_minutes(20) == -ENOSPC);
    assert(wom_timeout_get_minutes() == 30 && wom_planned && cancellations == 0);
    write_result = 1;
    assert(wom_timeout_set_minutes(20) == -EIO && wom_timeout_get_minutes() == 30);
    storage_available = false;
    assert(wom_timeout_set_minutes(20) == -EIO && wom_planned);
    storage_available = true; write_result = 2;
    assert(wom_timeout_set_minutes(20) == 0 && !wom_planned && cancellations == 1);
    wom_planned = wom_announced = true; wom_normal_timeout = false;
    assert(wom_timeout_set_minutes(40) == 0 && wom_planned && cancellations == 1);
    write_result = 0;
    assert(wom_timeout_set_minutes(40) == 0 && wom_timeout_get_minutes() == 40);
    write_result = 2;
    unsigned old_writes = writes;
    power_requests.physical_started = true;
    assert(wom_timeout_set_minutes(50) == -EBUSY && writes == old_writes);
    assert(sys_clear() == -EBUSY && wom_timeout_get_minutes() == 40);
    power_requests.physical_started = false;
    power_requests.request = SYS_POWER_REQ_SYSTEM_OFF;
    assert(wom_timeout_set_minutes(50) == -EBUSY);
    power_requests.request = 0;

#if CONFIG_DYNAMIC_ACTIVE_TIMEOUT
    sensor_session_woke_from_wom = true;
    retained->wom_idle_wake_streak = 0;
    assert(!sensor_uses_normal_timeout() && sensor_get_active_timeout_delay() == 90000);
    assert(wom_timeout_set_minutes(60) == 0);
    assert(sensor_get_active_timeout_delay() == 90000);
    retained->wom_idle_wake_streak = 3;
    assert(sensor_get_active_timeout_delay() == 90000);
    sensor_update_session_motion(101, 0, 100000);
    assert(sensor_uses_normal_timeout() && sensor_get_active_timeout_delay() == 3600000);
#else
    assert(sensor_uses_normal_timeout() && sensor_get_active_timeout_delay() == 2400000);
#endif

    /* Clear is a committed delete, including when RAM already matches default. */
    assert(wom_timeout_set_minutes(30) == 0);
    expected_delete_delay = 1800000;
    wom_planned = wom_announced = wom_normal_timeout = true;
    unsigned old_cancellations = cancellations, old_deletes = deletes;
    old_writes = writes;
    delete_result = -EIO;
    assert(wom_timeout_set_minutes(0) == -EIO);
    assert(stored_present && stored == 30 && wom_timeout_get_delay_ms() == 1800000);
    assert(wom_planned && cancellations == old_cancellations && writes == old_writes);
    delete_result = 2; /* Positive delete result is not a successful value write. */
    assert(wom_timeout_set_minutes(0) == -EIO && stored_present && stored == 30);
    assert(wom_timeout_get_delay_ms() == 1800000 && wom_planned);
    delete_result = 0;
    storage_available = false;
    old_deletes = deletes;
    assert(wom_timeout_set_minutes(0) == -EIO && deletes == old_deletes);
    assert(stored_present && wom_timeout_get_delay_ms() == 1800000 && wom_planned);
    storage_available = true;
    power_requests.physical_started = true;
    assert(wom_timeout_set_minutes(0) == -EBUSY && deletes == old_deletes);
    power_requests.physical_started = false;
    power_requests.request = SYS_POWER_REQ_SYSTEM_OFF;
    assert(wom_timeout_set_minutes(0) == -EBUSY && deletes == old_deletes);
    power_requests.request = 0;
    power_requests.ota_reboot = 1;
    assert(wom_timeout_set_minutes(0) == -EBUSY && deletes == old_deletes);
    power_requests.ota_reboot = POWER_OTA_REBOOT_NONE;
    assert(stored_present && stored == 30 && wom_timeout_get_delay_ms() == 1800000);
    assert(wom_timeout_set_minutes(0) == 0 && deletes == old_deletes + 1);
    assert(!stored_present && writes == old_writes);
    assert(wom_timeout_get_delay_ms() == CONFIG_ACTIVE_TIMEOUT_DELAY);
    assert(!wom_planned && cancellations == old_cancellations + 1);
    atomic_set(&wom_timeout_delay_ms, 60000);
    wom_timeout_load();
    assert(wom_timeout_get_delay_ms() == CONFIG_ACTIVE_TIMEOUT_DELAY);
    expected_delete_delay = CONFIG_ACTIVE_TIMEOUT_DELAY;
    assert(wom_timeout_set_minutes(0) == 0 && deletes == old_deletes + 2 && !stored_present);
    assert(wom_timeout_get_delay_ms() == CONFIG_ACTIVE_TIMEOUT_DELAY);
    if (DEFAULT_MINUTES) {
        assert(wom_timeout_set_minutes(DEFAULT_MINUTES) == 0 && stored_present);
        old_deletes = deletes;
        assert(wom_timeout_set_minutes(0) == 0 && deletes == old_deletes + 1 && !stored_present);
        assert(wom_timeout_get_delay_ms() == CONFIG_ACTIVE_TIMEOUT_DELAY);
    }

    assert(wom_timeout_set_minutes(30) == 0);
    expected_delete_delay = 1800000;
    wom_planned = wom_announced = true; wom_normal_timeout = false;
    old_cancellations = cancellations;
#if CONFIG_DYNAMIC_ACTIVE_TIMEOUT
    sensor_session_woke_from_wom = true;
    sensor_session_meaningful_motion = false;
    retained->wom_idle_wake_streak = 0;
    assert(sensor_get_active_timeout_delay() == 90000);
#endif
    assert(wom_timeout_set_minutes(0) == 0 && !stored_present);
    assert(wom_planned && cancellations == old_cancellations);
#if CONFIG_DYNAMIC_ACTIVE_TIMEOUT
    assert(sensor_get_active_timeout_delay() == 90000);
    retained->wom_idle_wake_streak = 3;
    assert(sensor_get_active_timeout_delay() == 90000);
    sensor_update_session_motion(101, 0, 100000);
#endif
    assert(sensor_uses_normal_timeout());
    assert(sensor_get_active_timeout_delay() == CONFIG_ACTIVE_TIMEOUT_DELAY);
    const char *bad[] = {"", "35792", "-1", "+1", "1.5", "2m", "off", "default", "999999999999999999999999"};
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); i++) {
        old_writes = writes; old_deletes = deletes; unsigned previous = rejected;
        command(bad[i]);
        assert(writes == old_writes && deletes == old_deletes && rejected == previous + 1);
    }
    command(NULL); old_writes = writes;
    char *extra[] = {"wom", "timeout", "5", "extra"};
    console_cmd_wom(4, extra); assert(writes == old_writes);
    command("5"); assert(wom_timeout_get_minutes() == 5);
    expected_delete_delay = 300000;
    old_writes = writes; old_deletes = deletes;
    unsigned old_rejected = rejected;
    command(NULL);
    assert(writes == old_writes && deletes == old_deletes && stored_present);
    delete_result = -EIO;
    command("0");
    assert(rejected == old_rejected + 1 && strstr(last_output, "setting unchanged"));
    assert(stored_present && stored == 5 && wom_timeout_get_delay_ms() == 300000);
    delete_result = 0;
    command("0");
    assert(rejected == old_rejected + 1 && strstr(last_output, "restored to firmware default"));
    assert(writes == old_writes && deletes == old_deletes + 2 && !stored_present);
    assert(wom_timeout_get_delay_ms() == CONFIG_ACTIVE_TIMEOUT_DELAY);
    expected_delete_delay = CONFIG_ACTIVE_TIMEOUT_DELAY;
    command("0");
    assert(rejected == old_rejected + 1 && deletes == old_deletes + 3 && !stored_present);
    assert(wom_timeout_get_delay_ms() == CONFIG_ACTIVE_TIMEOUT_DELAY);
    command("5");
    clear_result = -EIO;
    assert(sys_clear() == -EIO && wom_timeout_get_minutes() == 5);
    clear_result = 0;
    assert(sys_clear() == 0 && wom_timeout_get_minutes() == DEFAULT_MINUTES && !stored_present);
    wom_timeout_load(); assert(wom_timeout_get_minutes() == DEFAULT_MINUTES);
    assert(wom_timeout_get_delay_ms() == CONFIG_ACTIVE_TIMEOUT_DELAY);
#else
    assert(wom_timeout_get_minutes() == 0);
    assert(wom_timeout_set_minutes(10) == -ENOTSUP && writes == 0);
    assert(wom_timeout_set_minutes(0) == -ENOTSUP && deletes == 0);
    assert(sensor_get_active_timeout_delay() == CONFIG_ACTIVE_TIMEOUT_DELAY); /* shutdown-only legacy policy intact */
    command(NULL); command("10"); assert(rejected == 1 && writes == 0);
    command("0"); assert(rejected == 2 && deletes == 0 && writes == 0);
#endif
    assert(!sys_storage_lock.depth && !power_plan_lock.depth);
    puts("WOM timeout: storage, boundaries, policy, power gate and console passed");
    return 0;
}
