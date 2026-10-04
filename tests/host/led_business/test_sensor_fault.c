#include <errno.h>
#include <stdatomic.h>
#include "../led_sync/test_runtime.h"
#include "../../../src/system/status.h"

/* Only the status extraction uses this lock. The LED runtime retains its
 * existing host backend; interleaving happens at the real snapshot boundary. */
typedef _Atomic int atomic_t;
static int atomic_get(const atomic_t *state) { return atomic_load(state); }
static void atomic_or(atomic_t *state, int mask) { (void)atomic_fetch_or(state, mask); }
static void atomic_and(atomic_t *state, int mask) { (void)atomic_fetch_and(state, mask); }
static bool status_locked;
static void (*after_status_unlock)(void);
static k_spinlock_key_t status_spin_lock(struct k_spinlock *lock)
{
    (void)lock;
    assert(!status_locked);
    status_locked = true;
    return 0;
}
static void status_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
    (void)lock;
    (void)key;
    assert(status_locked);
    status_locked = false;
    void (*interleave)(void) = after_status_unlock;
    after_status_unlock = NULL;
    if (interleave) interleave();
}
static int transmitted_status;
static void connection_update_status(int snapshot)
{
    assert((snapshot & ~SYS_STATUS_ALL) == 0);
    transmitted_status = snapshot;
}
#define LOG_INF(...) assert(!status_locked)
#define LOG_WRN(...) assert(!status_locked)
#define LOG_DBG(...) assert(!status_locked)
#undef LOG_ERR
#define LOG_ERR(...) assert(!status_locked)
#define k_spin_lock status_spin_lock
#define k_spin_unlock status_spin_unlock
#include "status_fault_production.inc"
#undef k_spin_lock
#undef k_spin_unlock

struct host_sensor_driver { int identity; };
static const struct host_sensor_driver sensor_imu_none, supported_imu = {1};
static const struct host_sensor_driver *sensor_imus[] = {NULL, &supported_imu, &sensor_imu_none};
static const struct host_sensor_driver *sensor_imu;
static bool sensor_sensor_init;
static unsigned scan_clears, scan_completions, retained_writes, reboot_requests;
static int reboot_result;
static int packet_errors;
static int64_t no_packets_since_ms;
static bool no_packets_timeout_logged, sensor_int_during_loop;
struct host_sensor_frame { int packets, processed_packets; };
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
static void sensor_scan_clear(void) { ++scan_clears; }
static void sensor_life_mark_scan_done(void) { ++scan_completions; }
static void sensor_retained_write(void) { ++retained_writes; }
static int sys_request_system_reboot(void) { ++reboot_requests; return reboot_result; }
#include "sensor_fault_production.inc"

static void reset(void)
{
    host_reset(LED_CAP_RGB_PWM);
    atomic_store(&status_state, 0);
    status_sensor_fault = SYS_SENSOR_FAULT_NONE;
    status_wake_sem.count = 0;
    status_locked = false;
    after_status_unlock = NULL;
    transmitted_status = 0;
    sensor_imu = &sensor_imu_none;
    sensor_sensor_init = false;
    scan_clears = scan_completions = retained_writes = reboot_requests = 0;
    reboot_result = 0;
    packet_errors = 0;
    no_packets_since_ms = 0;
    no_packets_timeout_logged = sensor_int_during_loop = false;
}
static void publish(void)
{
    status_publish_faults();
    host_step(host_now_ms);
}
static void fault_glyph(enum led_semantic semantic)
{
    publish();
    assert(engine.winner.semantic == semantic);
    uint32_t origin = engine.winner.origin_ms;
    uint32_t start = host_now_ms, phase = (start - origin) % 5000;
    if (phase) start += 5000 - phase;
    for (uint32_t age = 0; age < 5000; ++age) {
        host_step(start + age);
        assert(engine.winner.semantic == semantic);
        bool on = age < 800 || (age >= 1000 && age < 1800);
        assert(host_value == (on ? 10000 : 0));
    }
}
static void scan_classification_and_init_fallback(void)
{
    reset();
    assert(host_sensor_scan_result(-1) == -1);
    assert(scan_clears == 1 && scan_completions == 1);
    assert(get_status(SYS_STATUS_SENSOR_ERROR) == SYS_STATUS_SENSOR_ERROR);
    status_wake_sem.count = 0;
    host_sensor_outer_init_failure(-EIO);
    assert(status_wake_sem.count == 0); /* No overwrite or redundant bridge wake. */
    fault_glyph(LED_SENSOR_MISSING);

    const int unsupported_ids[] = {0, 2, 7};
    for (unsigned i = 0; i < ARRAY_SIZE(unsupported_ids); ++i) {
        reset();
        assert(host_sensor_scan_result(unsupported_ids[i]) == -1);
        assert(scan_clears == 1 && scan_completions == 1);
        publish();
        assert(engine.winner.semantic == LED_SENSOR_FAULT);
    }
    fault_glyph(LED_SENSOR_FAULT);

    /* An unsupported rescan must replace a previously precise missing cause. */
    reset();
    assert(host_sensor_scan_result(-1) == -1);
    publish();
    status_wake_sem.count = 0;
    assert(host_sensor_scan_result(0) == -1);
    assert(status_wake_sem.count == 1);
    publish();
    assert(engine.winner.semantic == LED_SENSOR_FAULT);

    /* The real outer init failure branch covers driver-init/DRDY errors even
     * when no scan-specific fault was already published. */
    reset();
    host_sensor_outer_init_failure(-EIO);
    publish();
    assert(engine.winner.semantic == LED_SENSOR_FAULT);
    assert(transmitted_status == SYS_STATUS_SENSOR_ERROR);
}
static void successful_rescan_clears_cause(void)
{
    reset();
    assert(host_sensor_scan_result(-1) == -1);
    set_status(SYS_STATUS_USB_CONNECTED, true);
    set_status(SYS_STATUS_SYSTEM_ERROR, true);
    assert(host_sensor_scan_result(1) == 0 && sensor_imu == &supported_imu);
    assert(host_sensor_scan_success() == 0 && sensor_sensor_init);
    assert(get_status(SYS_STATUS_SENSOR_ERROR) == 0);
    assert(get_status(SYS_STATUS_ALL) == (SYS_STATUS_USB_CONNECTED | SYS_STATUS_SYSTEM_ERROR));
    publish();
    assert(engine.owners[LED_OWNER_SENSOR].fault == LED_FAULT_NONE);
    assert(engine.owners[LED_OWNER_SYSTEM].fault == LED_FAULT_SYSTEM);
    set_status(SYS_STATUS_SYSTEM_ERROR, false);
    host_sensor_outer_init_failure(-EIO);
    publish();
    assert(engine.winner.semantic == LED_SENSOR_FAULT); /* Missing is not stale. */

    /* General mask clearing also clears presentation cause, not just the bit. */
    set_status(SYS_STATUS_ALL, false);
    set_status(SYS_STATUS_SENSOR_ERROR, true);
    publish();
    assert(engine.winner.semantic == LED_SENSOR_FAULT);
    assert(transmitted_status == SYS_STATUS_SENSOR_ERROR);
}
static void runtime_faults_preserve_recovery_policy(void)
{
    reset();
    const struct host_sensor_frame empty = {0, 0}, good = {1, 1}, invalid = {1, 0};
    host_now_ms = 101;
    host_sensor_packet_check(&empty);
    host_now_ms = 101 + NO_PACKETS_TIMEOUT_MS - 1;
    host_sensor_packet_check(&empty);
    assert(get_status(SYS_STATUS_SENSOR_ERROR) == 0);
    ++host_now_ms;
    host_sensor_packet_check(&empty);
    publish();
    assert(engine.winner.semantic == LED_SENSOR_FAULT && no_packets_timeout_logged);
    host_sensor_packet_check(&good);
    assert(packet_errors == 0 && no_packets_since_ms == 0 && !no_packets_timeout_logged);
    assert(get_status(SYS_STATUS_SENSOR_ERROR) != 0); /* Runtime success is not an auto-clear. */
    assert(host_sensor_scan_success() == 0);
    publish();
    assert(engine.owners[LED_OWNER_SENSOR].fault == LED_FAULT_NONE);

    reset();
    host_now_ms = 101;
    for (unsigned i = 0; i < 9; ++i) host_sensor_packet_check(&invalid);
    assert(get_status(SYS_STATUS_SENSOR_ERROR) == 0 && reboot_requests == 0);
    reboot_result = -EBUSY;
    host_sensor_packet_check(&invalid);
    assert(packet_errors == 9 && reboot_requests == 1 && retained_writes == 1);
    publish();
    assert(engine.winner.semantic == LED_SENSOR_FAULT);
    reboot_result = 0;
    host_sensor_packet_check(&invalid);
    assert(packet_errors == 10 && reboot_requests == 2 && retained_writes == 2);
    host_sensor_packet_check(&good);
    assert(get_status(SYS_STATUS_SENSOR_ERROR) != 0);
}
static void system_and_safety_remain_distinct(void)
{
    reset();
    set_status(SYS_STATUS_CONNECTION_ERROR, true);
    assert(get_status(SYS_STATUS_CONNECTION_ERROR) == 2 && status_ready());
    set_status(SYS_STATUS_SYSTEM_ERROR, true);
    assert(get_status(SYS_STATUS_SYSTEM_ERROR) == 4 && !status_ready());
    assert(transmitted_status == (SYS_STATUS_CONNECTION_ERROR | SYS_STATUS_SYSTEM_ERROR));
    fault_glyph(LED_BLOCKING_FAULT);
    led_fault_publish(LED_OWNER_TCAL, LED_FAULT_SAFETY, 42);
    status_publish_faults();
    fault_glyph(LED_SAFETY_FAULT);
    set_status(SYS_STATUS_SYSTEM_ERROR, false);
    publish();
    assert(engine.owners[LED_OWNER_SYSTEM].fault == LED_FAULT_NONE);
    assert(engine.owners[LED_OWNER_TCAL].fault == LED_FAULT_SAFETY);
    assert(get_status(SYS_STATUS_ALL) == SYS_STATUS_CONNECTION_ERROR);
}
static void interleave_cause_and_system_clear(void)
{
    set_sensor_fault(SYS_SENSOR_FAULT_OTHER);
    set_status(SYS_STATUS_SYSTEM_ERROR, false);
}
static void coherent_bridge_snapshot(void)
{
    reset();
    set_sensor_fault(SYS_SENSOR_FAULT_MISSING);
    set_status(SYS_STATUS_SYSTEM_ERROR, true);
    after_status_unlock = interleave_cause_and_system_clear;
    status_publish_faults();
    /* State changes after unlock cannot mix new cause with the old status.
     * Both owners receive the same valid pre-interleave snapshot. */
    assert(get_status(SYS_STATUS_ALL) == SYS_STATUS_SENSOR_ERROR);
    assert(engine.owners[LED_OWNER_SENSOR].fault == LED_FAULT_SENSOR_MISSING);
    assert(engine.owners[LED_OWNER_SYSTEM].fault == LED_FAULT_SYSTEM);
    assert(status_wake_sem.count == 1);
    status_publish_faults();
    assert(engine.owners[LED_OWNER_SENSOR].fault == LED_FAULT_SENSOR);
    assert(engine.owners[LED_OWNER_SYSTEM].fault == LED_FAULT_NONE);
}
int main(void)
{
    scan_classification_and_init_fallback();
    successful_rescan_clears_cause();
    runtime_faults_preserve_recovery_policy();
    system_and_safety_remain_distinct();
    coherent_bridge_snapshot();
    puts("Sensor faults: scan/init/FIFO typed causes share long-pair glyph; rescan, status masks, safety and coherent snapshots preserved");
    return 0;
}
