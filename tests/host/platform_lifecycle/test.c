#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <string.h>
#define LOG_INF(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define IS_ENABLED(x) (x)
static bool host_clock_lfxo = true;
#define CONFIG_CLOCK_USE_LFXO host_clock_lfxo
#define CONFIG_CLOCK_USE_LF_SYNTH (!host_clock_lfxo)
#define CONFIG_CLOCK_USE_LFXO_MODE_FULL_SWING 0
#define CONFIG_CLOCK_USE_LFXO_MODE_LOW_SWING 0
#define CLOCK_LFCLKSRC_SRC_Synth 1
#define CONFIG_TASK_WDT 1
#define CONFIG_BUILD_OUTPUT_UF2 ADAFRUIT_FAULT_RECOVERY
#define CONFIG_BOOTLOADER_MCUBOOT (!ADAFRUIT_FAULT_RECOVERY)
#define WATCHDOG_RESET_THRESHOLD 3
#define CONFIG_SENSOR_TCAL_HEATED 0
#include "clock_config.inc"
#define SYS_REBOOT_COLD 0
static _Thread_local unsigned int irq_depth, mutex_depth, caller;
static int reboots;
static void sys_reboot(int type) { assert(!irq_depth && !mutex_depth); reboots++; }
#if ADAFRUIT_FAULT_RECOVERY
static void k_msleep(int ms) { assert(ms == 100); }
#endif
static pthread_mutex_t rendezvous = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t changed = PTHREAD_COND_INITIALIZER;
static bool gate_enabled, owner_waiting, contender_waiting, owner_released;
struct k_mutex { pthread_mutex_t native; };
#define K_MUTEX_DEFINE(name) struct k_mutex name
#define K_FOREVER (-1)
static inline int k_mutex_lock(struct k_mutex *mutex, int timeout)
{
    if (gate_enabled && caller == 2 && mutex_depth == 0) {
        assert(pthread_mutex_trylock(&mutex->native) == EBUSY);
        assert(pthread_mutex_lock(&rendezvous) == 0);
        contender_waiting = true;
        assert(pthread_cond_broadcast(&changed) == 0);
        assert(pthread_mutex_unlock(&rendezvous) == 0);
    }
    assert(pthread_mutex_lock(&mutex->native) == 0);
    ++mutex_depth;
    return 0;
}
static inline int k_mutex_unlock(struct k_mutex *mutex)
{
    assert(!irq_depth && mutex_depth > 0);
    --mutex_depth;
    assert(pthread_mutex_unlock(&mutex->native) == 0);
    return 0;
}

typedef enum {
    NRF_CLOCK_LFCLK_RC, NRF_CLOCK_LFCLK_XTAL, NRF_CLOCK_LFCLK_SYNTH,
#if TEST_EXTERNAL
    NRF_CLOCK_LFCLK_XTAL_FULL_SWING, NRF_CLOCK_LFCLK_XTAL_LOW_SWING,
#endif
} nrf_clock_lfclk_t;
#if TEST_EXTERNAL
#define NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES 1
#endif
#define NRF_CLOCK 0
#define NRF_CLOCK_DOMAIN_LFCLK 0
#define NRF_CLOCK_TASK_LFCLKSTOP 0
#define NRF_CLOCK_TASK_LFCLKSTART 1
#define NRF_CLOCK_EVENT_LFCLKSTARTED 0
#define NRF_CLOCK_INT_LF_STARTED_MASK 1u
#define OTHER_CLOCK_MASK 2u
enum start_result { READY, NO_START, WRONG_SOURCE, NO_EVENT };
enum stop_failure { STOP_OK, REQUEST_STUCK, SOURCE_STUCK, FALLBACK_STOP_STUCK };
static struct {
    nrf_clock_lfclk_t active, requested;
    bool running, event, request, watchdog, other_event, nvic_pending;
    uint32_t interrupts;
    enum start_result result[3];
    enum stop_failure stop_failure;
    unsigned int stop_request_left, stop_source_left, start_left;
    unsigned int starts, stops, writes, delays, watchdog_writes, event_clears;
    unsigned int radio_irqs, sensor_irqs, other_irqs, lf_callbacks, reserved_events;
    bool last_lf_policy;
    struct { unsigned int caller; int task; } trace[9];
    unsigned int trace_count;
} clock_model;

static void shared_clock_irq(void)
{
    if (irq_depth || !clock_model.nvic_pending) return;
    clock_model.nvic_pending = false;
    if (clock_model.other_event && (clock_model.interrupts & OTHER_CLOCK_MASK)) {
        clock_model.other_event = false;
        clock_model.interrupts &= ~OTHER_CLOCK_MASK;
        ++clock_model.other_irqs;
    }
    if (clock_model.event && !(clock_model.interrupts & NRF_CLOCK_INT_LF_STARTED_MASK)) {
        ++clock_model.reserved_events;
    }
    if (clock_model.event && (clock_model.interrupts & NRF_CLOCK_INT_LF_STARTED_MASK)) {
        clock_model.event = false;
        clock_model.interrupts &= ~NRF_CLOCK_INT_LF_STARTED_MASK;
        ++clock_model.lf_callbacks;
    }
}
static unsigned int irq_lock(void)
{
    unsigned int key = irq_depth;
    ++irq_depth;
    return key;
}
static void irq_unlock(unsigned int key)
{
    assert(irq_depth == key + 1);
    irq_depth = key;
    shared_clock_irq();
}
static inline uint32_t nrf_clock_int_enable_check(int clock, uint32_t mask)
{
    return clock_model.interrupts & mask;
}
static inline void nrf_clock_int_enable(int clock, uint32_t mask)
{
    assert(irq_depth == 1 && mask == NRF_CLOCK_INT_LF_STARTED_MASK);
    clock_model.interrupts |= mask;
    clock_model.last_lf_policy = true;
    if (clock_model.event) clock_model.nvic_pending = true;
}
static inline void nrf_clock_int_disable(int clock, uint32_t mask)
{
    assert(irq_depth == 1 && mask == NRF_CLOCK_INT_LF_STARTED_MASK);
    clock_model.interrupts &= ~mask;
    clock_model.last_lf_policy = false;
}
static inline bool nrf_clock_start_task_check(int clock, int domain)
{
    return clock_model.request;
}
static bool nrf_clock_is_running(int clock, int domain, nrf_clock_lfclk_t *source)
{
    *source = clock_model.active;
    return clock_model.running;
}
static void trace_clock(int task)
{
    assert(irq_depth == 1);
    assert(clock_model.trace_count < sizeof(clock_model.trace) / sizeof(clock_model.trace[0]));
    clock_model.trace[clock_model.trace_count].caller = caller;
    clock_model.trace[clock_model.trace_count++].task = task;
}
static void nrf_clock_task_trigger(int clock, int task)
{
    trace_clock(task);
    if (task == NRF_CLOCK_TASK_LFCLKSTOP) {
        ++clock_model.stops;
        clock_model.stop_request_left = 1;
        clock_model.stop_source_left = 3;
        clock_model.start_left = 0;
    } else {
        assert(!clock_model.request);
        assert(!clock_model.running || clock_model.active == NRF_CLOCK_LFCLK_RC);
        assert(!clock_model.event);
        ++clock_model.starts;
        clock_model.request = true;
        clock_model.start_left = 3;
        clock_model.stop_request_left = clock_model.stop_source_left = 0;
    }
}
static void nrf_clock_lf_src_set(int clock, nrf_clock_lfclk_t source)
{
    trace_clock(2);
    assert(!clock_model.request);
    assert(!clock_model.running || clock_model.active == NRF_CLOCK_LFCLK_RC);
    if (clock_model.running && clock_model.watchdog) ++clock_model.watchdog_writes;
    clock_model.requested = source;
    ++clock_model.writes;
}
static void nrf_clock_event_clear(int clock, int which)
{
    assert(irq_depth == 1 && !clock_model.request);
    assert(!clock_model.running || clock_model.active == NRF_CLOCK_LFCLK_RC);
    clock_model.event = false;
    ++clock_model.event_clears;
}
static bool nrf_clock_event_check(int clock, int which) { return clock_model.event; }
static void wait_for_contender(void)
{
    if (!gate_enabled || caller != 1 || clock_model.starts != 1 || owner_waiting) return;
    assert(pthread_mutex_lock(&rendezvous) == 0);
    owner_waiting = true;
    assert(pthread_cond_broadcast(&changed) == 0);
    while (!owner_released) assert(pthread_cond_wait(&changed, &rendezvous) == 0);
    assert(pthread_mutex_unlock(&rendezvous) == 0);
}
static void nrfx_coredep_delay_us(uint32_t us)
{
    assert(us == LFCLK_WAIT_STEP_US);
    assert(++clock_model.delays < 10000);
    if (clock_model.start_left) {
        assert(irq_depth == 0);
        assert(!(clock_model.interrupts & NRF_CLOCK_INT_LF_STARTED_MASK));
        ++clock_model.radio_irqs;
        ++clock_model.sensor_irqs;
        wait_for_contender();
        if (--clock_model.start_left == 0) {
            enum start_result result = clock_model.result[clock_model.starts - 1];
            nrf_clock_lfclk_t actual = clock_model.requested;
#if TEST_EXTERNAL
            if (actual == NRF_CLOCK_LFCLK_XTAL_FULL_SWING ||
                actual == NRF_CLOCK_LFCLK_XTAL_LOW_SWING) actual = NRF_CLOCK_LFCLK_XTAL;
#endif
            if (result != NO_START) {
                clock_model.running = true;
                clock_model.active = result == WRONG_SOURCE
                    ? (actual == NRF_CLOCK_LFCLK_RC ? NRF_CLOCK_LFCLK_XTAL : NRF_CLOCK_LFCLK_RC)
                    : actual;
                clock_model.event = result != NO_EVENT;
            }
            clock_model.other_event = true;
            if ((clock_model.other_event && (clock_model.interrupts & OTHER_CLOCK_MASK)) ||
                (clock_model.event && (clock_model.interrupts & NRF_CLOCK_INT_LF_STARTED_MASK))) {
                clock_model.nvic_pending = true;
            }
        }
    } else if (clock_model.stop_request_left || clock_model.stop_source_left) {
        assert(irq_depth == 1);
        bool request_stuck = clock_model.stop_failure == REQUEST_STUCK ||
            (clock_model.stop_failure == FALLBACK_STOP_STUCK && clock_model.stops == 2);
        if (!request_stuck && clock_model.stop_request_left &&
            --clock_model.stop_request_left == 0) clock_model.request = false;
        if (clock_model.stop_failure != SOURCE_STUCK && clock_model.stop_source_left &&
            --clock_model.stop_source_left == 0) {
            clock_model.running = clock_model.watchdog;
            if (clock_model.watchdog) clock_model.active = NRF_CLOCK_LFCLK_RC;
        }
    } else {
        assert(irq_depth == 0);
        ++clock_model.radio_irqs;
        ++clock_model.sensor_irqs;
    }
    shared_clock_irq();
}
static void reset_clock_fence(void);
static void reset_clock(bool watchdog, nrf_clock_lfclk_t source, bool lf_enabled)
{
    assert(!irq_depth && !mutex_depth);
    reset_clock_fence();
    host_clock_lfxo = true;
    memset(&clock_model, 0, sizeof(clock_model));
    clock_model.running = clock_model.request = true;
    clock_model.watchdog = watchdog;
    clock_model.active = clock_model.requested = source;
    clock_model.interrupts = OTHER_CLOCK_MASK | (lf_enabled ? NRF_CLOCK_INT_LF_STARTED_MASK : 0);
    clock_model.last_lf_policy = lf_enabled;
    reboots = 0;
    gate_enabled = owner_waiting = contender_waiting = owner_released = false;
    caller = 0;
}

#include "watchdog_config.inc"
static int channel_ids[WDT_CHANNEL_COUNT];
static uint32_t channel_timeouts[WDT_CHANNEL_COUNT];
static bool watchdog_initialized;
static int add_result, adds, feeds, last_feed;
static uint32_t added_timeout;
static void watchdog_timeout_callback(int channel, void *data) {}
static int task_wdt_add(uint32_t timeout, void (*callback)(int, void *), void *data)
{ adds++; added_timeout = timeout; return add_result; }
static int task_wdt_feed(int channel) { feeds++; last_feed = channel; return 0; }
static int task_wdt_delete(int channel) { return 0; }

struct device { int index; };
static const struct device bus1 = {1}, bus2 = {2};
#define DT_NODELABEL(x) x
#define DT_PARENT(x) x
#define DT_NODE_HAS_STATUS_OKAY(x) 1
#define DEVICE_DT_GET_EXPANDED(x) DEVICE_##x
#define DEVICE_DT_GET(x) DEVICE_DT_GET_EXPANDED(x)
#define DEVICE_imu_spi (&bus1)
#define DEVICE_imu (&bus1)
#define DEVICE_mag_spi (&bus2)
#define DEVICE_mag (&bus2)
#define WATCHDOG_NODE wdt
#define DEVICE_wdt (&bus1)
#define WATCHDOG_STATE_MAGIC 0xA5
static struct {
    struct {
        unsigned magic, reset_count, total_wdt_resets, last_failed_channel;
    } watchdog_state;
} retained_storage, *const retained = &retained_storage;
static struct { unsigned GPREGRET; } power;
#define NRF_POWER (&power)
static bool reset_was_wdt, wdt_ready = true;
static int wdt_init_result, wdt_init_calls;
static bool watchdog_caused_reset(void) { return reset_was_wdt; }
static bool device_is_ready(const struct device *dev) { return wdt_ready; }
static int task_wdt_init(const struct device *dev) { ++wdt_init_calls; return wdt_init_result; }
static bool recovery_supported;
static bool sys_bootloader_supports_recovery(void) { return recovery_supported; }
static int retained_updates;
static void retained_update(void) { ++retained_updates; }
#if !ADAFRUIT_FAULT_RECOVERY
static void sys_enter_dfu(bool ota) { ++reboots; }
#endif
enum pm_device_action { PM_DEVICE_ACTION_SUSPEND, PM_DEVICE_ACTION_RESUME };
static bool bus_suspended[3];
static int pm_error[2][3], pm_calls[2][3];
static int pm_device_action_run(const struct device *device, enum pm_device_action action)
{
    int id = device->index;
    pm_calls[action][id]++;
    if (pm_error[action][id]) return pm_error[action][id];
    bool suspended = action == PM_DEVICE_ACTION_SUSPEND;
    if (bus_suspended[id] == suspended) return -EALREADY;
    bus_suspended[id] = suspended;
    return 0;
}
static bool main_ok, sensor_sensor_init = true, mag_available = true, mag_enabled = true;
static int shutdowns, wom_calls, scan_result;
static void shutdown_driver(void) { shutdowns++; }
static uint8_t setup_wom_driver(void) { wom_calls++; return 0x12; }
struct driver { void (*shutdown)(void); uint8_t (*setup_WOM)(void); };
static const struct driver sensor_imu_none, sensor_mag_none;
static const struct driver driver = {shutdown_driver, setup_wom_driver};
static const struct driver *sensor_imu = &driver, *sensor_mag = &driver;
static void sensor_calibration_set_consumer_ready(bool ready) {}
static void sensor_mag_timing_reset(void) {}
static int sensor_request_scan(bool a, bool b) { return scan_result; }
static K_MUTEX_DEFINE(sensor_lifecycle_lock);
static int sensor_owner_token;
static void *const power_thread_id = &sensor_owner_token;
static void *sensor_suspend_owner = &sensor_owner_token;
static void *k_current_get(void) { return &sensor_owner_token; }
static int main_imu_suspend(void)
{
    sensor_suspend_owner = k_current_get();
    return 0;
}
#include "production.inc"

static void reset_clock_fence(void)
{
#if HOST_LF_INIT_FENCE
    /* Each case models a fresh boot; production never clears this latch. */
    clock_external_init_disallowed = false;
#endif
}

static void init_mutex(struct k_mutex *mutex)
{
    pthread_mutexattr_t attributes;
    assert(pthread_mutexattr_init(&attributes) == 0);
    assert(pthread_mutexattr_settype(&attributes, PTHREAD_MUTEX_RECURSIVE) == 0);
    assert(pthread_mutex_init(&mutex->native, &attributes) == 0);
    assert(pthread_mutexattr_destroy(&attributes) == 0);
}

static void check_clock_release(bool saved_lf, unsigned int callbacks)
{
    assert(!irq_depth && !mutex_depth);
    assert(clock_model.last_lf_policy == saved_lf);
    assert(clock_model.lf_callbacks == callbacks);
}

static void test_clock_failures(void)
{
    static const struct {
        bool watchdog;
        nrf_clock_lfclk_t initial, target, final;
        enum stop_failure stop_failure;
        enum start_result first, second;
        unsigned int starts, stops, reboots, watchdog_writes;
        bool stale_event, final_event;
    } cases[] = {
        {false, NRF_CLOCK_LFCLK_XTAL, NRF_CLOCK_LFCLK_RC, NRF_CLOCK_LFCLK_RC,
         STOP_OK, READY, READY, 1, 1, 0, 0, false, true},
        {true, NRF_CLOCK_LFCLK_RC, NRF_CLOCK_LFCLK_XTAL, NRF_CLOCK_LFCLK_XTAL,
         STOP_OK, READY, READY, 1, 1, 0, 1, false, true},
        {true, NRF_CLOCK_LFCLK_XTAL, NRF_CLOCK_LFCLK_RC, NRF_CLOCK_LFCLK_RC,
         STOP_OK, READY, READY, 1, 1, 0, 1, false, true},
        {false, NRF_CLOCK_LFCLK_XTAL, NRF_CLOCK_LFCLK_RC, NRF_CLOCK_LFCLK_XTAL,
         SOURCE_STUCK, READY, READY, 0, 1, 1, 0, false, false},
        {true, NRF_CLOCK_LFCLK_RC, NRF_CLOCK_LFCLK_XTAL, NRF_CLOCK_LFCLK_RC,
         REQUEST_STUCK, READY, READY, 0, 2, 1, 0, false, false},
        {true, NRF_CLOCK_LFCLK_RC, NRF_CLOCK_LFCLK_XTAL, NRF_CLOCK_LFCLK_RC,
         FALLBACK_STOP_STUCK, NO_START, READY, 1, 2, 1, 1, false, false},
        {false, NRF_CLOCK_LFCLK_RC, NRF_CLOCK_LFCLK_XTAL, NRF_CLOCK_LFCLK_RC,
         STOP_OK, NO_START, READY, 2, 2, 0, 0, false, true},
        {true, NRF_CLOCK_LFCLK_RC, NRF_CLOCK_LFCLK_XTAL, NRF_CLOCK_LFCLK_RC,
         STOP_OK, NO_START, READY, 2, 2, 0, 2, false, true},
        {false, NRF_CLOCK_LFCLK_RC, NRF_CLOCK_LFCLK_XTAL, NRF_CLOCK_LFCLK_RC,
         STOP_OK, NO_START, NO_START, 2, 2, 1, 0, false, false},
        {true, NRF_CLOCK_LFCLK_RC, NRF_CLOCK_LFCLK_XTAL, NRF_CLOCK_LFCLK_RC,
         STOP_OK, NO_START, NO_START, 2, 2, 1, 2, false, false},
        {false, NRF_CLOCK_LFCLK_RC, NRF_CLOCK_LFCLK_XTAL, NRF_CLOCK_LFCLK_RC,
         STOP_OK, WRONG_SOURCE, READY, 2, 2, 0, 0, false, true},
        {false, NRF_CLOCK_LFCLK_RC, NRF_CLOCK_LFCLK_XTAL, NRF_CLOCK_LFCLK_RC,
         STOP_OK, NO_EVENT, READY, 2, 2, 0, 0, true, true},
        {false, NRF_CLOCK_LFCLK_RC, NRF_CLOCK_LFCLK_XTAL, NRF_CLOCK_LFCLK_XTAL,
         STOP_OK, NO_EVENT, WRONG_SOURCE, 2, 2, 1, 0, false, true},
    };
    for (unsigned int saved = 0; saved < 2; ++saved) {
        for (unsigned int i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
            reset_clock(cases[i].watchdog, cases[i].initial, saved);
            clock_model.stop_failure = cases[i].stop_failure;
            clock_model.result[0] = cases[i].first;
            clock_model.result[1] = cases[i].second;
            if (cases[i].stale_event) {
                clock_model.event = clock_model.other_event = clock_model.nvic_pending = true;
            }
            clock_switch(cases[i].target);
            assert(clock_model.active == cases[i].final);
            assert(clock_model.starts == cases[i].starts && clock_model.writes == cases[i].starts);
            assert(clock_model.stops == cases[i].stops && reboots == (int)cases[i].reboots);
            assert(clock_model.event_clears == cases[i].starts);
            assert(clock_model.watchdog_writes == cases[i].watchdog_writes);
            check_clock_release(saved, saved && cases[i].final_event);
            assert(clock_model.event == (cases[i].final_event && !saved));
            assert(!!(clock_model.interrupts & NRF_CLOCK_INT_LF_STARTED_MASK) ==
                   (saved && !cases[i].final_event));
            if (cases[i].starts) {
                assert(clock_model.radio_irqs >= 3 && clock_model.sensor_irqs >= 3);
                assert(clock_model.other_irqs == 1 && !(clock_model.interrupts & OTHER_CLOCK_MASK));
            }
            if (!cases[i].reboots) assert(clock_model.running && clock_model.request);
            if (cases[i].stop_failure == REQUEST_STUCK) assert(clock_model.request);
            if (i == 0) {
                assert(clock_model.reserved_events == 1);
            }
        }
    }
}

struct clock_call { unsigned int caller; nrf_clock_lfclk_t target; bool shutdown; };
static void *call_clock(void *data)
{
    const struct clock_call *call = data;
    caller = call->caller;
    if (call->shutdown) clock_pre_shutdown();
    else clock_switch(call->target);
    return NULL;
}
static void *call_external_init(void *data)
{
    const struct clock_call *call = data;
    caller = call->caller;
    clock_init_external();
    return NULL;
}
static void test_clock_contention(void)
{
    for (unsigned int mode = 0; mode < 3; ++mode) {
        reset_clock(true, NRF_CLOCK_LFCLK_RC, false);
        gate_enabled = true;
        const struct clock_call calls[] = {
            {1, NRF_CLOCK_LFCLK_XTAL, false},
            {2, mode == 0 ? NRF_CLOCK_LFCLK_XTAL : NRF_CLOCK_LFCLK_RC, mode == 2},
        };
        pthread_t owner, contender;
        assert(pthread_create(&owner, NULL, call_clock, (void *)&calls[0]) == 0);
        assert(pthread_mutex_lock(&rendezvous) == 0);
        while (!owner_waiting) assert(pthread_cond_wait(&changed, &rendezvous) == 0);
        assert(clock_model.running && clock_model.active == NRF_CLOCK_LFCLK_RC);
        assert(pthread_create(&contender, NULL, call_clock, (void *)&calls[1]) == 0);
        while (!contender_waiting) assert(pthread_cond_wait(&changed, &rendezvous) == 0);
        assert(clock_model.stops == 1 && clock_model.writes == 1 && clock_model.starts == 1);
        owner_released = true;
        assert(pthread_cond_broadcast(&changed) == 0);
        assert(pthread_mutex_unlock(&rendezvous) == 0);
        assert(pthread_join(owner, NULL) == 0 && pthread_join(contender, NULL) == 0);
        unsigned int transactions = mode == 0 ? 1 : 2;
        assert(!reboots && clock_model.running && clock_model.request);
        assert(clock_model.active == (mode == 0 ? NRF_CLOCK_LFCLK_XTAL : NRF_CLOCK_LFCLK_RC));
        assert(clock_model.starts == transactions && clock_model.stops == transactions);
        assert(clock_model.writes == transactions && clock_model.trace_count == 3 * transactions);
        for (unsigned int i = 0; i < clock_model.trace_count; ++i) {
            assert(clock_model.trace[i].caller == (i < 3 ? 1u : 2u));
            assert(clock_model.trace[i].task == (i % 3 == 0 ? 0 : i % 3 == 1 ? 2 : 1));
        }
        check_clock_release(false, 0);
    }
}

static void test_clock_noop_and_shutdown(void)
{
    for (unsigned int saved = 0; saved < 2; ++saved) {
        for (unsigned int source = NRF_CLOCK_LFCLK_RC; source <= NRF_CLOCK_LFCLK_XTAL; ++source) {
            reset_clock(false, source, saved);
            clock_model.event = clock_model.other_event = clock_model.nvic_pending = true;
            if (source == NRF_CLOCK_LFCLK_RC) clock_pre_shutdown();
            else clock_switch(source);
            assert(clock_model.event && clock_model.other_event && clock_model.nvic_pending);
            assert(clock_model.starts == 0 && clock_model.stops == 0 && clock_model.event_clears == 0);
            assert(clock_model.other_irqs == 0 && !reboots);
            check_clock_release(saved, 0);
        }
        reset_clock(false, NRF_CLOCK_LFCLK_XTAL, saved);
        clock_model.running = clock_model.request = false;
        clock_pre_shutdown();
        assert(clock_model.running && clock_model.request && clock_model.active == NRF_CLOCK_LFCLK_RC);
        assert(clock_model.starts == 1 && clock_model.stops == 1 && !reboots);
        check_clock_release(saved, saved);
#if TEST_EXTERNAL
        for (unsigned int source = NRF_CLOCK_LFCLK_XTAL_FULL_SWING;
             source <= NRF_CLOCK_LFCLK_XTAL_LOW_SWING; ++source) {
            reset_clock(false, NRF_CLOCK_LFCLK_RC, saved);
            clock_switch(source);
            assert(clock_model.running && clock_model.active == NRF_CLOCK_LFCLK_XTAL);
            assert(clock_model.requested == source && clock_model.starts == 1 && !reboots);
            check_clock_release(saved, saved);
            reset_clock(false, NRF_CLOCK_LFCLK_XTAL, saved);
            clock_switch(source);
            assert(clock_model.starts == 0 && clock_model.stops == 0);
            check_clock_release(saved, 0);
        }
#endif
    }
}

static void check_late_external_init(void)
{
    unsigned int starts = clock_model.starts;
    unsigned int stops = clock_model.stops;
    unsigned int writes = clock_model.writes;
    unsigned int trace_count = clock_model.trace_count;
    clock_init_external();
    assert(clock_model.active == NRF_CLOCK_LFCLK_RC);
    assert(clock_model.starts == starts && clock_model.stops == stops);
    assert(clock_model.writes == writes && clock_model.trace_count == trace_count);
}

static void test_external_clock_contention(bool use_lfxo, bool init_first)
{
    nrf_clock_lfclk_t external = use_lfxo ? NRF_CLOCK_LFCLK_XTAL : NRF_CLOCK_LFCLK_SYNTH;
    reset_clock(true, init_first ? NRF_CLOCK_LFCLK_RC : external, false);
    host_clock_lfxo = use_lfxo;
    gate_enabled = true;
    const struct clock_call calls[] = {
        {1, NRF_CLOCK_LFCLK_RC, !init_first},
        {2, NRF_CLOCK_LFCLK_RC, init_first},
    };
    pthread_t owner, contender;
    assert(pthread_create(&owner, NULL, init_first ? call_external_init : call_clock,
                          (void *)&calls[0]) == 0);
    assert(pthread_mutex_lock(&rendezvous) == 0);
    while (!owner_waiting) assert(pthread_cond_wait(&changed, &rendezvous) == 0);
    assert(pthread_create(&contender, NULL, init_first ? call_clock : call_external_init,
                          (void *)&calls[1]) == 0);
    while (!contender_waiting) assert(pthread_cond_wait(&changed, &rendezvous) == 0);
    assert(clock_model.start_left != 0);
    owner_released = true;
    assert(pthread_cond_broadcast(&changed) == 0);
    assert(pthread_mutex_unlock(&rendezvous) == 0);
    assert(pthread_join(owner, NULL) == 0 && pthread_join(contender, NULL) == 0);

    assert(!reboots && clock_model.running && clock_model.request);
    assert(clock_model.active == NRF_CLOCK_LFCLK_RC);
    caller = 0;
    check_late_external_init();
    check_clock_release(false, 0);
}

static void test_external_clock_shutdown_order(void)
{
    for (unsigned int use_lfxo = 0; use_lfxo < 2; ++use_lfxo) {
        nrf_clock_lfclk_t external = use_lfxo ? NRF_CLOCK_LFCLK_XTAL : NRF_CLOCK_LFCLK_SYNTH;
        reset_clock(true, NRF_CLOCK_LFCLK_RC, false);
        host_clock_lfxo = use_lfxo;
        clock_init_external();
        assert(clock_model.active == external);
        clock_pre_shutdown();
        assert(clock_model.active == NRF_CLOCK_LFCLK_RC);
        check_late_external_init();

        reset_clock(true, NRF_CLOCK_LFCLK_RC, false);
        host_clock_lfxo = use_lfxo;
        clock_pre_shutdown();
        check_late_external_init();
        assert(!reboots && clock_model.running && clock_model.request);

        test_external_clock_contention(use_lfxo, true);
        test_external_clock_contention(use_lfxo, false);
    }
    puts("LF clock: shutdown fences completed, in-flight and queued external init");
}

int main(void)
{
    init_mutex(&clock_switch_lock);
    init_mutex(&sensor_lifecycle_lock);
#if ADAFRUIT_FAULT_RECOVERY
    recovery_supported = true;
    /* GPREGRET owns the streak even if retained RAM is stale or replaced. */
    for (unsigned count = 1; count <= 2; ++count) {
        watchdog_initialized = false;
        reset_was_wdt = true;
        power.GPREGRET = count == 1 ? 0xE1 : 0xE2;
        retained->watchdog_state.magic = WATCHDOG_STATE_MAGIC;
        retained->watchdog_state.reset_count = 250;
        retained->watchdog_state.total_wdt_resets = 17;
        retained->watchdog_state.last_failed_channel = 2;
        assert(watchdog_init() == 0);
        assert(power.GPREGRET == (count == 1 ? 0xE1u : 0xE2u));
        assert(retained->watchdog_state.reset_count == 250);
        assert(retained->watchdog_state.total_wdt_resets == 18);
        assert(retained->watchdog_state.last_failed_channel == 2 && !reboots);
        int calls = wdt_init_calls;
        assert(watchdog_init() == 0 && wdt_init_calls == calls);
        assert(retained->watchdog_state.total_wdt_resets == 18);
        watchdog_mark_boot_success();
        assert(power.GPREGRET == 0);
    }
    watchdog_initialized = false;
    retained->watchdog_state.magic = 0;
    power.GPREGRET = 0xE2;
    assert(watchdog_init() == 0 && power.GPREGRET == 0xE2);
    assert(retained->watchdog_state.total_wdt_resets == 1);
    watchdog_initialized = false;
    wdt_ready = false;
    assert(watchdog_init() == -ENODEV && !watchdog_initialized);
    assert(power.GPREGRET == 0xE2);
    wdt_ready = true;
    wdt_init_result = -EIO;
    assert(watchdog_init() == -EIO && !watchdog_initialized);
    assert(power.GPREGRET == 0xE2);
    wdt_init_result = 0;
    recovery_supported = false;
#endif
    /* Same Adafruit binary now takes the legacy retained-counter path. */
    for (unsigned marker = 0; marker < 256; ++marker) {
        power.GPREGRET = marker;
        retained->watchdog_state.reset_count = 2;
        assert(watchdog_get_reset_count() == 2);
        watchdog_clear_reset_count();
        assert(watchdog_get_reset_count() == 0 && power.GPREGRET == marker);
    }
    retained_updates = 0;
    reboots = 0;
    watchdog_initialized = false;
    reset_was_wdt = true;
    retained->watchdog_state.magic = WATCHDOG_STATE_MAGIC;
    retained->watchdog_state.reset_count = 1;
    retained->watchdog_state.total_wdt_resets = 17;
    assert(watchdog_init() == 0);
    assert(watchdog_get_reset_count() == 2 && !reboots);
    assert(retained->watchdog_state.total_wdt_resets == 18);
    watchdog_initialized = false;
    assert(watchdog_init() == 1);
    assert(watchdog_get_reset_count() == 0 && reboots == 1);
    assert(retained->watchdog_state.total_wdt_resets == 19 && retained_updates == 1);
    reboots = 0;
    retained->watchdog_state.reset_count = 2;
    watchdog_mark_boot_success();
    assert(watchdog_get_reset_count() == 0 && retained_updates == 2);
    reset_was_wdt = false;
    retained->watchdog_state.reset_count = 2;
    assert(watchdog_init() == 0 && watchdog_get_reset_count() == 0);
    watchdog_initialized = false;
    /* Start an actual consecutive sequence from invalid retained state. */
    retained->watchdog_state.magic = 0;
    reset_was_wdt = true;
    reboots = 0;
    for (unsigned count = 1; count <= 3; ++count) {
        watchdog_initialized = false;
        assert(watchdog_init() == (count == 3 ? 1 : 0));
        assert(retained->watchdog_state.total_wdt_resets == count);
        assert(watchdog_get_reset_count() == (count == 3 ? 0 : count));
        assert(reboots == (count == 3 ? 1 : 0));
    }
#if ADAFRUIT_FAULT_RECOVERY
    assert(power.GPREGRET == ADAFRUIT_DFU_MAGIC_UF2_RESET);
#endif
    /* A non-DOG reboot clears a streak without deleting cumulative telemetry. */
    watchdog_initialized = false;
    reset_was_wdt = false;
    retained->watchdog_state.reset_count = 2;
    assert(watchdog_init() == 0);
    assert(watchdog_get_reset_count() == 0);
    assert(retained->watchdog_state.total_wdt_resets == 3);
    test_clock_failures();
    test_clock_contention();
    test_clock_noop_and_shutdown();
    test_external_clock_shutdown_order();
    reboots = 0;

    /* Registration cases start before init, independently of boot-policy cases. */
    watchdog_initialized = false;
    for (int i = 0; i < WDT_CHANNEL_COUNT; i++) channel_ids[i] = -1;
    assert(watchdog_register_thread(0, 0) == -ENODEV && adds == 0 && feeds == 0);
    watchdog_initialized = true; add_result = -ENOMEM;
    assert(watchdog_register_thread(0, 1234) == -ENOMEM);
    assert(channel_ids[0] == -1 && channel_timeouts[0] == 0 && feeds == 0);
    add_result = 0;
    assert(watchdog_register_thread(0, 1234) == 0 && feeds == 1 && last_feed == 0);
    assert(watchdog_register_thread(0, 1234) == 0 && adds == 2);
    watchdog_pause(0); assert(channel_ids[0] == -1);
    watchdog_resume(0); assert(added_timeout == 1234 && channel_ids[0] == 0);
    watchdog_pause(0); add_result = -ENOMEM;
    watchdog_resume(0); assert(reboots == 1 && channel_ids[0] == -1);
    assert(watchdog_register_thread(-1, 0) == -EINVAL);
    add_result = 4;
    assert(watchdog_register_thread(WDT_CHANNEL_CALIBRATION, 0) == 4);
    assert(added_timeout == 60000 && last_feed == 4);
    add_result = 5;
    assert(watchdog_register_thread(WDT_CHANNEL_SCAN, 0) == 5);
    assert(added_timeout == 10000 && last_feed == 5);
    assert(watchdog_register_thread(WDT_CHANNEL_COUNT, 0) == -EINVAL);

    pm_error[PM_DEVICE_ACTION_SUSPEND][2] = -EIO;
    assert(sys_interface_suspend() == -EIO);
    assert(bus_suspended[1] && !bus_suspended[2]);
    assert(pm_calls[PM_DEVICE_ACTION_SUSPEND][1] == 1 && pm_calls[PM_DEVICE_ACTION_SUSPEND][2] == 1);
    pm_error[PM_DEVICE_ACTION_SUSPEND][2] = 0;
    assert(sys_interface_suspend() == 0 && bus_suspended[2]);
    pm_error[PM_DEVICE_ACTION_RESUME][2] = -EBUSY;
    assert(sensor_shutdown() == -EBUSY && shutdowns == 0);
    assert(sensor_setup_WOM() == -EBUSY && wom_calls == 0);
    pm_error[PM_DEVICE_ACTION_RESUME][2] = 0;
    pm_error[PM_DEVICE_ACTION_SUSPEND][1] = -EIO;
    assert(sensor_shutdown() == -EIO && shutdowns == 2);
    assert(sensor_setup_WOM() == -EIO && wom_calls == 1);
    pm_error[PM_DEVICE_ACTION_SUSPEND][1] = 0;
    assert(sensor_shutdown() == 0 && shutdowns == 4);
    assert(sensor_setup_WOM() == 0x12 && wom_calls == 2);
    assert(bus_suspended[1] && bus_suspended[2]);
    puts("platform lifecycle regression passed");
    return 0;
}
