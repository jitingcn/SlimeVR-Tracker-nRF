#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <errno.h>
#define LOG_INF(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define IS_ENABLED(x) (x)
#define CONFIG_CLOCK_USE_LFXO 1
#define CONFIG_SENSOR_TCAL_HEATED 0
#define LFCLK_WAIT_STEP_US 300
#define LFCLK_STOP_TIMEOUT_US 10000
#define LFCLK_START_TIMEOUT_US 1000000
#define SYS_REBOOT_COLD 0
static int reboots;
static void sys_reboot(int type) { reboots++; }
static unsigned int irq_lock(void) { return 0; }
static void irq_unlock(unsigned int key) {}
typedef enum { NRF_CLOCK_LFCLK_RC, NRF_CLOCK_LFCLK_XTAL } nrf_clock_lfclk_t;
#define NRF_CLOCK 0
#define NRF_CLOCK_DOMAIN_LFCLK 0
#define NRF_CLOCK_TASK_LFCLKSTOP 0
#define NRF_CLOCK_TASK_LFCLKSTART 1
#define NRF_CLOCK_EVENT_LFCLKSTARTED 0
static nrf_clock_lfclk_t active_source, requested_source;
static bool running, event, start_requested, watchdog_running, stop_stuck, fail_rc, fail_xtal;
static unsigned int stop_remaining, delays, source_writes, starts, stops, watchdog_rc_writes;
static unsigned int stop_request_remaining;
static inline bool nrf_clock_start_task_check(int clock, int domain)
{
    return start_requested;
}
static bool nrf_clock_is_running(int clock, int domain, nrf_clock_lfclk_t *source)
{
    *source = active_source;
    return running;
}
static void nrf_clock_task_trigger(int clock, int task)
{
    if (task == NRF_CLOCK_TASK_LFCLKSTOP) {
        stops++;
        stop_request_remaining = 1;
        stop_remaining = 3;
    } else {
        starts++;
        assert(!start_requested && (!running || active_source == NRF_CLOCK_LFCLK_RC));
        start_requested = true;
        bool failed = requested_source == NRF_CLOCK_LFCLK_RC ? fail_rc : fail_xtal;
        running = !failed || watchdog_running;
        active_source = failed ? NRF_CLOCK_LFCLK_RC : requested_source;
        event = !failed;
        stop_remaining = stop_request_remaining = 0;
    }
}
static void nrf_clock_lf_src_set(int clock, nrf_clock_lfclk_t source)
{
    assert(!start_requested);
    assert(!running || active_source == NRF_CLOCK_LFCLK_RC);
    if (running && watchdog_running) {
        watchdog_rc_writes++;
    }
    requested_source = source;
    source_writes++;
}
static void nrf_clock_event_clear(int clock, int which) { event = false; }
static bool nrf_clock_event_check(int clock, int which) { return event; }
static void nrfx_coredep_delay_us(uint32_t us)
{
    assert(us == LFCLK_WAIT_STEP_US);
    assert(++delays < 10000);
    if (stop_stuck) return;
    if (stop_request_remaining && --stop_request_remaining == 0) start_requested = false;
    if (stop_remaining && --stop_remaining == 0) {
        running = watchdog_running;
        if (watchdog_running) active_source = NRF_CLOCK_LFCLK_RC;
    }
}

static void reset_clock(bool watchdog, nrf_clock_lfclk_t source)
{
    running = start_requested = true;
    watchdog_running = watchdog;
    active_source = requested_source = source;
    event = stop_stuck = fail_rc = fail_xtal = false;
    stop_remaining = stop_request_remaining = delays = source_writes = starts = stops = 0;
    watchdog_rc_writes = 0;
    reboots = 0;
}

typedef int wdt_channel_id_t;
#define WDT_CHANNEL_COUNT 9
static int channel_ids[WDT_CHANNEL_COUNT];
static uint32_t channel_timeouts[WDT_CHANNEL_COUNT];
static const uint32_t default_timeouts[WDT_CHANNEL_COUNT] = {10000,10000,10000};
static const char *channel_names[WDT_CHANNEL_COUNT];
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
#include "production.inc"

int main(void)
{
    /* Normal STOP clears the software request before the oscillator settles. */
    reset_clock(false, NRF_CLOCK_LFCLK_XTAL);
    clock_switch(NRF_CLOCK_LFCLK_RC);
    assert(running && active_source == NRF_CLOCK_LFCLK_RC && source_writes == 1);
    assert(delays == 3 && starts == 1 && !reboots);

    reset_clock(false, NRF_CLOCK_LFCLK_RC);
    clock_switch(NRF_CLOCK_LFCLK_XTAL);
    assert(running && active_source == NRF_CLOCK_LFCLK_XTAL && start_requested);
    assert(source_writes == 1 && starts == 1 && !reboots);

    /* WDT keeps LFRC physically running after STOP has cleared LFCLKRUN. */
    reset_clock(true, NRF_CLOCK_LFCLK_RC);
    clock_switch(NRF_CLOCK_LFCLK_XTAL);
    assert(running && active_source == NRF_CLOCK_LFCLK_XTAL && start_requested);
    assert(source_writes == 1 && watchdog_rc_writes == 1 && starts == 1 && !reboots);
    assert(delays == 1);

    /* Shutdown XTAL -> RC must wait for the old XTAL to settle to forced RC. */
    reset_clock(true, NRF_CLOCK_LFCLK_XTAL);
    clock_switch(NRF_CLOCK_LFCLK_RC);
    assert(running && active_source == NRF_CLOCK_LFCLK_RC && start_requested);
    assert(delays == 3 && watchdog_rc_writes == 1 && starts == 1 && !reboots);

    /* RC in LFCLKSTAT alone cannot authorize a write while STOP is stuck. */
    reset_clock(true, NRF_CLOCK_LFCLK_RC);
    stop_stuck = true;
    clock_switch(NRF_CLOCK_LFCLK_XTAL);
    assert(reboots == 1 && source_writes == 0 && starts == 0 && stops == 2);
    assert(start_requested && running && active_source == NRF_CLOCK_LFCLK_RC);
    assert(delays == 2 * ((LFCLK_STOP_TIMEOUT_US + LFCLK_WAIT_STEP_US - 1) / LFCLK_WAIT_STEP_US));

    /* Oscillator failure still starts RC, with or without a WDT clock request. */
    for (int watchdog = 0; watchdog < 2; watchdog++) {
        reset_clock(watchdog, NRF_CLOCK_LFCLK_RC);
        fail_xtal = true;
        clock_switch(NRF_CLOCK_LFCLK_XTAL);
        assert(running && active_source == NRF_CLOCK_LFCLK_RC && start_requested);
        assert(starts == 2 && source_writes == 2 && stops == 2 && !reboots);
        assert(delays >= LFCLK_START_TIMEOUT_US / LFCLK_WAIT_STEP_US);
        if (watchdog) assert(watchdog_rc_writes == 2);

        /* Physical WDT RC does not replace a successful START acknowledgement. */
        reset_clock(watchdog, NRF_CLOCK_LFCLK_RC);
        fail_xtal = fail_rc = true;
        clock_switch(NRF_CLOCK_LFCLK_XTAL);
        assert(reboots == 1 && starts == 2 && source_writes == 2);
        assert(running == (bool)watchdog);
    }
    reboots = 0;

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
