#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "thread_priority.h"

/* These are firmware configuration/input choices, not altered function bodies.
 * No magnetometer, heater, or motion-history support; TWIM timeout is the 25 ms
 * selected in tracker prj.conf. Cold scan delay is the production 50 ms. */
#define CONFIG_DYNAMIC_ACTIVE_TIMEOUT 0
#define CONFIG_SENSOR_USE_MAGNETOMETER 0
#define CONFIG_SENSOR_TCAL_HEATED 0
#define CONFIG_SENSOR_VQF_CYCLE_BIAS_OBSERVATION 0
#define CONFIG_CONNECTION_RAW_COLLECTION 0
#define CONFIG_SENSOR_STARTUP_DISCARD_MS 100
#define IMU_INT_EXISTS 0
#define DEBUG 0
#define STATUS_INTERVAL_MS 10000
#define SENSOR_IMU_SPI_EXISTS 0
#define SENSOR_IMU_EXISTS 1
#define SENSOR_SCAN_COLD_POWER_UP_DELAY_MS 50
#define CONFIG_SENSOR_RETAINED_SCAN_POWER_UP_DELAY_MS 5
#define I2C_TRANSFER_TIMEOUT_MSEC K_MSEC(25)
#define SCAN_ADDR_START 8
#define SCAN_ADDR_STOP 0x7e
#define FORCE_SCAN_THRESHOLD 3
#define FORCE_SCAN_WINDOW_MS 60000
#define SENSOR_LIFE_IDLE BIT(0)
#define WDT_CHANNEL_SENSOR 0
#define WDT_CHANNEL_SCAN 8
#define SYS_SENSOR_FAULT_OTHER 1
#define SYS_SENSOR_FAULT_MISSING 2
#define SYS_SENSOR_FAULT_NONE 0
#define SYS_REBOOT_COLD 0
#define TRACKER_REST_SUSPENDED 1
#define TRACKER_REST_INITIALIZING 3
#define TRACKER_REST_RESET 2
#define LED_OWNER_SENSOR 1
#define LED_ACCEPTED 1
#define LED_MAINTENANCE 2
#define LED_INITIALIZING 3
#define LED_FAILED 4
#define LED_SUCCESS 5
#define SENSOR_LIFECYCLE_LOCK_TIMEOUT_MS 80
#define SENSOR_ACTIVITY_TIMEOUT_MS 60
#define LED_NONE 0
#define LOG_INF(...) do {} while (0)
#define LOG_WRN(...) do {} while (0)
#define LOG_ERR(...) do {} while (0)
#define LOG_DBG(...) do {} while (0)

/* Only Nordic peripheral descriptors and physical/DMA/PM leaves are modeled.
 * The production TWIM data struct, all four TWIM functions, scan functions and
 * lifecycle functions are extracted verbatim at build time. All k_thread,
 * k_event, k_sem, k_sleep and k_thread_join operations are the Zephyr kernel. */
typedef struct { int unused; } nrfx_twim_t;
typedef struct {
    int type;
    struct { int type; uint8_t *p_primary_buf; size_t primary_length; } xfer_desc;
} nrfx_twim_event_t;
#define NRFX_TWIM_XFER_TX 0
#define NRFX_TWIM_XFER_RX 1
#define NRFX_TWIM_EVT_DONE 0
#define NRFX_TWIM_EVT_ADDRESS_NACK 1
#define NRFX_TWIM_EVT_DATA_NACK 2
#include "twim_data.inc"
struct i2c_nrfx_twim_common_config {
    uint8_t *msg_buf;
    uint16_t msg_buf_size;
    void *mem_reg;
};
static uint8_t concat_buf[16];
static struct i2c_nrfx_twim_data twim_data;
static const struct i2c_nrfx_twim_common_config twim_config = {
    .msg_buf = concat_buf, .msg_buf_size = sizeof(concat_buf), .mem_reg = NULL,
};
static const struct device twim_device = {
    .name = "modeled-twim", .config = &twim_config, .data = &twim_data,
};

static struct k_sem completion_entered;
static atomic_t acquire_attempts, acquire_successes, release_calls, completion_waits;
static atomic_t completion_timeouts, bus_starts, callback_count, recover_count;
static bool hold_first_irq;
static void event_handler(nrfx_twim_event_t const *event, void *context);

/* Wrappers observe ownership boundaries; semaphore operations are real. */
static int observed_sem_take(struct k_sem *sem, k_timeout_t timeout)
{
    bool acquire = sem == &twim_data.transfer_sync && K_TIMEOUT_EQ(timeout, K_FOREVER);
    bool completion = sem == &twim_data.completion_sync && K_TIMEOUT_EQ(timeout, K_MSEC(25));
    if (acquire) atomic_inc(&acquire_attempts);
    if (completion) {
        atomic_inc(&completion_waits);
        k_sem_give(&completion_entered);
    }
    int rc = k_sem_take(sem, timeout);
    if (acquire && rc == 0) atomic_inc(&acquire_successes);
    if (completion && rc != 0) atomic_inc(&completion_timeouts);
    return rc;
}
static void observed_sem_give(struct k_sem *sem)
{
    if (sem == &twim_data.transfer_sync) atomic_inc(&release_calls);
    if (sem == &twim_data.completion_sync) atomic_inc(&callback_count);
    k_sem_give(sem);
}
static int smoke_pm_get(const struct device *dev) { ARG_UNUSED(dev); return 0; }
static int smoke_pm_put(const struct device *dev) { ARG_UNUSED(dev); return 0; }
static bool nrf_dma_accessible_check(const nrfx_twim_t *twim, const void *buf)
{ ARG_UNUSED(twim); ARG_UNUSED(buf); return true; }
static int dmm_buffer_in_prepare(void *reg, void *buf, size_t len, void **dma)
{ ARG_UNUSED(reg); ARG_UNUSED(len); *dma = buf; return 0; }
static int dmm_buffer_out_prepare(void *reg, void *buf, size_t len, void **dma)
{ ARG_UNUSED(reg); ARG_UNUSED(len); *dma = buf; return 0; }
static void dmm_buffer_out_release(void *reg, void **dma)
{ ARG_UNUSED(reg); ARG_UNUSED(dma); }
static void dmm_buffer_in_release(void *reg, void *buf, size_t len, void *dma)
{ ARG_UNUSED(reg); ARG_UNUSED(buf); ARG_UNUSED(len); ARG_UNUSED(dma); }
static int i2c_nrfx_twim_recover_bus(const struct device *dev)
{ ARG_UNUSED(dev); atomic_inc(&recover_count); return 0; }
static int i2c_nrfx_twim_msg_transfer(const struct device *dev, uint8_t flags,
                                       uint8_t *buf, size_t len, uint16_t addr);

#define k_sem_take observed_sem_take
#define k_sem_give observed_sem_give
#define pm_device_runtime_get smoke_pm_get
#define pm_device_runtime_put smoke_pm_put
#include "twim_funcs.inc"
#undef pm_device_runtime_put
#undef pm_device_runtime_get
#undef k_sem_give
#undef k_sem_take

/* Only the physical peripheral is modeled. A held completion exposes the
 * real TWIM semaphore ownership and the 25 ms driver's timeout. */
static void synthetic_done(const struct device *dev, uint8_t flags, uint8_t *buf, size_t len)
{
    nrfx_twim_event_t event = {
        .type = NRFX_TWIM_EVT_DONE,
        .xfer_desc = { .type = (flags & I2C_MSG_READ) ? NRFX_TWIM_XFER_RX : NRFX_TWIM_XFER_TX,
                       .p_primary_buf = buf, .primary_length = len },
    };
    event_handler(&event, (void *)dev);
}
static int i2c_nrfx_twim_msg_transfer(const struct device *dev, uint8_t flags,
                                       uint8_t *buf, size_t len, uint16_t addr)
{
    ARG_UNUSED(addr);
    int n = atomic_inc(&bus_starts) + 1;
    if (!(hold_first_irq && n == 1)) synthetic_done(dev, flags, buf, len);
    return 0;
}

/* Map Zephyr register-I/O leaves onto the real extracted TWIM transfer. The
 * unused special BMM register-write / burst-read paths deliberately return an
 * error. The fixture's actual extracted discovery table uses neither path. */
static int smoke_reg_read(const struct device *dev, uint16_t addr, uint8_t reg, uint8_t *val)
{
    struct i2c_msg msg[] = {
        { .buf = &reg, .len = 1, .flags = I2C_MSG_WRITE },
        { .buf = val, .len = 1, .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP },
    };
    return i2c_nrfx_twim_transfer(dev, msg, 2, addr);
}
static int smoke_reg_write(const struct device *dev, uint16_t addr, uint8_t reg, uint8_t value)
{ ARG_UNUSED(dev); ARG_UNUSED(addr); ARG_UNUSED(reg); ARG_UNUSED(value); return -EIO; }
static int smoke_burst_read(const struct device *dev, uint16_t addr, uint8_t reg,
                            uint8_t *buf, uint32_t count)
{ ARG_UNUSED(dev); ARG_UNUSED(addr); ARG_UNUSED(reg); ARG_UNUSED(buf); ARG_UNUSED(count); return -EIO; }
#define i2c_reg_read_byte smoke_reg_read
#define i2c_reg_write_byte smoke_reg_write
#define i2c_burst_read smoke_burst_read
#include "scan_funcs.inc"
#undef i2c_burst_read
#undef i2c_reg_write_byte
#undef i2c_reg_read_byte

struct led_token { unsigned int id; };
struct imu_stub { void (*shutdown)(void); int (*setup_WOM)(void); };
struct fusion_stub { void (*take_rest_observation)(void *); };
static int shutdown_calls, wom_calls;
static void imu_shutdown(void) { shutdown_calls++; }
static int imu_setup_wom(void) { wom_calls++; return 0x31; }
static struct imu_stub sensor_imu_none, mock_imu = {
    .shutdown = imu_shutdown, .setup_WOM = imu_setup_wom,
};
static struct imu_stub *sensor_imu = &sensor_imu_none;
static struct imu_stub *sensor_imus[] = { &mock_imu };
static const char *dev_imu_names[] = { "modeled IMU" };
static struct fusion_stub fusion;
static struct fusion_stub *sensor_fusion = &fusion;
static struct i2c_dt_spec sensor_imu_dev = { .bus = &twim_device, .addr = 0 };
static uint8_t sensor_imu_dev_reg = 0xff;
static int sensor_imu_id;
static bool sensor_sensor_init, main_running, main_ok;
static bool no_packets_timeout_logged, sensor_wom_fast_wake_resume_pending;
static int packet_errors, force_scan_request_count, sensor_scan_last_power_up_delay_ms;
static int64_t force_scan_request_times[FORCE_SCAN_THRESHOLD], last_sensor_send_time;
static atomic_t main_suspended, output_ready, sensor_loop_stop, loop_active, loop_starts;
static struct led_token initialization_feedback;
static struct k_event sensor_life_events;
static struct k_mutex sensor_lifecycle_lock, sensor_activity_lock;
static struct k_mutex power_plan_lock;
static struct k_sem sensor_loop_gate;
static k_tid_t sensor_suspend_owner;
enum sensor_thread_role { SENSOR_THREAD_NONE, SENSOR_THREAD_SCAN, SENSOR_THREAD_LOOP };
static enum sensor_thread_role sensor_thread_role;
static struct k_thread sensor_thread_id, requester1, requester2, requester3;
static const k_tid_t power_thread_id = &requester3;
static K_THREAD_STACK_DEFINE(sensor_thread_id_stack, 4096);
static K_THREAD_STACK_DEFINE(requester1_stack, 4096);
static K_THREAD_STACK_DEFINE(requester2_stack, 4096);
static K_THREAD_STACK_DEFINE(requester3_stack, 4096);
static struct k_sem loop_entered, loop_hold, loop_progress, done1, done2, done3;
static struct k_sem suspended, allow_resume, scan_resume_entered, scan_resume_gate;
static struct k_sem loop_exit_waiting, loop_exit_gate, self_done;
static struct k_sem power_acquired, allow_power_resume;
static bool requester1_created, requester2_created, requester3_created, wdt_scan_active, wdt_sensor_active;
static bool hold_loop_init, hold_scan_resume, shutdown_on_suspend, hold_loop_exit, self_test, power_setup_wom;
static bool recoverable_failure;
static int heater_stop_error, heater_stop_attempts;
static int scan_suspend_error, shutdown_result, result1, result2, result3, self_scan_result, self_suspend_result;
static float processing_work_time_ema_ms, sensor_window_fused_angle_rad;
static int64_t max_loop_time, last_status_time, sensor_startup_discard_until_ms;
static bool sensor_startup_discard_logged;
static int sensor_update_time_ms = 5;
static uint32_t sensor_int_timeouts, sensor_window_iters, sensor_window_publishes, sensor_window_ints;
static uint64_t sensor_window_proc_us, sensor_window_wait_us, sensor_window_acq_us, sensor_window_vqf_us;
static uint64_t sensor_window_packets, sensor_window_acq_max_us, sensor_window_proc_max_us;
static uint64_t sensor_window_resume_max_us, sensor_window_fifo_max_us;
typedef struct { uint32_t sensor_epoch; int packets; } sensor_loop_frame_t;

static int sensor_scan(void);
static void sensor_scan_thread(void);
static void sensor_loop(void);
static int main_imu_suspend(void);
static void main_imu_resume(void);
static int sensor_request_scan(bool force, bool feedback);
static int watchdog_register_thread(int channel, unsigned int timeout)
{
    ARG_UNUSED(timeout);
    if (channel == WDT_CHANNEL_SCAN) wdt_scan_active = true;
    if (channel == WDT_CHANNEL_SENSOR) wdt_sensor_active = true;
    return 0;
}
static void watchdog_pause(int channel)
{
    if (channel == WDT_CHANNEL_SCAN) wdt_scan_active = false;
    if (channel == WDT_CHANNEL_SENSOR) wdt_sensor_active = false;
}
static void watchdog_resume(int channel)
{ if (channel == WDT_CHANNEL_SENSOR) wdt_sensor_active = true; }
static int sys_interface_resume(void)
{
    if (hold_scan_resume && sensor_thread_role == SENSOR_THREAD_SCAN) {
        k_sem_give(&scan_resume_entered);
        k_sem_take(&scan_resume_gate, K_FOREVER);
    }
    return 0;
}
static int sys_interface_suspend(void) { return scan_suspend_error; }
static void sys_reboot(int kind) { ARG_UNUSED(kind); }
static void watchdog_feed(int channel) { ARG_UNUSED(channel); }
static void set_sensor_fault_if_unset(int fault) { ARG_UNUSED(fault); }
static bool sensor_calibration_imu_ready(void) { return false; }
static void sensor_apply_calibration_frame(void) {}
static uint32_t tracker_events_sensor_epoch(void) { return 1; }
static void sensor_loop_acquire(sensor_loop_frame_t *frame) { ARG_UNUSED(frame); }
static void sensor_motion_prepare(uint32_t epoch, int64_t now) { ARG_UNUSED(epoch); ARG_UNUSED(now); }
static bool sensor_motion_frame_current(uint32_t epoch) { ARG_UNUSED(epoch); return true; }
static void sensor_loop_process_fifo(sensor_loop_frame_t *frame) { ARG_UNUSED(frame); }
static void sensor_loop_check_packets(sensor_loop_frame_t *frame, int64_t now) { ARG_UNUSED(frame); ARG_UNUSED(now); }
static void sensor_loop_publish(sensor_loop_frame_t *frame) { ARG_UNUSED(frame); }
static void set_sensor_fault(int fault) { ARG_UNUSED(fault); }
static void sensor_scan_read(void) {}
static void sensor_scan_write(void) {}
static void sensor_scan_clear(void) {}
static bool sensor_consume_wom_fast_wake_hint(void) { return false; }
static int set_sensor_clock(bool enabled, int rate, float *actual)
{ ARG_UNUSED(enabled); ARG_UNUSED(rate); *actual = 0; return 0; }
static int sensor_scan_retry_delay_ms(void) { return 5; }
static void sensor_interface_register_sensor_imu_i2c(struct i2c_dt_spec *dev) { ARG_UNUSED(dev); }
static void sensor_refresh_sensor_ids(void) {}
static int sensor_scan_imu(struct i2c_dt_spec *dev, uint8_t *reg)
{
    static const uint8_t addresses[] = {1, 0x20}, regs[] = {1, 0x0f}, ids[] = {1, 0x00};
    static const int names[] = {0};
    return sensor_scan_i2c(dev, reg, 1, addresses, regs, ids, names);
}
static struct led_token led_begin(int owner, int request)
{ ARG_UNUSED(owner); ARG_UNUSED(request); return (struct led_token){1}; }
static int led_request_id(void) { return 1; }
static int sensor_operation_result(int owner, int value, bool success)
{ ARG_UNUSED(owner); ARG_UNUSED(success); return value; }
static void sensor_calibration_result(struct led_token token, int result)
{ ARG_UNUSED(token); ARG_UNUSED(result); }
static void sensor_calibration_stage(struct led_token token, int stage)
{ ARG_UNUSED(token); ARG_UNUSED(stage); }
static void sensor_calibration_set_consumer_ready(bool ready) { ARG_UNUSED(ready); }
static void sys_cancel_WOM(void)
{
    k_mutex_lock(&power_plan_lock, K_FOREVER);
    k_mutex_unlock(&power_plan_lock);
}
static void tracker_events_sensor_invalidate(int reason) { ARG_UNUSED(reason); }
static void tracker_events_notify(void) {}

#include "lifecycle_funcs.inc"

/* Compile the same production suspend body with heated TCal enabled, while
 * modeling only its heater-off leaf, to exercise failed-restart admission. */
static int sensor_temperature_invalidate(void)
{
    heater_stop_attempts++;
    return heater_stop_error;
}
static void sensor_temperature_resume(void) {}
#undef CONFIG_SENSOR_TCAL_HEATED
#define CONFIG_SENSOR_TCAL_HEATED 1
#include "sensor_suspend_heated.inc"
#undef CONFIG_SENSOR_TCAL_HEATED
#define CONFIG_SENSOR_TCAL_HEATED 0

/* Actual sensor_loop and wait functions follow. Only initialization and
 * hardware/fusion leaves are modeled; their lock boundaries remain real. */
static int sensor_init(void)
{
    atomic_inc(&loop_starts);
    atomic_set(&loop_active, 1);
    if (self_test) {
        self_scan_result = sensor_request_scan(true, false);
        self_suspend_result = main_imu_suspend();
        k_sem_give(&self_done);
    }
    k_sem_give(&loop_entered);
    if (hold_loop_init) k_sem_take(&loop_hold, K_FOREVER);
    k_mutex_lock(&power_plan_lock, K_FOREVER);
    k_mutex_unlock(&power_plan_lock);
    atomic_clear(&loop_active);
    return 0;
}

#include "sensor_wait.inc"
static void observed_loop_wait(int64_t time_begin)
{
    sensor_loop_wait(time_begin);
    k_sem_give(&loop_progress);
}
static void observed_loop_leave(void)
{
    sensor_loop_leave();
    if (hold_loop_exit) {
        hold_loop_exit = false;
        k_sem_give(&loop_exit_waiting);
        k_sem_take(&loop_exit_gate, K_FOREVER);
    }
}
#define sensor_loop_wait observed_loop_wait
#define sensor_loop_leave observed_loop_leave
#include "sensor_loop.inc"
#undef sensor_loop_leave
#undef sensor_loop_wait
static void request1_entry(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    result1 = sensor_request_scan(true, false);
    k_sem_give(&done1);
}
static void request2_entry(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    result2 = sensor_request_scan(true, true);
    k_sem_give(&done2);
}
static void request3_entry(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    result3 = sensor_request_scan(true, true);
    k_sem_give(&done3);
}
static void power_entry(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    int err = main_imu_suspend();
    result3 = err;
    if (!err && power_setup_wom) result3 = sensor_setup_WOM();
    k_sem_give(&power_acquired);
    k_sem_take(&allow_power_resume, K_FOREVER);
    if (!err) main_imu_resume();
    k_sem_give(&done3);
}
static void suspend_entry(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    result2 = main_imu_suspend();
    if (!result2 && shutdown_on_suspend) shutdown_result = sensor_shutdown();
    if (!result2 && recoverable_failure) sensor_release_failed_restart();
    k_sem_give(&suspended);
    if (!result2) {
        k_sem_take(&allow_resume, K_FOREVER);
        if (!shutdown_on_suspend && !recoverable_failure) main_imu_resume();
    }
    k_sem_give(&done2);
}
static void start_first(void)
{
    requester1_created = true;
    k_thread_create(&requester1, requester1_stack, K_THREAD_STACK_SIZEOF(requester1_stack),
                    request1_entry, NULL, NULL, NULL, SENSOR_REQUEST_SCAN_THREAD_PRIORITY, 0, K_NO_WAIT);
}
static void start_second(k_thread_entry_t entry, int priority)
{
    requester2_created = true;
    k_thread_create(&requester2, requester2_stack, K_THREAD_STACK_SIZEOF(requester2_stack),
                    entry, NULL, NULL, NULL, priority, 0, K_NO_WAIT);
}
static void start_third(void)
{
    requester3_created = true;
    k_thread_create(&requester3, requester3_stack, K_THREAD_STACK_SIZEOF(requester3_stack),
                    request3_entry, NULL, NULL, NULL, CONSOLE_THREAD_PRIORITY, 0, K_NO_WAIT);
}
static void start_power(void)
{
    requester3_created = true;
    k_thread_create(&requester3, requester3_stack, K_THREAD_STACK_SIZEOF(requester3_stack),
                    power_entry, NULL, NULL, NULL, POWER_THREAD_PRIORITY, 0, K_NO_WAIT);
}
static void before(void *fixture)
{
    ARG_UNUSED(fixture);
    requester1_created = requester2_created = requester3_created = false;
    k_sem_init(&twim_data.transfer_sync, 1, 1);
    k_sem_init(&twim_data.completion_sync, 0, 1);
    k_sem_init(&completion_entered, 0, 1);
    k_sem_init(&loop_entered, 0, 1);
    k_sem_init(&loop_hold, 0, 1);
    k_sem_init(&loop_progress, 0, 1);
    k_sem_init(&done1, 0, 1);
    k_sem_init(&done2, 0, 1);
    k_sem_init(&done3, 0, 1);
    k_sem_init(&suspended, 0, 1);
    k_sem_init(&allow_resume, 0, 1);
    k_sem_init(&scan_resume_entered, 0, 1);
    k_sem_init(&scan_resume_gate, 0, 1);
    k_sem_init(&loop_exit_waiting, 0, 1);
    k_sem_init(&loop_exit_gate, 0, 1);
    k_sem_init(&self_done, 0, 1);
    k_sem_init(&power_acquired, 0, 1);
    k_sem_init(&allow_power_resume, 0, 1);
    k_mutex_init(&sensor_lifecycle_lock);
    k_mutex_init(&sensor_activity_lock);
    k_mutex_init(&power_plan_lock);
    k_sem_init(&sensor_loop_gate, 0, 1);
    k_event_init(&sensor_life_events);
    k_event_set(&sensor_life_events, SENSOR_LIFE_IDLE);
    atomic_clear(&acquire_attempts); atomic_clear(&acquire_successes);
    atomic_clear(&release_calls); atomic_clear(&completion_waits);
    atomic_clear(&completion_timeouts); atomic_clear(&bus_starts);
    atomic_clear(&callback_count); atomic_clear(&recover_count);
    atomic_clear(&main_suspended); atomic_clear(&output_ready);
    atomic_clear(&sensor_loop_stop); atomic_clear(&loop_active); atomic_clear(&loop_starts);
    sensor_thread_role = SENSOR_THREAD_NONE;
    sensor_suspend_owner = NULL;
    sensor_sensor_init = main_running = main_ok = false;
    no_packets_timeout_logged = sensor_wom_fast_wake_resume_pending = false;
    sensor_imu_dev.addr = 0; sensor_imu_dev_reg = 0xff; sensor_imu = &sensor_imu_none;
    packet_errors = force_scan_request_count = 0;
    memset(force_scan_request_times, 0, sizeof(force_scan_request_times));
    last_sensor_send_time = 0;
    initialization_feedback = (struct led_token){0};
    wdt_scan_active = wdt_sensor_active = hold_first_irq = false;
    hold_loop_init = hold_scan_resume = shutdown_on_suspend = hold_loop_exit = self_test = power_setup_wom = false;
    recoverable_failure = false;
    scan_suspend_error = shutdown_calls = wom_calls = 0;
    heater_stop_error = heater_stop_attempts = 0;
    shutdown_result = result1 = result2 = result3 = self_scan_result = self_suspend_result = -99;
}
static void after(void *fixture)
{
    ARG_UNUSED(fixture);
    if (requester1_created) k_thread_abort(&requester1);
    if (requester2_created) k_thread_abort(&requester2);
    if (requester3_created) k_thread_abort(&requester3);
    k_sem_give(&loop_exit_gate);
    if (sensor_thread_role == SENSOR_THREAD_LOOP) {
        atomic_set(&sensor_loop_stop, true);
        k_sem_give(&loop_hold);
        k_sem_give(&sensor_loop_gate);
        k_thread_join(&sensor_thread_id, K_SECONDS(1));
    } else if (sensor_thread_role == SENSOR_THREAD_SCAN) {
        k_thread_abort(&sensor_thread_id);
    }
}
static void await_first_owner(void)
{
    zassert_equal(k_sem_take(&completion_entered, K_SECONDS(1)), 0);
    zassert_equal(sensor_thread_role, SENSOR_THREAD_SCAN);
    zassert_false(sensor_sensor_init);
    zassert_false(main_running);
    zassert_equal(k_sem_count_get(&twim_data.transfer_sync), 0);
    zassert_equal(atomic_get(&acquire_successes), 1);
    zassert_equal(atomic_get(&release_calls), 0);
}

ZTEST(sensor_lifecycle, test_healthy_force_guard_and_override)
{
    sensor_sensor_init = main_ok = true;
    last_sensor_send_time = k_uptime_get();
    zassert_equal(sensor_request_scan(true, true), 0);
    zassert_equal(sensor_request_scan(true, true), 0);
    zassert_equal(force_scan_request_count, 2);
    zassert_equal(atomic_get(&acquire_attempts), 0);
    zassert_equal(sensor_request_scan(true, true), 0);
    zassert_equal(force_scan_request_count, 0);
    zassert_true(atomic_get(&acquire_successes) > 0);
    zassert_equal(atomic_get(&acquire_successes), atomic_get(&release_calls));
}

static void overlap_scenario(int priority)
{
    hold_first_irq = true;
    start_first();
    await_first_owner();
    packet_errors = 1; /* Exercise a second real scan, not the healthy guard. */
    start_second(request2_entry, priority);
    k_msleep(1);
    zassert_equal(k_sem_take(&done1, K_NO_WAIT), -EBUSY);
    zassert_equal(k_sem_take(&done2, K_NO_WAIT), -EBUSY);
    zassert_equal(k_sem_count_get(&twim_data.transfer_sync), 0);
    zassert_equal(atomic_get(&acquire_attempts), 1);
    zassert_true(wdt_scan_active);
    synthetic_done(&twim_device, I2C_MSG_WRITE, NULL, 0);
    zassert_equal(k_sem_take(&done1, K_SECONDS(2)), 0);
    zassert_equal(k_sem_take(&done2, K_SECONDS(2)), 0);
    zassert_equal(result1, 0);
    zassert_equal(result2, 0);
    zassert_true(atomic_get(&acquire_successes) >= 4);
    zassert_equal(atomic_get(&acquire_successes), atomic_get(&release_calls));
    zassert_equal(k_sem_count_get(&twim_data.transfer_sync), 1);
}
ZTEST(sensor_lifecycle, test_remote_overlap) { overlap_scenario(ESB_THREAD_PRIORITY); }
ZTEST(sensor_lifecycle, test_console_overlap) { overlap_scenario(CONSOLE_THREAD_PRIORITY); }

ZTEST(sensor_lifecycle, test_suspend_waits_for_scan_and_rejects_queued_restart)
{
    hold_first_irq = true;
    start_first();
    await_first_owner();
    start_second(suspend_entry, ESB_THREAD_PRIORITY);
    start_third();
    k_msleep(1);
    zassert_equal(k_sem_take(&suspended, K_NO_WAIT), -EBUSY);
    zassert_equal(k_sem_count_get(&twim_data.transfer_sync), 0);
    synthetic_done(&twim_device, I2C_MSG_WRITE, NULL, 0);
    zassert_equal(k_sem_take(&done1, K_SECONDS(2)), 0);
    zassert_equal(k_sem_take(&suspended, K_SECONDS(2)), 0);
    zassert_equal(k_sem_take(&done3, K_SECONDS(2)), 0);
    zassert_equal(result3, -EBUSY);
    zassert_equal(result2, 0);
    zassert_true(atomic_get(&main_suspended));
    zassert_equal(k_sem_count_get(&twim_data.transfer_sync), 1);
    zassert_equal(atomic_get(&loop_active), 0);
    int starts = atomic_get(&loop_starts);
    zassert_equal(starts, 0, "suspend should win before loop initialization");
    zassert_false(main_running);
    zassert_equal(sensor_request_scan(true, true), -EBUSY);
    zassert_equal(main_imu_suspend(), -EBUSY);
    main_imu_resume();
    zassert_true(atomic_get(&main_suspended));
    k_msleep(5);
    zassert_equal(atomic_get(&loop_starts), starts);
    k_sem_give(&allow_resume);
    zassert_equal(k_sem_take(&done2, K_SECONDS(2)), 0);
    zassert_false(atomic_get(&main_suspended));
    zassert_equal(k_sem_take(&loop_progress, K_SECONDS(1)), 0);
}

ZTEST(sensor_lifecycle, test_suspend_waits_for_initialization)
{
    hold_loop_init = true;
    start_first();
    zassert_equal(k_sem_take(&done1, K_SECONDS(2)), 0);
    zassert_equal(k_sem_take(&loop_entered, K_SECONDS(1)), 0);
    zassert_true(main_running);
    start_second(suspend_entry, ESB_THREAD_PRIORITY);
    k_msleep(1);
    zassert_equal(k_sem_take(&suspended, K_NO_WAIT), -EBUSY);
    zassert_true(wdt_sensor_active);
    /* Suspend must not retain the modeled power gate while waiting on the
     * activity owner, or the owner could not reach its release point. */
    zassert_equal(k_mutex_lock(&power_plan_lock, K_NO_WAIT), 0);
    k_mutex_unlock(&power_plan_lock);
    k_sem_give(&loop_hold);
    zassert_equal(k_sem_take(&suspended, K_SECONDS(1)), 0);
    zassert_equal(result2, 0);
    zassert_equal(atomic_get(&loop_active), 0);
    zassert_false(wdt_sensor_active);
    k_sem_give(&allow_resume);
    zassert_equal(k_sem_take(&done2, K_SECONDS(1)), 0);
    zassert_equal(k_sem_take(&loop_progress, K_SECONDS(1)), 0);
    zassert_true(wdt_sensor_active);
}

ZTEST(sensor_lifecycle, test_activity_timeout_preserves_watchdog)
{
    hold_loop_init = true;
    start_first();
    zassert_equal(k_sem_take(&done1, K_SECONDS(2)), 0);
    zassert_equal(k_sem_take(&loop_entered, K_SECONDS(1)), 0);
    zassert_equal(main_imu_suspend(), -ETIMEDOUT);
    zassert_false(atomic_get(&main_suspended));
    zassert_true(wdt_sensor_active);
    zassert_equal(atomic_get(&loop_active), 1);
    k_sem_give(&loop_hold);
    zassert_equal(k_sem_take(&loop_progress, K_SECONDS(1)), 0);
}

ZTEST(sensor_lifecycle, test_sensor_thread_self_requests_are_rejected)
{
    self_test = true;
    start_first();
    zassert_equal(k_sem_take(&done1, K_SECONDS(2)), 0);
    zassert_equal(k_sem_take(&self_done, K_SECONDS(1)), 0);
    zassert_equal(self_scan_result, -EDEADLK);
    zassert_equal(self_suspend_result, -EDEADLK);
    zassert_false(atomic_get(&main_suspended));
    zassert_equal(k_sem_take(&loop_progress, K_SECONDS(1)), 0);
}

ZTEST(sensor_lifecycle, test_join_timeout_never_reuses_live_thread)
{
    hold_loop_exit = true;
    start_first();
    zassert_equal(k_sem_take(&done1, K_SECONDS(2)), 0);
    zassert_equal(k_sem_take(&loop_exit_waiting, K_SECONDS(1)), 0);
    int acquisitions = atomic_get(&acquire_successes);
    packet_errors = 1; /* Bypass the healthy force-scan guard for this test. */
    start_second(request2_entry, CONSOLE_THREAD_PRIORITY);
    zassert_equal(k_sem_take(&done2, K_SECONDS(1)), 0);
    zassert_equal(result2, -ETIMEDOUT);
    zassert_equal(sensor_thread_role, SENSOR_THREAD_LOOP);
    zassert_equal(atomic_get(&loop_starts), 1);
    zassert_equal(atomic_get(&acquire_successes), acquisitions);
    zassert_false(atomic_get(&main_suspended));
    zassert_true(wdt_sensor_active);
    k_sem_give(&loop_exit_gate);
    zassert_equal(k_sem_take(&loop_progress, K_SECONDS(1)), 0);
}

ZTEST(sensor_lifecycle, test_scan_wait_timeout_preserves_scan_watchdog)
{
    hold_scan_resume = true;
    start_first();
    zassert_equal(k_sem_take(&scan_resume_entered, K_SECONDS(1)), 0);
    zassert_true(wdt_scan_active);
    zassert_equal(main_imu_suspend(), -ETIMEDOUT);
    zassert_false(atomic_get(&main_suspended));
    zassert_true(wdt_scan_active);
    k_sem_give(&scan_resume_gate);
    zassert_equal(k_sem_take(&done1, K_SECONDS(2)), 0);
    zassert_equal(result1, 0);
}


ZTEST(sensor_lifecycle, test_unaborted_twim_timeout_releases_owner)
{
    hold_first_irq = true;
    start_first();
    zassert_equal(k_sem_take(&done1, K_SECONDS(2)), 0);
    zassert_equal(result1, 0);
    zassert_equal(atomic_get(&completion_timeouts), 1);
    zassert_equal(atomic_get(&recover_count), 1);
    zassert_equal(atomic_get(&acquire_successes), atomic_get(&release_calls));
    zassert_equal(k_sem_count_get(&twim_data.transfer_sync), 1);
}

ZTEST(sensor_lifecycle, test_nonterminal_restart_failure_can_be_rescanned)
{
    start_first();
    zassert_equal(k_sem_take(&done1, K_SECONDS(2)), 0);
    zassert_equal(result1, 0);
    recoverable_failure = true;
    start_second(suspend_entry, ESB_THREAD_PRIORITY);
    zassert_equal(k_sem_take(&suspended, K_SECONDS(1)), 0);
    zassert_equal(result2, 0);
    zassert_true(atomic_get(&main_suspended));
    zassert_is_null(sensor_suspend_owner);
    int previous = atomic_get(&acquire_successes);
    zassert_equal(sensor_request_scan(true, true), 0);
    zassert_false(atomic_get(&main_suspended));
    zassert_true(atomic_get(&acquire_successes) > previous);
    k_sem_give(&allow_resume);
    zassert_equal(k_sem_take(&done2, K_SECONDS(1)), 0);
}

ZTEST(sensor_lifecycle, test_heated_nonterminal_recovery_rechecks_hardware_off)
{
    atomic_set(&main_suspended, true);
    sensor_suspend_owner = k_current_get();
    sensor_release_failed_restart();
    heater_stop_error = -EIO;
    zassert_equal(sensor_suspend_heated(), -EIO);
    zassert_true(atomic_get(&main_suspended));
    zassert_is_null(sensor_suspend_owner);
    zassert_equal(heater_stop_attempts, 1);
    heater_stop_error = 0;
    zassert_equal(sensor_suspend_heated(), 0);
    zassert_true(sensor_suspend_owner == k_current_get());
    zassert_equal(heater_stop_attempts, 2);
    main_imu_resume();
    zassert_false(atomic_get(&main_suspended));
}

ZTEST(sensor_lifecycle, test_shutdown_handoff_to_power_and_wom)
{
    start_first();
    zassert_equal(k_sem_take(&done1, K_SECONDS(2)), 0);
    zassert_equal(result1, 0);
    shutdown_on_suspend = true;
    start_second(suspend_entry, ESB_THREAD_PRIORITY);
    zassert_equal(k_sem_take(&suspended, K_SECONDS(1)), 0);
    zassert_equal(result2, 0);
    zassert_equal(shutdown_result, 0);
    zassert_equal(shutdown_calls, 1);
    zassert_true(atomic_get(&main_suspended));
    zassert_true(sensor_suspend_owner == power_thread_id);
    zassert_equal(sensor_request_scan(true, true), -EBUSY);
    zassert_equal(main_imu_suspend(), -EBUSY); /* late calibration is not power */
    main_imu_resume();
    zassert_true(atomic_get(&main_suspended));
    power_setup_wom = true;
    start_power();
    zassert_equal(k_sem_take(&power_acquired, K_SECONDS(1)), 0);
    zassert_equal(result3, 0x31);
    zassert_equal(wom_calls, 1);
    zassert_true(atomic_get(&main_suspended));
    k_sem_give(&allow_resume);
    zassert_equal(k_sem_take(&done2, K_SECONDS(1)), 0);
    zassert_true(atomic_get(&main_suspended));
    k_sem_give(&allow_power_resume);
    zassert_equal(k_sem_take(&done3, K_SECONDS(1)), 0);
    zassert_false(atomic_get(&main_suspended));
}

ZTEST(sensor_lifecycle, test_shutdown_failure_reports_error_and_preserves_suspension)
{
    start_first();
    zassert_equal(k_sem_take(&done1, K_SECONDS(2)), 0);
    zassert_equal(result1, 0);
    scan_suspend_error = -EIO;
    shutdown_on_suspend = true;
    start_second(suspend_entry, ESB_THREAD_PRIORITY);
    zassert_equal(k_sem_take(&suspended, K_SECONDS(1)), 0);
    zassert_equal(result2, 0);
    zassert_equal(shutdown_result, -EIO);
    zassert_true(atomic_get(&main_suspended));
    zassert_true(sensor_suspend_owner == power_thread_id);
    zassert_equal(sensor_request_scan(true, true), -EBUSY);
    zassert_equal(main_imu_suspend(), -EBUSY);
    start_power();
    zassert_equal(k_sem_take(&power_acquired, K_SECONDS(1)), 0);
    zassert_equal(result3, 0);
    k_sem_give(&allow_resume);
    zassert_equal(k_sem_take(&done2, K_SECONDS(1)), 0);
    zassert_true(atomic_get(&main_suspended));
    k_sem_give(&allow_power_resume);
    zassert_equal(k_sem_take(&done3, K_SECONDS(1)), 0);
}

ZTEST(sensor_lifecycle, test_scan_interface_failure_is_reported)
{
    scan_suspend_error = -EIO;
    start_first();
    zassert_equal(k_sem_take(&done1, K_SECONDS(2)), 0);
    zassert_equal(result1, -EIO);
    zassert_false(sensor_sensor_init);
    zassert_equal(sensor_thread_role, SENSOR_THREAD_NONE);
    zassert_false(wdt_scan_active);
}

ZTEST_SUITE(sensor_lifecycle, NULL, NULL, before, after, NULL);
