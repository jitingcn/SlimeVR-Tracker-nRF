#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>

#include "thread_priority.h"

/* The entire production wait and resume functions are extracted at build time.
 * Only logging, watchdog and calibration leaves are stubbed. All thread and
 * semaphore operations below and inside those functions use the real kernel. */
#define IMU_INT_EXISTS 1
#define DEBUG 0
#define WDT_CHANNEL_SENSOR 0
#define STATUS_INTERVAL_MS 10000
static void ignore_log(const char *format, ...) { ARG_UNUSED(format); }
#define LOG_INF(...) ignore_log(__VA_ARGS__)
#define LOG_WRN(...) ignore_log(__VA_ARGS__)
#define LOG_DBG(...) ignore_log(__VA_ARGS__)

static atomic_t main_suspended;
static bool main_ok = true;
static struct k_thread sensor_thread_id, calibration_thread;
K_THREAD_STACK_DEFINE(sensor_stack, 2048);
K_THREAD_STACK_DEFINE(calibration_stack, 2048);
K_SEM_DEFINE(at_idle, 0, 1);
K_SEM_DEFINE(release_idle, 0, 1);
K_SEM_DEFINE(sensor_int_sem, 0, 1);
K_SEM_DEFINE(progress, 0, 1);
K_SEM_DEFINE(done, 0, 1);
static bool consumer_ready, watchdog_running, prerequisites_ready;
static int progress_result;

/* State used by the unmodified wait function, including its diagnostic branch. */
static float processing_work_time_ema_ms, sensor_window_fused_angle_rad, mag_feed_hz;
static int64_t max_loop_time, last_status_time, sensor_data_time;
static uint32_t sensor_update_time_ms = 5, sensor_int_timeouts;
static uint32_t sensor_window_iters, sensor_window_publishes, sensor_window_packets;
static uint32_t sensor_window_ints, mag_vqf_updates_since_status;
static uint64_t sensor_window_proc_us, sensor_window_wait_us, sensor_window_acq_us;
static uint64_t sensor_window_vqf_us, sensor_window_acq_max_us, sensor_window_proc_max_us;
static uint64_t sensor_window_resume_max_us, sensor_window_fifo_max_us;
static bool mag_available, mag_enabled, sensor_int_during_loop;
static float mag_actual_time;

static void watchdog_feed(int channel) { ARG_UNUSED(channel); }
static void watchdog_resume(int channel) { ARG_UNUSED(channel); watchdog_running = true; }
static void sensor_calibration_set_consumer_ready(bool ready) { consumer_ready = ready; }
static bool sensor_debug_is_active(void) { return false; }
static void sensor_life_mark_busy(void) {}
static void sensor_life_mark_idle(void)
{
	/* Arrange the audited external-suspend location: after IDLE publication,
	 * before the wait tail tests pending suspension. The FIFO is made ready
	 * while suspended, so no blocking call can hide the resume ordering bug. */
	k_sem_give(&at_idle);
	k_sem_take(&release_idle, K_FOREVER);
}

#include "sensor_lifecycle.inc"

static void sensor_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	sensor_loop_wait(k_uptime_get());
	prerequisites_ready = consumer_ready && watchdog_running;
	k_sem_give(&progress);
}

static void calibration_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	k_sem_take(&at_idle, K_FOREVER);
	atomic_set(&main_suspended, true);
	k_thread_suspend(&sensor_thread_id);
	k_sem_give(&release_idle);
	k_sem_give(&sensor_int_sem);
	main_imu_resume();
	/* Sensor priority 7 must preempt calibration priority 8 and finish before
	 * resume returns. In the old order it instead suspends itself a second time. */
	progress_result = k_sem_take(&progress, K_NO_WAIT);
	k_sem_give(&done);
}

ZTEST(sensor_recovery, test_higher_priority_resume_does_not_self_suspend)
{
	zassert_true(SENSOR_LOOP_THREAD_PRIORITY < CALIBRATION_THREAD_PRIORITY);
	k_thread_create(&sensor_thread_id, sensor_stack, K_THREAD_STACK_SIZEOF(sensor_stack),
			sensor_entry, NULL, NULL, NULL, K_PRIO_PREEMPT(SENSOR_LOOP_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_create(&calibration_thread, calibration_stack, K_THREAD_STACK_SIZEOF(calibration_stack),
			calibration_entry, NULL, NULL, NULL, K_PRIO_PREEMPT(CALIBRATION_THREAD_PRIORITY), 0, K_NO_WAIT);
	int completed = k_sem_take(&done, K_SECONDS(1));
	/* Also clean up a failing old-source run: it leaves the sensor suspended. */
	k_thread_abort(&sensor_thread_id);
	k_thread_abort(&calibration_thread);
	zassert_equal(completed, 0, "Calibration recovery did not return");
	zassert_equal(progress_result, 0, "Resumed sensor self-suspended before making progress");
	zassert_true(prerequisites_ready, "Watchdog/readiness must be restored before sensor execution");
}

ZTEST_SUITE(sensor_recovery, NULL, NULL, NULL, NULL, NULL);
