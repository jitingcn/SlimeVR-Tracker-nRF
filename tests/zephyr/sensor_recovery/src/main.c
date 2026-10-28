#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>

#include "thread_priority.h"

/* Extract the real gate and resume functions; only hardware/readiness leaves
 * are modeled. Mutexes, semaphores and scheduling use the Zephyr kernel. */
#define WDT_CHANNEL_SENSOR 0
#define TRACKER_REST_INITIALIZING 1
#define LOG_INF(...) do {} while (0)
#define LOG_WRN(...) do {} while (0)
#define LOG_ERR(...) do {} while (0)

static atomic_t main_suspended, sensor_loop_stop;
static bool main_ok = true;
static struct k_thread sensor_thread_id, calibration_thread;
static struct k_mutex sensor_lifecycle_lock, sensor_activity_lock;
static struct k_sem sensor_loop_gate;
static k_tid_t sensor_suspend_owner;
enum sensor_thread_role { SENSOR_THREAD_NONE, SENSOR_THREAD_SCAN, SENSOR_THREAD_LOOP };
static enum sensor_thread_role sensor_thread_role = SENSOR_THREAD_LOOP;
K_THREAD_STACK_DEFINE(sensor_stack, 2048);
K_THREAD_STACK_DEFINE(calibration_stack, 2048);
K_SEM_DEFINE(at_gate, 0, 1);
K_SEM_DEFINE(progress, 0, 1);
K_SEM_DEFINE(done, 0, 1);
static bool consumer_ready, watchdog_running, prerequisites_ready;
static int progress_result;

static void watchdog_resume(int channel) { ARG_UNUSED(channel); watchdog_running = true; }
static void sensor_calibration_set_consumer_ready(bool ready) { consumer_ready = ready; }
static void tracker_events_sensor_invalidate(int reason) { ARG_UNUSED(reason); }
static void tracker_events_notify(void) {}

#include "sensor_lifecycle.inc"

static void sensor_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	k_sem_give(&at_gate);
	if (sensor_loop_enter()) {
		prerequisites_ready = consumer_ready && watchdog_running;
		sensor_loop_leave();
		k_sem_give(&progress);
	}
}

static void calibration_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	k_sem_take(&at_gate, K_FOREVER);
	/* Let the higher-priority loop reach the real suspended gate first. */
	k_msleep(1);
	main_imu_resume();
	/* The higher-priority loop must make progress with readiness restored. */
	progress_result = k_sem_take(&progress, K_NO_WAIT);
	k_sem_give(&done);
}

ZTEST(sensor_recovery, test_higher_priority_resume_observes_readiness)
{
	zassert_true(SENSOR_LOOP_THREAD_PRIORITY < CALIBRATION_THREAD_PRIORITY);
	k_mutex_init(&sensor_lifecycle_lock);
	k_mutex_init(&sensor_activity_lock);
	k_sem_init(&sensor_loop_gate, 0, 1);
	atomic_set(&main_suspended, true);
	sensor_suspend_owner = &calibration_thread;
	k_thread_create(&sensor_thread_id, sensor_stack, K_THREAD_STACK_SIZEOF(sensor_stack),
			sensor_entry, NULL, NULL, NULL, K_PRIO_PREEMPT(SENSOR_LOOP_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_create(&calibration_thread, calibration_stack, K_THREAD_STACK_SIZEOF(calibration_stack),
			calibration_entry, NULL, NULL, NULL, K_PRIO_PREEMPT(CALIBRATION_THREAD_PRIORITY), 0, K_NO_WAIT);
	int completed = k_sem_take(&done, K_SECONDS(1));
	k_thread_abort(&sensor_thread_id);
	k_thread_abort(&calibration_thread);
	zassert_equal(completed, 0, "Calibration recovery did not return");
	zassert_equal(progress_result, 0, "Resumed sensor did not make progress");
	zassert_true(prerequisites_ready, "Watchdog/readiness must be restored before sensor execution");
}

ZTEST_SUITE(sensor_recovery, NULL, NULL, NULL, NULL, NULL);
