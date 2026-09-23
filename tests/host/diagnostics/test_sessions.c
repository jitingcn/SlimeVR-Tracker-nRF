#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Compile production diagnostics, replacing only kernel/time and output leaves. */
#define SLIMENRF_SENSOR
#define CONFIG_SENSOR_USE_VQF 0
#define CONFIG_SENSOR_RANGE_STATS 0
float sensor_get_accel_odr(void);
#include "../../../src/sensor/diagnostics.c"

static int64_t now_ms;
static unsigned int lock_depth;
static unsigned int clock_calls_until_restart;
static void (*pending_console)(void);
static void (*print_hook)(void);
static void (*log_hook)(void);
static unsigned int stopped_counts[32];
static unsigned int stopped_count;
static unsigned int reports;
static float last_elapsed;
static float last_raw_accel;

static void restart_session(void)
{
	sensor_debug_start(2);
}

k_spinlock_key_t k_spin_lock(struct k_spinlock *lock)
{
	assert(!lock->held && !lock_depth);
	lock->held = 1;
	lock_depth++;
	return 0;
}

void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
	(void)key;
	assert(lock->held && lock_depth == 1);
	lock->held = 0;
	lock_depth--;
	if (pending_console) {
		void (*run)(void) = pending_console;
		pending_console = NULL;
		run();
	}
}

int64_t k_uptime_get(void)
{
	int64_t sampled = now_ms;
	if (clock_calls_until_restart && --clock_calls_until_restart == 0) {
		/* A higher-priority console becomes runnable while the sensor checks
		 * its deadline. It runs immediately unless interrupts are locked. */
		if (lock_depth) {
			pending_console = restart_session;
		} else {
			restart_session();
		}
	}
	return sampled;
}

float sensor_get_accel_odr(void)
{
	assert(lock_depth == 0);
	return 100.0f;
}

void test_log(const char *format, ...)
{
	assert(lock_depth == 0);
	if (log_hook) {
		void (*run)(void) = log_hook;
		log_hook = NULL;
		run();
	}
	if (strstr(format, "outputs generated")) {
		va_list args;
		va_start(args, format);
		assert(stopped_count < sizeof(stopped_counts) / sizeof(stopped_counts[0]));
		stopped_counts[stopped_count++] = va_arg(args, unsigned int);
		va_end(args);
	}
}

int printk(const char *format, ...)
{
	assert(lock_depth == 0);
	if (print_hook) {
		void (*run)(void) = print_hook;
		print_hook = NULL;
		run();
	}
	char line[512];
	va_list args;
	va_start(args, format);
	int len = vsnprintf(line, sizeof(line), format, args);
	va_end(args);
	float elapsed, accel;
	if (sscanf(line, "[%fs] RAW: A[%f", &elapsed, &accel) == 2) {
		reports++;
		last_elapsed = elapsed;
		last_raw_accel = accel;
	}
	return len;
}

static void collect(unsigned int samples, float value)
{
	const float a[3] = {value, value, value};
	for (unsigned int i = 0; i < samples; i++) {
		sensor_diagnostics_on_raw_accel(a);
	}
}

static void output(void)
{
	const float q[4] = {1, 0, 0, 0};
	const float a[3] = {0, 0, 0};
	sensor_diagnostics_output(q, a, a, 25, false);
}

static void emit(unsigned int samples, float value)
{
	sensor_diagnostics_reset_frame();
	collect(samples, value);
	output();
}

static void reset_observations(void)
{
	sensor_debug_stop();
	stopped_count = 0;
	reports = 0;
}

static void test_restart_at_expiry(void)
{
	reset_observations();
	now_ms = 1000;
	sensor_debug_start(1);
	emit(4, 1);
	now_ms = 2000;
	clock_calls_until_restart = 1;
	/* Either overlapping operation may linearize first. The replacement
	 * session must survive the old expiry check until its own deadline. */
	(void)sensor_debug_is_active();
	assert(sensor_debug_is_active());
	unsigned int reports_before_expiry = stopped_count;
	now_ms = 3999;
	assert(sensor_debug_is_active());
	now_ms = 4000;
	assert(!sensor_debug_is_active());
	assert(stopped_count == reports_before_expiry + 1);
	assert(stopped_counts[reports_before_expiry] == 0);
	assert(!sensor_debug_is_active());
	assert(stopped_count == reports_before_expiry + 1);
}

static void test_restart_discards_old_frame_and_sample_credit(void)
{
	reset_observations();
	now_ms = 5000;
	sensor_debug_start(2);
	emit(3, 7); // Credit belongs only to the old session.
	sensor_diagnostics_reset_frame();
	collect(4, 9);
	sensor_debug_start(2);
	collect(4, 11); // A crossing frame must not mix sessions.
	output();
	assert(reports == 0);
	emit(1, 2);
	assert(reports == 0);
	emit(3, 2);
	assert(reports == 1 && last_raw_accel == 2);
	sensor_debug_stop();
	assert(stopped_count == 1 && stopped_counts[0] == 1);
}

static void test_restart_during_output_admission(void)
{
	reset_observations();
	now_ms = 6000;
	sensor_debug_start(2);
	sensor_diagnostics_reset_frame();
	collect(4, 9);
	/* Console restart after the public active check but before reservation. */
	clock_calls_until_restart = 1;
	output();
	assert(reports == 0);
	emit(4, 2);
	assert(reports == 1 && last_raw_accel == 2);
	sensor_debug_stop();
	assert(stopped_count == 1 && stopped_counts[0] == 1);
}

static void test_reserved_output_keeps_its_session(void)
{
	reset_observations();
	now_ms = 7000;
	sensor_debug_start(2);
	sensor_diagnostics_reset_frame();
	collect(4, 3);
	now_ms = 7500;
	/* First clock is the active check; second is the output reservation. */
	clock_calls_until_restart = 2;
	output();
	assert(reports == 1 && last_elapsed == 0.5f && last_raw_accel == 3);
	sensor_debug_stop();
	assert(stopped_count == 1 && stopped_counts[0] == 0);

	/* Also allow stop/restart from the slow printk leaf, never under lock. */
	sensor_debug_start(2);
	print_hook = restart_session;
	emit(4, 5);
	sensor_debug_stop();
	assert(stopped_count == 2 && stopped_counts[1] == 0);
}

static void test_stop_report_snapshot(void)
{
	reset_observations();
	now_ms = 8000;
	sensor_debug_start(2);
	emit(4, 1);
	log_hook = restart_session;
	sensor_debug_stop();
	assert(sensor_debug_is_active());
	assert(stopped_count == 1 && stopped_counts[0] == 1);
	sensor_debug_stop();
	assert(stopped_count == 2 && stopped_counts[1] == 0);
}

static void test_deadline_rejects_unreserved_output(void)
{
	reset_observations();
	now_ms = 9000;
	sensor_debug_start(1);
	sensor_diagnostics_reset_frame();
	collect(4, 1);
	now_ms = 9999;
	assert(sensor_debug_is_active());
	now_ms = 10000;
	output();
	assert(reports == 0);
	assert(stopped_count == 1 && stopped_counts[0] == 0);

	/* The advertised maximum must survive until 60s, including at long uptime. */
	now_ms = (int64_t)UINT32_MAX + 1234;
	sensor_debug_start(60);
	now_ms += 59999;
	assert(sensor_debug_is_active());
	now_ms++;
	assert(!sensor_debug_is_active());
}

int main(void)
{
	test_restart_at_expiry();
	test_restart_discards_old_frame_and_sample_credit();
	test_restart_during_output_admission();
	test_reserved_output_keeps_its_session();
	test_stop_report_snapshot();
	test_deadline_rejects_unreserved_output();
	puts("diagnostics sessions: PASS");
	return 0;
}
