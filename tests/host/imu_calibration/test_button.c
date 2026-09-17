#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <setjmp.h>
#include <stdio.h>

#include "../../../src/connection/tracker_event_protocol.h"
#define USER_SHUTDOWN_ENABLED 1
#define WDT_CHANNEL_BUTTON 0
#define SYS_STATUS_BUTTON_PRESSED 0
#define SYS_LED_PATTERN_ON 0
#define SYS_LED_PATTERN_OFF 1
#define SYS_LED_PATTERN_OFF_FORCE 2
#define SYS_LED_PATTERN_LONG 3
#define SYS_LED_PATTERN_ONESHOT_PROGRESS 4
#define SYS_LED_PATTERN_ONESHOT_POWEROFF 5
#define SYS_LED_PRIORITY_HIGHEST 0
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)

static int64_t now, press_time, last_press_duration;
static int button_thread_id;
static bool button_status, hold_to_cancel;
static unsigned reads, pairs, aborts, requests, feeds;
static int admission_result;
static jmp_buf iteration_done;
static bool scripted, ota_busy, test_busy;
static struct { int64_t time, duration; } script[600];
static unsigned script_count, script_index, notices;
static uint8_t counts[4];
static void tracker_event_notice(uint8_t kind, uint8_t phase, uint8_t detail)
{
	assert(kind == TRACKER_EVENT_KIND_BUTTON && phase == BUTTON_CLICK_GROUP);
	assert(notices < 4 && detail);
	counts[notices++] = detail;
}
static void tracker_events_notify(void) {}

static int64_t k_uptime_get(void) { return now; }
static bool button_read(void) { return !scripted && (reads++ == 0 || hold_to_cancel); }
static void set_status(int id, bool value) { (void)id; button_status = value; }
static bool get_status(int id) { (void)id; return button_status; }
static void set_led(int pattern, int priority) { (void)pattern; (void)priority; }
static void watchdog_register_thread(int channel, int timeout) { (void)channel; (void)timeout; }
static void watchdog_feed(int channel) { (void)channel; feeds++; }
static void reboot_counter_write(int value) { (void)value; }
static bool esb_ota_is_active(void) { return ota_busy; }
static bool connection_get_ota_suppressed(void) { return false; }
static bool test_mode_get(void) { return test_busy; }
#if !CONFIG_USER_EXTRA_ACTIONS
static void esb_reset_pair(void) { pairs++; }
#endif
static int sys_request_system_off(void) { requests++; return admission_result; }
static int sys_request_system_reboot(void) { requests++; return admission_result; }
#if CONFIG_USER_EXTRA_ACTIONS
static void sys_reset_mode(int mode) { (void)mode; }
#endif
static void k_msleep(int ms)
{
	now += ms;
	if (ms == 20) {
		if (scripted && ++script_index < script_count) {
			now = script[script_index].time;
			last_press_duration = script[script_index].duration;
			return;
		}
		longjmp(iteration_done, 1); // one complete real button iteration
	}
}
static void k_thread_abort(int thread)
{
	(void)thread;
	aborts++;
	longjmp(iteration_done, 1);
}

#include "button.inc"

static void dispatch(int result, bool cancel)
{
	now = 10000;
	press_time = 1;
	last_press_duration = 0;
	button_status = false;
	hold_to_cancel = cancel;
	reads = pairs = aborts = requests = feeds = 0;
	admission_result = result;
	if (setjmp(iteration_done) == 0) {
		button_thread();
	}
}

static void add_iteration(int64_t time, int64_t duration)
{
	assert(script_count < 600);
	script[script_count].time = time;
	script[script_count++].duration = duration;
}

static void run_script(void)
{
	scripted = true;
	script_index = notices = requests = 0;
	press_time = 0;
	now = script[0].time;
	last_press_duration = script[0].duration;
	if (setjmp(iteration_done) == 0) {
		button_thread();
	}
	scripted = false;
}

static void click_groups(void)
{
	script_count = 0;
	add_iteration(100, 50); /* exactly 50ms is not accepted */
	add_iteration(1200, 0);
	run_script();
	assert(notices == 0);
	script_count = 0;
	add_iteration(100, 51);
	add_iteration(1100, 0); /* exactly 1000ms does not close the group */
	run_script();
	assert(notices == 0);
	script_count = 0;
	add_iteration(100, 51); add_iteration(200, 60); add_iteration(1201, 0);
	run_script();
	assert(notices == 1 && counts[0] == 2);
	for (unsigned blocked = 0; blocked < 3; blocked++) {
		ota_busy = blocked == 1; test_busy = blocked == 2;
		script_count = 0;
		add_iteration(100, 60); add_iteration(1101, 0);
		add_iteration(1200, 60); add_iteration(2201, 0);
		run_script();
		assert(notices == 2 && counts[0] == 1 && counts[1] == 1);
		assert(requests == (blocked ? 0U : 2U));
	}
	ota_busy = test_busy = false;
	for (unsigned count = 255; count <= 258; count += 3) {
		script_count = 0;
		for (unsigned i = 0; i < count; i++) add_iteration(100 + i * 20, 60);
		add_iteration(100 + (count - 1) * 20 + 1001, 0);
		run_script();
		assert(notices == 1 && counts[0] == 255);
	}
}

int main(void)
{
	dispatch(-EBUSY, false);
	assert(requests == 1 && pairs == 0 && aborts == 0 && feeds == 1);
	assert(press_time == 0 && !button_status);
	dispatch(0, false);
	assert(requests == 1 && pairs == 0 && aborts == 1);
	dispatch(0, true);
	assert(requests == 0 && aborts == 0 && feeds == 1);
	assert(pairs == (CONFIG_USER_EXTRA_ACTIONS ? 0U : 1U));
	assert(press_time == 0 && !button_status);
	assert(notices == 0); /* long hold/reset-counter paths are not click groups */
	click_groups();
	puts("Actual shutdown producer/button error dispatch regressions passed");
	return 0;
}
