#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <setjmp.h>
#include <stdio.h>

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

static int64_t k_uptime_get(void) { return now; }
static bool button_read(void) { return reads++ == 0 || hold_to_cancel; }
static void set_status(int id, bool value) { (void)id; button_status = value; }
static bool get_status(int id) { (void)id; return button_status; }
static void set_led(int pattern, int priority) { (void)pattern; (void)priority; }
static void watchdog_register_thread(int channel, int timeout) { (void)channel; (void)timeout; }
static void watchdog_feed(int channel) { (void)channel; feeds++; }
static void reboot_counter_write(int value) { (void)value; }
static bool esb_ota_is_active(void) { return false; }
static bool connection_get_ota_suppressed(void) { return false; }
static bool test_mode_get(void) { return false; }
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
	puts("Actual shutdown producer/button error dispatch regressions passed");
	return 0;
}
