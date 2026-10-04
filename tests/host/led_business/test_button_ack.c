#include <errno.h>
#include <setjmp.h>
#include "../led_sync/test_runtime.h"
#include "../../../src/connection/tracker_event_protocol.h"

#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define SYS_REBOOT_COLD 0
static void sys_reboot(int mode) { (void)mode; assert(!"unexpected watchdog reboot"); }
#define WDT_CHANNEL_BUTTON 0
#define SYS_STATUS_BUTTON_PRESSED (1 << 0)
#define SYS_STATUS_CONNECTION_ERROR (1 << 1)
#define HEATED_BUTTON_EXISTS 0
struct device { int unused; };
struct gpio_callback { int unused; };
static int64_t press_time, last_press_duration, last_press_started_at;
static uint32_t press_generation, last_press_generation;
static bool button_held_from_init, physical_pressed, ota_busy, test_busy;
static int status_state;
static int atomic_get(const int *state) { return *state; }
static unsigned reboot_requests, shutdown_requests, pair_requests, reset_requests, notices;
static uint32_t reboot_at_ms, notice_at_ms;
static uint8_t group_count;
static unsigned reset_mode;
static enum led_semantic reset_result;
static int button_thread_id;
static jmp_buf script_finished;
static bool button_read(void) { return physical_pressed; }
static int button0;
static int gpio_pin_get_dt(const int *pin) { (void)pin; return physical_pressed; }
static void set_status(int status, bool value)
{
	assert(status == SYS_STATUS_BUTTON_PRESSED);
	if (value) status_state |= status;
	else status_state &= ~status;
}
static bool get_status(int status) { return (status_state & status) != 0; }
static int watchdog_register_thread(int channel, int timeout) { (void)channel; (void)timeout; return 0; }
static void watchdog_feed(int channel) { (void)channel; }
static inline unsigned int irq_lock(void) { return 0; }
static inline void irq_unlock(unsigned int key) { (void)key; }
static bool esb_ota_is_active(void) { return ota_busy; }
static bool connection_get_ota_suppressed(void) { return false; }
static bool test_mode_get(void) { return test_busy; }
static void publish_result(enum led_semantic semantic)
{
	if (semantic != LED_NONE) {
		struct led_token token = led_begin(LED_OWNER_SYSTEM, led_request_id());
		assert(led_result(token, led_event_id(), semantic) == LED_ADMITTED);
	}
}
static int sys_button_reboot(uint32_t generation)
{
	(void)generation;
	++reboot_requests;
	reboot_at_ms = host_now_ms;
	return 0;
}
static int sys_button_shutdown(int64_t original_press_time, uint32_t generation)
{ (void)original_press_time; (void)generation; ++shutdown_requests; return 1; }
#if CONFIG_USER_EXTRA_ACTIONS
static void sys_reset_mode(unsigned mode)
{
	reset_mode = mode;
	++reset_requests;
	publish_result(reset_result);
}
#else
static void esb_reset_pair(void) { ++pair_requests; }
#endif
static void tracker_event_notice(uint8_t kind, uint8_t phase, uint8_t detail)
{
	assert(kind == TRACKER_EVENT_KIND_BUTTON && phase == BUTTON_CLICK_GROUP);
	++notices;
	group_count = detail;
	notice_at_ms = host_now_ms;
}
static void tracker_events_notify(void) {}
static void k_msleep(int milliseconds);
static void k_thread_abort(int thread) { (void)thread; assert(false); }
#include "button_production.inc"

enum action_kind {
	BUTTON_EDGE, CLEAR_FAULT, SET_BLOCKING, SET_SAFETY, SET_LOW, CLEAR_LOW,
	QUIESCE, CHECK_HELD_SLEEP_VETO, SUCCESS_RESULT
};
struct action { uint32_t at_ms; enum action_kind kind; bool pressed; };
static struct action actions[540];
static unsigned action_count, action_index;
static uint32_t end_ms;
struct pulse { uint32_t start_ms, end_ms, generation, identity; };
static struct pulse pulses[260];
static unsigned pulse_count;
static bool input_lit;
static bool count_selected;
static uint32_t last_train_end_ms;
enum replacement_phase { REPLACE_NONE, REPLACE_OFF, REPLACE_ON };
static enum replacement_phase replacement_phase;
static uint32_t replacement_press_ms;
static void add_press(uint32_t press_ms, uint32_t release_ms);
static void observe_output(void)
{
	bool lit = engine.winner.button_count != 0 && host_value != 0;
	if (count_selected && !engine.winner.button_count) last_train_end_ms = host_now_ms;
	count_selected = engine.winner.button_count != 0;
	if (press_time && host_now_ms - (uint32_t)press_time > 70) {
		assert(!lit); /* Validated input cannot coexist with the old train. */
	}
	if (lit && !input_lit) {
		assert(pulse_count < sizeof(pulses) / sizeof(pulses[0]));
		pulses[pulse_count++] = (struct pulse){
			.start_ms = host_now_ms,
			.generation = engine.active_event.event_id,
			.identity = engine.winner.identity
		};
		if (pulse_count == 1 && replacement_phase != REPLACE_NONE) {
			/* Anchor input to the observed train, not old debounce latency. */
			replacement_press_ms = host_now_ms
				+ (replacement_phase == REPLACE_ON ? LED_BUTTON_CYCLE_MS + 40U : 80U + 200U);
			add_press(replacement_press_ms, replacement_press_ms + 80U);
		}
	}
	if (!lit && input_lit) pulses[pulse_count - 1].end_ms = host_now_ms;
	input_lit = lit;
}
static void apply_actions(void)
{
	while (action_index < action_count && actions[action_index].at_ms == host_now_ms) {
		const struct action *action = &actions[action_index++];
		switch (action->kind) {
		case BUTTON_EDGE:
			physical_pressed = action->pressed;
			button_interrupt_handler(NULL, NULL, 1);
			break;
		case CLEAR_FAULT: led_fault_publish(LED_OWNER_SENSOR, LED_FAULT_NONE, 0); break;
		case SET_BLOCKING: led_fault_publish(LED_OWNER_SENSOR, LED_FAULT_SENSOR, 0); break;
		case SET_SAFETY: led_fault_publish(LED_OWNER_SENSOR, LED_FAULT_SAFETY, 0); break;
		case SET_LOW: led_power_publish(LED_POWER_BATTERY, true); break;
		case CLEAR_LOW: led_power_publish(LED_POWER_BATTERY, false); break;
		case QUIESCE: led_quiesce(); break;
		case CHECK_HELD_SLEEP_VETO:
			assert(physical_pressed && !status_ready());
			break;
		case SUCCESS_RESULT: publish_result(LED_SUCCESS); break;
		}
	}
}
static void k_msleep(int milliseconds)
{
	uint32_t until_ms = host_now_ms + (uint32_t)milliseconds;
	while (host_now_ms < until_ms) {
		++host_now_ms;
		apply_actions();
		host_step(host_now_ms); /* Actual LED worker, policy and timeline. */
		observe_output();
		if (host_now_ms >= end_ms) longjmp(script_finished, 1);
	}
}
static void reset_case(void)
{
	host_reset(LED_CAP_RGB_PWM);
	press_time = last_press_duration = last_press_started_at = 0;
	press_generation = last_press_generation = 0;
	button_held_from_init = physical_pressed = ota_busy = test_busy = false;
	status_state = 0;
	reboot_requests = shutdown_requests = pair_requests = reset_requests = notices = 0;
	reboot_at_ms = notice_at_ms = group_count = reset_mode = 0;
	reset_result = LED_NONE;
	action_count = action_index = pulse_count = 0;
	input_lit = false;
	count_selected = false;
	last_train_end_ms = 0;
	replacement_phase = REPLACE_NONE; replacement_press_ms = 0;
	memset(pulses, 0, sizeof(pulses));
}
static void add_action(uint32_t time, enum action_kind kind, bool pressed)
{
	assert(action_count < sizeof(actions) / sizeof(actions[0]));
	assert(!action_count || actions[action_count - 1].at_ms < time);
	actions[action_count++] = (struct action){time, kind, pressed};
}
static void add_press(uint32_t press_ms, uint32_t release_ms)
{
	add_action(press_ms, BUTTON_EDGE, true);
	add_action(release_ms, BUTTON_EDGE, false);
}
static void run_until(uint32_t finish_ms)
{
	end_ms = finish_ms;
	host_step(host_now_ms);
	if (setjmp(script_finished) == 0) button_thread();
}
static void complete_train(unsigned first, unsigned count)
{
	assert(first + count <= pulse_count);
	for (unsigned i = first; i < first + count; ++i) {
		assert(pulses[i].end_ms - pulses[i].start_ms == 80);
		if (i > first) {
			assert(pulses[i].start_ms - pulses[i - 1].end_ms == 420);
			assert(pulses[i].generation == pulses[first].generation);
			assert(pulses[i].identity == pulses[first].identity);
		}
	}
}
static void one_and_several_presses(void)
{
	for (unsigned count = 2; count <= 4; count += 2) {
		reset_case();
		led_power_publish(LED_POWER_EXTERNAL_UNKNOWN, false);
		for (unsigned i = 0; i < count; ++i) add_press(100 + i * 350, 180 + i * 350);
		run_until(2000 + count * 850);
		assert(pulse_count == count);
		complete_train(0, count);
		assert(pulses[0].start_ms > 180 + (count - 1) * 350 + 1000);
		assert(notices == 1 && group_count == count && shutdown_requests == 0 && pair_requests == 0);
		assert(reboot_requests == 0);
		assert(reset_requests == (CONFIG_USER_EXTRA_ACTIONS ? 1U : 0U));
		if (CONFIG_USER_EXTRA_ACTIONS) assert(reset_mode == count - 1);
		assert(host_value != 0 && engine.winner.semantic == LED_EXTERNAL_POWER_UNKNOWN);
		/* The final off tail belongs to the train even above a solid lamp. */
		uint32_t tail = pulses[count - 1].end_ms;
		assert(last_train_end_ms == tail + 420);
	}
}
static void held_press_spans_previous_group_expiry(void)
{
	reset_case();
	add_press(100, 180);
	add_press(450, 530);
	add_action(1400, BUTTON_EDGE, true);
	add_action(1700, CHECK_HELD_SLEEP_VETO, false);
	add_action(1900, BUTTON_EDGE, false);
	run_until(3500);
	assert(pulse_count == 0); /* Terminal single click uses only its exit cue. */
	assert(notices == 2 && group_count == 1 && reboot_requests == 1 && shutdown_requests == 0);
	assert(reset_requests == (CONFIG_USER_EXTRA_ACTIONS ? 2U : 0U));
}
static void debounce_fast_inputs_and_large_group(void)
{
	reset_case();
	add_press(100, 150);
	add_press(200, 249);
	run_until(1600);
	assert(pulse_count == 0 && notices == 0 && reboot_requests == 0);
	reset_case();
	for (unsigned i = 0; i < 5; ++i) add_press(101 + i * 140, 152 + i * 140);
	run_until(5000);
	assert(pulse_count == 5 && notices == 1 && group_count == 5);
	complete_train(0, 5);
	assert(pulses[0].start_ms > 1712 && reboot_requests == 0 && shutdown_requests == 0);
	if (CONFIG_USER_EXTRA_ACTIONS) assert(reset_requests == 1 && reset_mode == 4);
	/* Wire notice saturation is not a display-count limit. */
	reset_case();
	for (unsigned i = 0; i < 258; ++i) add_press(101 + i * 80, 152 + i * 80);
	run_until(152 + 257 * 80 + 1000 + 258 * LED_BUTTON_CYCLE_MS + 500);
	assert(pulse_count == 258 && notices == 1 && group_count == 255);
	complete_train(0, 258);
}
static void new_group_cancels_old_train(void)
{
	for (unsigned during_on = 0; during_on < 2; ++during_on) {
		reset_case();
		add_press(100, 180);
		add_press(450, 530);
		add_press(800, 880);
		replacement_phase = during_on ? REPLACE_ON : REPLACE_OFF;
		run_until(4300);
		unsigned new_index = during_on ? 2 : 1;
		assert(pulse_count == new_index);
		complete_train(0, 1);
		if (during_on) {
			assert(replacement_press_ms == pulses[1].start_ms + 40U);
			assert(pulses[1].end_ms == replacement_press_ms); /* Immediate physical-edge cancellation. */
		}
		assert(pulses[0].start_ms > 1880);
		assert(notices == 2 && group_count == 1 && reboot_requests == 1 && shutdown_requests == 0);
	}
}
static void long_hold_is_not_a_click_group(void)
{
	reset_case();
	add_press(100, 1300);
	run_until(2700);
	assert(pulse_count == 0 && notices == 0 && reboot_requests == 0 && shutdown_requests == 1);
	assert(pair_requests == (CONFIG_USER_EXTRA_ACTIONS ? 0U : 1U));
}
static void real_results_keep_their_contract(void)
{
	reset_case();
	add_press(100, 180);
	add_press(450, 530);
	add_press(800, 880);
	add_action(2000, SUCCESS_RESULT, false);
	run_until(3400);
	assert(pulse_count == 1); /* A truthful result interrupts and does not resume count. */
	complete_train(0, 1);
	for (unsigned busy = 0; busy < 2; ++busy) {
		reset_case();
		ota_busy = busy == 0;
		test_busy = busy == 1;
		add_press(100, 180);
		run_until(2600);
		assert(notices == 1 && reboot_requests == 0 && pulse_count == 0);
	}
#if CONFIG_USER_EXTRA_ACTIONS
	reset_case();
	reset_result = LED_SUCCESS;
	add_press(100, 180);
	add_press(450, 530);
	run_until(2900);
	assert(reset_requests == 1 && reset_mode == 1 && pulse_count == 0);
#endif
}
static void masking_and_preemption(void)
{
	for (unsigned blocker = 0; blocker < 4; ++blocker) {
		reset_case();
		if (blocker == 0) led_fault_publish(LED_OWNER_SENSOR, LED_FAULT_SENSOR, 0);
		if (blocker == 1) led_fault_publish(LED_OWNER_SENSOR, LED_FAULT_SAFETY, 0);
		if (blocker == 3) led_quiesce();
		add_press(100, 180);
		add_press(450, 530);
		if (blocker == 2) add_action(1000, SET_LOW, false);
		if (blocker != 3) add_action(2500, blocker == 2 ? CLEAR_LOW : CLEAR_FAULT, false);
		run_until(3000);
		assert(notices == 1 && group_count == 2 && pulse_count == 0);
		if (blocker == 3) {
			assert(engine.winner.priority == LED_PRIORITY_SHUTDOWN && host_value == 0);
		}
	}
	for (unsigned blocker = 0; blocker < 3; ++blocker) {
		reset_case();
		add_press(100, 180);
		add_press(450, 530);
		add_press(800, 880);
		add_action(2000, blocker == 0 ? SET_SAFETY : blocker == 1 ? SET_LOW : QUIESCE, false);
		if (blocker != 2) add_action(2500, blocker == 0 ? CLEAR_FAULT : CLEAR_LOW, false);
		run_until(3400);
		assert(pulse_count == 1);
		complete_train(0, 1);
		if (blocker == 2) {
			assert(engine.winner.priority == LED_PRIORITY_SHUTDOWN && host_value == 0);
		}
	}
}
static void stale_generations_and_arithmetic_bound(void)
{
	reset_case();
	uint32_t old = led_button_input();
	uint32_t current = led_button_input();
	assert(led_button_group(old, 3) == LED_STALE);
	assert(led_button_group(current, 0) == LED_INVALID);
	assert(led_button_group(current, LED_BUTTON_GROUP_MAX + 1U) == LED_INVALID);
	assert(led_button_group(current, LED_BUTTON_GROUP_MAX) == LED_ADMITTED);
	assert(led_button_group(current, LED_BUTTON_GROUP_MAX) == LED_DUPLICATE);
	uint32_t origin = 120;
	host_step(0);
	host_step(origin);
	assert(engine.winner.button_count == LED_BUTTON_GROUP_MAX && host_value != 0);
	uint32_t duration = LED_BUTTON_GROUP_MAX * LED_BUTTON_CYCLE_MS;
	host_step(origin + duration - 421);
	assert(host_value != 0);
	host_step(origin + duration - 420);
	assert(host_value == 0 && engine.winner.button_count == LED_BUTTON_GROUP_MAX);
	host_step(origin + duration);
	assert(engine.winner.button_count == 0);
	reset_case();
	host_step(UINT32_MAX - 200U);
	current = led_button_input();
	assert(led_button_group(current, 2) == LED_ADMITTED);
	host_step(UINT32_MAX - 200U);
	host_step(UINT32_MAX - 80U);
	assert(host_value != 0);
	host_step(919);
	assert(engine.winner.button_count == 0);
}
int main(void)
{
	one_and_several_presses();
	held_press_spans_previous_group_expiry();
	debounce_fast_inputs_and_large_group();
	new_group_cancels_old_train();
	long_hold_is_not_a_click_group();
	real_results_keep_their_contract();
	masking_and_preemption();
	stale_generations_and_arithmetic_bound();
	puts("Actual button ISR/thread + LED worker: nonterminal quiet-window N-count trains, cancellation, debounce, unchanged actions and fault/LOW/P0 precedence passed");
	return 0;
}
