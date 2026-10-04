#include <errno.h>
#include <setjmp.h>
#include "../led_sync/test_runtime.h"
#include "../../../src/system/power_request.h"
#include "../../../src/connection/tracker_event_protocol.h"

#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define WDT_CHANNEL_BUTTON 0
#define SYS_STATUS_BUTTON_PRESSED 1
#define HEATED_BUTTON_EXISTS 0
#define CONFIG_SENSOR_TCAL_HEATED 0
#define BUTTON_EXISTS 1
#define CONFIG_SENSOR_USE_TCAL 0
#define CONFIG_DISABLE_SENSOR_GPIOS_ON_SHUTDOWN 0
#define IMU_INT_EXISTS 0
#define ADAFRUIT_BOOTLOADER 0
#define SYS_REGULATOR_LDO 0
#define SYS_REBOOT_COLD 0
struct device { int unused; };
struct gpio_callback { int unused; };
struct k_mutex { bool locked; };
static unsigned manual_request_attempts;
static void fixture_mutex_lock(struct k_mutex *mutex, int64_t timeout, const char *caller)
{
	(void)timeout; assert(!mutex->locked); mutex->locked = true;
	manual_request_attempts += strcmp(caller, "sys_power_state_request") == 0;
}
#define k_mutex_lock(mutex, timeout) fixture_mutex_lock((mutex), (timeout), __func__)
static void k_mutex_unlock(struct k_mutex *mutex)
{ assert(mutex->locked); mutex->locked = false; }
static struct k_mutex power_plan_lock;
static struct power_request_mailbox power_requests;
static struct k_sem power_wake_sem;
static int64_t press_time, last_press_duration, last_press_started_at;
static uint32_t press_generation, last_press_generation;
static bool button_held_from_init, physical_pressed, ota_busy;
static bool button_status;
static int button_thread_id;
static unsigned pair_requests, reset_requests, click_notices, physical_offs, physical_reboots;
static uint32_t handoff_release_ms;
static uint32_t abort_at_ms, hold_first_ms, hold_last_ms, exit_origin_ms, power_notice_ms;
static bool saw_exit, saw_cancel, saw_failed;
static uint16_t last_exit_value;
static jmp_buf script_finished;

static bool button_read(void) { return physical_pressed; }
static int button0;
static int gpio_pin_get_dt(const int *pin) { (void)pin; return physical_pressed; }
static void set_status(int status, bool value)
{
	assert(status == SYS_STATUS_BUTTON_PRESSED);
	if (!value) assert(!press_generation && last_press_duration <= 50);
	button_status = value;
}
static bool get_status(int status) { (void)status; return button_status; }
static int watchdog_register_thread(int channel, int timeout) { (void)channel; (void)timeout; return 0; }
static void watchdog_feed(int channel) { (void)channel; }
static unsigned int irq_depth;
static void (*irq_deferred)(void);
static inline unsigned int irq_lock(void) { return irq_depth++; }
static inline void irq_unlock(unsigned int key)
{
	assert(irq_depth == key + 1); irq_depth = key;
	if (!irq_depth && irq_deferred) {
		void (*pending)(void) = irq_deferred; irq_deferred = NULL; pending();
	}
}
static bool esb_ota_is_active(void) { return ota_busy; }
static bool connection_get_ota_suppressed(void) { return false; }
static bool test_mode_get(void) { return false; }
static void reboot_counter_write(int value) { (void)value; }
static void sys_cancel_WOM_locked(void) { (void)power_request_cancel_wom(&power_requests); }
static void sys_reset_mode(unsigned mode) { (void)mode; ++reset_requests; }
#if !CONFIG_USER_EXTRA_ACTIONS
static void esb_reset_pair(void) { ++pair_requests; }
#endif
static void tracker_event_notice(uint8_t kind, uint8_t phase, uint8_t detail)
{
	(void)detail;
	if (kind == TRACKER_EVENT_KIND_BUTTON) {
		assert(phase == BUTTON_CLICK_GROUP); ++click_notices;
	} else {
		assert(kind == TRACKER_EVENT_KIND_POWER); power_notice_ms = host_now_ms;
	}
}
static void tracker_events_notify(void) {}
static bool configure_system_off(void) { led_shutdown(); return true; }
static int sys_flush_warm(void) { return 0; }
static void sensor_calibration_online_mag_cold_start(void) {}
static void sensor_retained_write(void) {}
static void set_regulator(int regulator) { (void)regulator; }
static void sys_disconnect_interface_pins(void) {}
static int power_battery_current_pptt(void) { return 5000; }
static bool power_battery_device_plugged(void) { return false; }
static void sys_update_battery_tracker(int pptt, bool plugged) { (void)pptt; (void)plugged; }
static void wait_for_logging(void) {}
static void sys_poweroff(void) { ++physical_offs; }
static void sys_reboot(int mode) { (void)mode; ++physical_reboots; }
static void k_msleep(int milliseconds);
static void k_thread_abort(int thread)
{ (void)thread; abort_at_ms = host_now_ms; longjmp(script_finished, 1); }
static int sys_user_shutdown(void);
static int sys_button_reboot(uint32_t generation);
#include "manual_production.inc"

enum action_kind { EDGE, OTA_RESERVE, OTA_ACTIVE, PROTECTIVE_OFF, SAFETY_ON, SAFETY_OFF, LOW_ON, STALE_SUCCESS };
struct action { uint32_t at_ms; enum action_kind kind; bool pressed; };
static struct action actions[8];
static unsigned action_count, action_index;
static uint32_t end_ms;
static uint16_t expected_exit_value(uint32_t age, bool gpio)
{
	if (age < 250 || age >= 1250) return 0;
	return gpio ? 10000 : (1250 - age) * 10;
}
static void observe(void)
{
	assert(!engine.winner.button_count); /* Terminal actions never stack N=1. */
	assert(engine.winner.semantic != LED_ACCEPTED && engine.winner.semantic != LED_SUCCESS);
	if (engine.winner.semantic == LED_BUTTON_HOLD) {
		if (!physical_pressed) {
			assert(last_press_duration >= LED_BUTTON_HOLD_MS || last_press_started_at == engine.winner.origin_ms);
		}
		if (led_output_enabled()) {
			uint32_t age = host_now_ms - engine.winner.origin_ms;
			assert(host_role == LED_ROLE_INTERACTION && host_level == LED_LEVEL_NOTICE);
			uint16_t expected = age < 1000 ? 10000 : age < 2500
				? expected_exit_value(age - 1000, host_hardware.gpio)
				: (age - 2500) % 1000 < 500 ? 10000 : 0;
			assert(host_value == expected);
		}
		if (!hold_first_ms) hold_first_ms = host_now_ms;
		hold_last_ms = host_now_ms;
	}
	if (engine.winner.semantic == LED_MANUAL_EXIT) {
		if (!saw_exit) {
			saw_exit = true; exit_origin_ms = engine.winner.origin_ms; last_exit_value = 10000;
		}
		assert(engine.winner.origin_ms == exit_origin_ms);
		assert(host_role == LED_ROLE_INTERACTION && host_level == LED_LEVEL_NOTICE);
		assert(host_value == expected_exit_value(host_now_ms - exit_origin_ms, host_hardware.gpio));
		last_exit_value = host_value;
	}
	saw_cancel |= engine.winner.semantic == LED_CANCELLED;
	saw_failed |= engine.winner.semantic == LED_FAILED || engine.winner.semantic == LED_REJECTED;
	if (engine.quiesced) {
		assert(engine.winner.priority == LED_PRIORITY_SHUTDOWN && host_value == 0);
	}
	if (handoff_release_ms && host_now_ms >= handoff_release_ms && !engine.quiesced) {
		assert(engine.winner.semantic == LED_BUTTON_HOLD || engine.winner.semantic == LED_MANUAL_EXIT);
		assert(host_role == LED_ROLE_INTERACTION && host_level == LED_LEVEL_NOTICE);
	}
}
static void apply_actions(void)
{
	while (action_index < action_count && actions[action_index].at_ms == host_now_ms) {
		const struct action *action = &actions[action_index++];
		switch (action->kind) {
		case EDGE:
			physical_pressed = action->pressed; button_interrupt_handler(NULL, NULL, 1); break;
		case OTA_RESERVE: assert(power_request_ota_reserve(&power_requests) == 0); break;
		case OTA_ACTIVE: ota_busy = true; break;
		case PROTECTIVE_OFF: assert(sys_system_off()); break;
		case SAFETY_ON: led_fault_publish(LED_OWNER_SENSOR, LED_FAULT_SAFETY, 0); break;
		case SAFETY_OFF: led_fault_publish(LED_OWNER_SENSOR, LED_FAULT_NONE, 0); break;
		case LOW_ON: led_power_publish(LED_POWER_BATTERY, true); break;
		case STALE_SUCCESS:
			assert(led_request_event(LED_OWNER_ACC, led_request_id(), led_event_id(), LED_SUCCESS) == LED_ADMITTED);
			break;
		}
	}
}
static void k_msleep(int milliseconds)
{
	uint32_t until = host_now_ms + (uint32_t)milliseconds;
	while (host_now_ms < until) {
		++host_now_ms; apply_actions(); host_step(host_now_ms); observe();
		if (host_now_ms >= end_ms) longjmp(script_finished, 1);
	}
}
static void reset_case(enum led_capability capability)
{
	host_reset(capability); memset(&power_requests, 0, sizeof(power_requests));
	power_plan_lock.locked = false; power_wake_sem.count = 0;
	manual_request_attempts = 0;
	press_time = last_press_duration = last_press_started_at = 0;
	press_generation = last_press_generation = 0;
	irq_depth = 0; irq_deferred = NULL;
	button_held_from_init = physical_pressed = ota_busy = button_status = false;
	pair_requests = reset_requests = click_notices = physical_offs = physical_reboots = 0;
	abort_at_ms = hold_first_ms = hold_last_ms = exit_origin_ms = power_notice_ms = 0;
	saw_exit = saw_cancel = saw_failed = false;
	handoff_release_ms = 0;
	action_count = action_index = 0; end_ms = 8000;
}
static void add_action(uint32_t at, enum action_kind kind, bool pressed)
{
	assert(action_count < sizeof(actions) / sizeof(actions[0]));
	assert(!action_count || actions[action_count - 1].at_ms < at);
	actions[action_count++] = (struct action){at, kind, pressed};
}
static void run_button(uint32_t finish)
{
	end_ms = finish; host_step(0);
	if (setjmp(script_finished) == 0) button_thread();
}
static void held_release_and_complete(void)
{
	reset_case(LED_CAP_RGB_PWM);
	handoff_release_ms = 1300;
	add_action(100, EDGE, true); add_action(1300, EDGE, false); add_action(1400, STALE_SUCCESS, false);
	run_button(4000);
	assert(hold_first_ms == 100 && hold_last_ms == 1300);
	assert(saw_exit && exit_origin_ms == 1100 && last_exit_value == 0);
	assert(abort_at_ms == 1300 + LED_MANUAL_EXIT_MS && !engine.button_hold_active);
	assert(click_notices == 0 && !saw_cancel && !saw_failed);
	assert(power_requests.request == (USER_SHUTDOWN_ENABLED ? SYS_POWER_REQ_SYSTEM_OFF : SYS_POWER_REQ_REBOOT));
	assert(USER_SHUTDOWN_ENABLED ? sys_system_off() : sys_system_reboot());
	assert(engine.quiesced && host_value == 0 && physical_offs + physical_reboots == 1);
	assert(host_now_ms - power_notice_ms == TRACKER_EVENT_POWER_FLUSH_MS);
}
static void qualified_release_preempts_older_click_deadline(void)
{
	reset_case(LED_CAP_RGB_PWM);
	add_action(100, EDGE, true); add_action(180, EDGE, false);
	add_action(190, EDGE, true); add_action(1190, EDGE, false);
	handoff_release_ms = 1190;
	run_button(4000);
	assert(power_requests.request == (USER_SHUTDOWN_ENABLED ? SYS_POWER_REQ_SYSTEM_OFF : SYS_POWER_REQ_REBOOT));
	assert(manual_request_attempts == 1 && !click_notices && !saw_failed);
	assert(saw_exit && exit_origin_ms == 1190 && last_exit_value == 0);
	assert(abort_at_ms == 1200 + LED_MANUAL_EXIT_MS && !engine.button_hold_active);
	assert(physical_reboots == 0 && physical_offs == 0);
}

static void short_hold_release_and_single_click(void)
{
	reset_case(LED_CAP_MONO_GPIO);
	add_action(100, EDGE, true); add_action(900, EDGE, false);
	run_button(5000);
	assert(hold_last_ms == 899 && !engine.button_hold_active && saw_exit);
	assert(click_notices == 1 && exit_origin_ms > 1900 && exit_origin_ms <= 1920);
	assert(abort_at_ms == 0); /* Short click retains the original thread lifecycle. */
	assert(power_requests.request == SYS_POWER_REQ_REBOOT && reset_requests == (CONFIG_USER_EXTRA_ACTIONS ? 1U : 0U));
}
static void physical_origin_and_duplicate_edges(void)
{
	reset_case(LED_CAP_RGB_PWM);
	host_now_ms = 101; physical_pressed = true;
	button_interrupt_handler(NULL, NULL, 1);
	assert(host_writes == 0); /* ISR only publishes facts, never driver work. */
	uint32_t generation = press_generation;
	host_step(351); /* Deliberately delay the first worker until 250ms of press. */
	assert(engine.winner.semantic == LED_BUTTON_HOLD && host_value == 10000);
	uint32_t identity = engine.winner.identity;
	host_now_ms = 601; button_interrupt_handler(NULL, NULL, 1); /* Duplicate pressed IRQ. */
	host_step(601);
	assert(host_value == 10000 && press_generation == generation && press_time == 101);
	assert(engine.winner.identity == identity && engine.winner.origin_ms == 101);
	assert(led_button_hold(generation, true, 601) == LED_DUPLICATE);
	host_step(851); assert(host_value == 10000 && engine.winner.identity == identity);
	host_step(1101); assert(host_value == 0);
	host_step(1351); assert(host_value == 10000);
	host_step(1851); assert(host_value == 5000);
	host_step(2351); assert(host_value == 0);
	host_step(2601); assert(host_value == 10000);
	host_step(3101); assert(host_value == 0);
	host_step(4101); assert(host_value == 0 && engine.winner.origin_ms == 101);
	reset_case(LED_CAP_MONO_GPIO);
	host_now_ms = 101; physical_pressed = true; button_interrupt_handler(NULL, NULL, 1);
	host_step(101); assert(host_value == 10000); /* Explicit binary fallback. */
	reset_case(LED_CAP_MONO_PWM);
	physical_pressed = true; button_interrupt_handler(NULL, NULL, 1);
	host_step(250); assert(host_value == 10000); /* Uptime zero is a real physical origin. */
	host_step(1000); assert(host_value == 0);
	reset_case(LED_CAP_MONO_PWM);
	host_now_ms = UINT32_MAX - 499U; physical_pressed = true;
	button_interrupt_handler(NULL, NULL, 1);
	host_step(0); assert(host_value == 10000); /* Uptime32 origin crosses wrap. */
	host_step(500); assert(host_value == 0);
}

static void release_boundaries_between_polls(void)
{
	for (unsigned duration = 999; duration <= 1001; ++duration) {
		reset_case(LED_CAP_RGB_PWM);
		uint32_t released = 105 + duration;
		add_action(105, EDGE, true); add_action(released, EDGE, false);
		if (duration >= LED_BUTTON_HOLD_MS) handoff_release_ms = released;
		run_button(5000);
		assert(saw_exit && last_exit_value == 0 && manual_request_attempts == 1);
		if (duration < LED_BUTTON_HOLD_MS) {
			assert(click_notices == 1 && !abort_at_ms);
			assert(exit_origin_ms > released + 1000 && exit_origin_ms <= released + 1040);
			assert(power_requests.request == SYS_POWER_REQ_REBOOT);
		} else {
			assert(!click_notices && exit_origin_ms == 1105);
			assert(abort_at_ms == 1120 + LED_MANUAL_EXIT_MS);
			assert(power_requests.request == (USER_SHUTDOWN_ENABLED ? SYS_POWER_REQ_SYSTEM_OFF : SYS_POWER_REQ_REBOOT));
		}
		assert(!engine.button_hold_active && !saw_cancel && !saw_failed);
	}
}
static void physical_press_priority_and_phase_restoration(void)
{
	reset_case(LED_CAP_RGB_PWM);
	host_begin(LED_OWNER_ACC, LED_COLLECT_STILL);
	assert(led_request_event(LED_OWNER_IMU, led_request_id(), led_event_id(), LED_FAILED) == LED_ADMITTED);
	host_now_ms = 101; physical_pressed = true; button_interrupt_handler(NULL, NULL, 1);
	host_step(351);
	assert(engine.winner.semantic == LED_BUTTON_HOLD && host_value == 10000);
	led_fault_publish(LED_OWNER_SYSTEM, LED_FAULT_SYSTEM, 0); host_step(401);
	assert(engine.winner.semantic == LED_BLOCKING_FAULT && host_role == LED_ROLE_NEGATIVE);
	led_fault_publish(LED_OWNER_SYSTEM, LED_FAULT_NONE, 0); host_step(601);
	assert(engine.winner.semantic == LED_BUTTON_HOLD && host_value == 10000);
	led_fault_publish(LED_OWNER_TCAL, LED_FAULT_SAFETY, 0); host_step(701);
	assert(engine.winner.semantic == LED_SAFETY_FAULT && host_role == LED_ROLE_NEGATIVE);
	led_fault_publish(LED_OWNER_TCAL, LED_FAULT_NONE, 0); host_step(851);
	assert(engine.winner.semantic == LED_BUTTON_HOLD && host_value == 10000);
	led_power_publish(LED_POWER_BATTERY, true); host_step(901);
	assert(engine.winner.semantic == LED_LOW_BATTERY);
	led_power_publish(LED_POWER_BATTERY, false); host_step(1101);
	assert(engine.winner.semantic == LED_BUTTON_HOLD && host_value == 0);
	led_quiesce(); host_step(1151);
	assert(engine.quiesced && host_value == 0);
}

static void deliberate_cancel_and_refusal(void)
{
	reset_case(LED_CAP_MONO_GPIO);
	add_action(100, EDGE, true); add_action(5500, EDGE, false);
	run_button(7200);
	assert(saw_cancel && !saw_exit && !engine.button_hold_active && !press_time);
	assert(power_requests.state == POWER_REQUEST_EMPTY && !abort_at_ms && !click_notices);
	assert(pair_requests == (CONFIG_USER_EXTRA_ACTIONS ? 0U : 1U));
	reset_case(LED_CAP_RGB_PWM);
	add_action(100, EDGE, true); add_action(1000, OTA_RESERVE, false); add_action(1300, EDGE, false);
	run_button(3500);
	assert(saw_failed && !saw_exit && !engine.button_hold_active && !press_time && !abort_at_ms);
	assert(power_requests.state == POWER_REQUEST_EMPTY && power_requests.ota_reboot == POWER_OTA_REBOOT_RESERVED);
	assert(click_notices == 0); /* Refused after release is not another click. */
	reset_case(LED_CAP_RGB_PWM);
	add_action(100, EDGE, true); add_action(900, OTA_ACTIVE, false); add_action(1500, EDGE, false);
	run_button(3500);
	assert(saw_failed && !saw_exit && !engine.button_hold_active && !press_time);
	assert(click_notices == 0 && power_requests.state == POWER_REQUEST_EMPTY && !abort_at_ms);
}
static void pending_new_tap(void)
{
	physical_pressed = true; host_now_ms = 1400; button_interrupt_handler(NULL, NULL, 1);
	physical_pressed = false; host_now_ms = 1500; button_interrupt_handler(NULL, NULL, 1);
}
static void pending_new_press(void)
{
	physical_pressed = false; host_now_ms = 1300; button_interrupt_handler(NULL, NULL, 1);
	physical_pressed = true; host_now_ms = 1400; button_interrupt_handler(NULL, NULL, 1);
}
static void irq_delivery_cannot_split_identity_and_clear(void)
{
	reset_case(LED_CAP_MONO_GPIO); host_now_ms = 1300;
	last_press_started_at = 100; last_press_duration = 1200;
	irq_deferred = pending_new_tap;
	button_release_consume(100);
	assert(!irq_depth && !irq_deferred && last_press_started_at == 1400 && last_press_duration == 100);
	reset_case(LED_CAP_MONO_GPIO); host_now_ms = 1300;
	press_time = 100; physical_pressed = true; irq_deferred = pending_new_press;
	press_generation = led_button_input();
	button_press_cancel(100);
	assert(!irq_depth && !irq_deferred && press_time == 1400 && physical_pressed);
	button_status = true;
	button_status_clear_if_idle();
	assert(button_status); /* Old cleanup cannot drop the new held input's veto. */
	reset_case(LED_CAP_MONO_GPIO); host_now_ms = 1300;
	last_press_started_at = 100; last_press_duration = 1200;
	irq_deferred = pending_new_tap;
	button_press_cancel(100);
	assert(last_press_started_at == 1400 && last_press_duration == 100);
}

static void fresh_input_survives_late_veto(void)
{
	reset_case(LED_CAP_RGB_PWM);
	add_action(100, EDGE, true); add_action(1300, EDGE, false);
	add_action(1400, EDGE, true); add_action(1500, EDGE, false);
	add_action(1600, OTA_RESERVE, false);
	run_button(2900);
	assert(saw_exit && saw_failed && !abort_at_ms && manual_request_attempts == 0);
	assert(click_notices == 1 && !press_time && !button_status);
	assert(power_requests.state == POWER_REQUEST_EMPTY);
	reset_case(LED_CAP_RGB_PWM);
	add_action(100, EDGE, true); add_action(1300, EDGE, false);
	add_action(1500, EDGE, true); add_action(1600, OTA_RESERVE, false);
	run_button(2300);
	/* The newer physical feedback outranks the old handler's ordinary refusal.
	 * Check its real terminal receipt, not visibility behind a held press. */
	assert(saw_exit && !saw_failed && !abort_at_ms && manual_request_attempts == 0);
	assert(engine.owners[LED_OWNER_SYSTEM].terminal == LED_FAILED);
	assert(engine.events[LED_OWNER_SYSTEM].present && engine.events[LED_OWNER_SYSTEM].semantic == LED_FAILED);
	assert(press_time == 1500 && physical_pressed && button_status && engine.button_hold_active);
	assert(engine.winner.semantic == LED_BUTTON_HOLD && engine.winner.origin_ms == 1500);
	assert(host_role == LED_ROLE_INTERACTION && host_value == 10000);
	assert(!click_notices && power_requests.state == POWER_REQUEST_EMPTY);
}

static void invisible_output_never_adds_manual_wait(void)
{
	reset_case(LED_CAP_NO_LED);
	add_action(100, EDGE, true); add_action(1300, EDGE, false);
	run_button(3500);
	assert(abort_at_ms == 1300 && !saw_exit && !host_writes);
	assert(power_requests.state == POWER_REQUEST_QUEUED);
	reset_case(LED_CAP_MONO_PWM); host_hardware.global_limit_pptt = 0;
	add_action(100, EDGE, true); add_action(1300, EDGE, false);
	run_button(3500);
	assert(abort_at_ms == 1300 && !saw_exit && !host_writes);
	assert(power_requests.state == POWER_REQUEST_QUEUED);
}

static void protective_priority_and_reversible_veto(void)
{
	reset_case(LED_CAP_RGB_PWM);
	add_action(100, EDGE, true); add_action(1300, EDGE, false); add_action(1600, PROTECTIVE_OFF, false);
	run_button(2500);
	assert(saw_exit && engine.quiesced && !engine.button_hold_active && physical_offs == 1);
	assert(power_notice_ms == 1600 && host_value == 0 && power_requests.physical_started);
	assert(!abort_at_ms && !press_time && manual_request_attempts == 0);
	reset_case(LED_CAP_RGB_PWM);
	add_action(100, EDGE, true); add_action(1300, EDGE, false); add_action(1600, OTA_RESERVE, false);
	run_button(3000);
	assert(saw_exit && saw_failed && engine.owners[LED_OWNER_SYSTEM].semantic == LED_NONE);
	assert(!abort_at_ms && !engine.button_hold_active && power_requests.state == POWER_REQUEST_EMPTY);
	assert(manual_request_attempts == 0);
	/* OTA begins after terminal single-click recognition but before commit.
	 * esb_ota_is_active alone must prevent the new delayed button request. */
	reset_case(LED_CAP_RGB_PWM);
	add_action(100, EDGE, true); add_action(900, EDGE, false); add_action(2220, OTA_ACTIVE, false);
	run_button(4000);
	assert(saw_exit && saw_failed && !abort_at_ms && manual_request_attempts == 0);
	assert(power_requests.state == POWER_REQUEST_EMPTY && engine.owners[LED_OWNER_SYSTEM].semantic == LED_NONE);
	/* Reservation appears in the final sleep, after the last loop check. */
	reset_case(LED_CAP_MONO_GPIO);
	add_action(100, EDGE, true); add_action(1300, EDGE, false); add_action(3100, OTA_RESERVE, false);
	run_button(4000);
	assert(saw_exit && saw_failed && !abort_at_ms && manual_request_attempts == 0);
	assert(power_requests.state == POWER_REQUEST_EMPTY && engine.owners[LED_OWNER_SYSTEM].semantic == LED_NONE);
	reset_case(LED_CAP_RGB_PWM);
	add_action(100, EDGE, true); add_action(1300, EDGE, false);
	add_action(1600, SAFETY_ON, false); add_action(1800, SAFETY_OFF, false);
	run_button(4000);
	assert(abort_at_ms == 1300 + LED_MANUAL_EXIT_MS && exit_origin_ms == 1100);
	reset_case(LED_CAP_MONO_GPIO);
	add_action(100, EDGE, true); add_action(1300, EDGE, false); add_action(1600, LOW_ON, false);
	run_button(4000);
	assert(abort_at_ms == 1300 + LED_MANUAL_EXIT_MS && engine.winner.semantic == LED_LOW_BATTERY);
}
static void stale_hold_and_command_budget(void)
{
	reset_case(LED_CAP_MONO_GPIO);
	uint32_t old = led_button_input();
	assert(led_button_hold(old, true, 0) == LED_ADMITTED);
	uint32_t current = led_button_input();
	assert(led_button_hold(old, true, 0) == LED_STALE && !engine.button_hold_active);
	assert(led_button_hold(current, true, 0) == LED_ADMITTED);
	assert(led_button_hold(old, false, 0) == LED_STALE && engine.button_hold_active);
	led_quiesce();
	assert(led_button_hold(old, true, 0) == LED_SHUTDOWN);
	reset_case(LED_CAP_MONO_GPIO);
	assert(sys_command_shutdown_request(led_request_id(), led_event_id(), led_event_id()) == 0);
	assert(host_now_ms == 1500 && power_requests.request == SYS_POWER_REQ_SYSTEM_OFF);
	assert(engine.winner.semantic == LED_EXIT_PENDING && host_value == 0);
	reset_case(LED_CAP_MONO_GPIO);
	assert(sys_user_reboot() == 0);
	assert(host_now_ms == 0 && power_requests.request == SYS_POWER_REQ_REBOOT);
	assert(engine.owners[LED_OWNER_SYSTEM].semantic != LED_MANUAL_EXIT);
}
#define DT_NODE_HAS_PROP(...) 0
#define IGNORE_RESET 1
static uint8_t reboot_counter_read(void) { return 100; }
static bool dock_read(void) { return false; }
static bool chg_read(void) { return false; }
static bool stby_read(void) { return false; }
static bool vin_read(void) { return false; }
static int64_t system_uptime_since_boot_ms(void) { return host_now_ms; }
static void k_usleep(int microseconds) { (void)microseconds; }
#define main startup_main
#include "startup_production.inc"
#undef main

static void startup_does_not_fake_button_input(void)
{
	reset_case(LED_CAP_RGB_PWM);
	assert(startup_main() == 0);
	host_step(0); host_step(120);
	assert(host_value == 0 && engine.winner.semantic == LED_NONE && !engine.ready);
}

int main(void)
{
	qualified_release_preempts_older_click_deadline();
	startup_does_not_fake_button_input();
	held_release_and_complete(); short_hold_release_and_single_click();
	fresh_input_survives_late_veto(); invisible_output_never_adds_manual_wait();
	irq_delivery_cannot_split_identity_and_clear();
	physical_origin_and_duplicate_edges(); release_boundaries_between_polls();
	physical_press_priority_and_phase_restoration();
	deliberate_cancel_and_refusal(); protective_priority_and_reversible_veto(); stale_hold_and_command_budget();
	puts("Actual button ISR/thread/power owner: physical-origin full/marker/linear fade, 999/1000/1001ms qualification, inherited release phase, provenance races, cancellation/refusal and unchanged short/protected/command paths passed");
	return 0;
}
