#include <ctype.h>
#include <stdarg.h>
#include <stdlib.h>
#include "../led_sync/test_runtime.h"
#include "../../../src/parse_args.h"
#include "../../../src/system/led_debug.h"

static char response[16384];
static size_t response_length;

int printk(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	int count = vsnprintf(response + response_length, sizeof(response) - response_length, format, args);
	va_end(args);
	assert(count >= 0 && (size_t)count < sizeof(response) - response_length);
	response_length += (size_t)count;
	return count;
}

static void command(uint32_t session, const char *text)
{
	char line[256];
	assert(strlen(text) < sizeof(line));
	strcpy(line, text);
	response[0] = '\0';
	response_length = 0;
	char *argv[8] = {0};
	size_t argc = parse_args(line, argv, sizeof(argv) / sizeof(argv[0]));
	for (size_t i = 0; i < argc; i++) {
		for (char *p = argv[i]; *p != '\0'; p++) {
			*p = (char)tolower((unsigned char)*p);
		}
	}
	if (argc != 0) {
		led_debug_command(session, argc, argv);
	}
}

static struct led_preview_record preview(void)
{
	struct led_preview_record result;
	led_preview_snapshot(&result);
	return result;
}

static void reset(enum led_capability capability)
{
	host_reset(capability);
	led_debug_session_start(1);
}

static void test_null_snapshots_are_readonly(void)
{
	reset(LED_CAP_RGB_PWM);
	command(1, "led play success");
	host_step(0);
	struct led_engine before;
	memcpy(&before, &engine, sizeof(before));
	led_changed.count = 0;
	led_preview_snapshot(NULL);
	led_policy_snapshot(NULL);
	assert(led_preview_start(1, LED_SUCCESS, -1, 0, NULL) == LED_PREVIEW_INVALID);
	assert(memcmp(&before, &engine, sizeof(before)) == 0);
	assert(led_changed.count == 0);
	host_step(600);
	assert(preview().status == LED_PREVIEW_ACTIVE && host_value == 10000);
	host_step(2600);
	assert(preview().status == LED_PREVIEW_COMPLETED);
}

static void test_invalid_preserves_live_preview(void)
{
	static const char *invalid[] = {
		"led play unknown", "led play charging -1", "led play charging +1000",
		"led play charging 4294967296", "led play charging 999999999999999999999999999999",
		"led play charging 1000x", "led play charging 249", "led play charging 30001",
		"led play charging 0", "led play charging 1000 extra", "led play success 1000",
		"led color cyan", "led color red -250", "led stop extra", "led list extra",
		"led play charging 1000 a b c d e f"
	};
	reset(LED_CAP_RGB_PWM);
	command(1, "LED PLAY CHARGING 10000");
	struct led_preview_record original = preview();
	assert(original.status == LED_PREVIEW_ACTIVE && original.semantic == LED_CHARGING);
	for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++) {
		command(1, invalid[i]);
		struct led_preview_record current = preview();
		assert(current.generation == original.generation && current.status == LED_PREVIEW_ACTIVE);
		assert(current.origin_ms == original.origin_ms && current.expires_ms == original.expires_ms);
		assert(current.semantic == LED_CHARGING && current.console_session == original.console_session);
	}
}

static void test_query_and_stop_restore_current_truth(void)
{
	reset(LED_CAP_RGB_PWM);
	struct led_token calibration = host_begin(LED_OWNER_IMU, LED_WAIT_STILL);
	command(1, "led play charging 10000");
	struct led_preview_record original = preview();
	assert(original.status == LED_PREVIEW_ACTIVE);
	(void)host_step(200);
	struct led_policy_view before, after;
	led_policy_snapshot(&before);
	command(1, "led status");
	command(1, "led list");
	command(1, "led help");
	led_policy_snapshot(&after);
	assert(before.winner.semantic == after.winner.semantic && before.winner.origin_ms == after.winner.origin_ms);
	assert(before.low_due_ms == after.low_due_ms);
	assert(preview().generation == original.generation && preview().expires_ms == original.expires_ms);
	assert(led_state(calibration, 2, LED_WAIT_MOVE) == LED_ADMITTED);
	command(1, "led stop");
	(void)host_step(201);
	led_policy_snapshot(&after);
	assert(preview().status == LED_PREVIEW_STOPPED && after.winner.semantic == LED_WAIT_MOVE);
	command(1, "led stop");
	assert(preview().status == LED_PREVIEW_STOPPED);

	reset(LED_CAP_RGB_PWM);
	led_request_event(LED_OWNER_IMU, led_request_id(), led_event_id(), LED_FAILED);
	led_policy_snapshot(&before);
	command(1, "led list");
	command(1, "led status");
	led_policy_snapshot(&after);
	assert(before.event_remaining_ms[LED_OWNER_IMU] != 0);
	assert(after.event_remaining_ms[LED_OWNER_IMU] == before.event_remaining_ms[LED_OWNER_IMU]);
	assert(after.winner.identity == before.winner.identity && after.winner.origin_ms == before.winner.origin_ms);
}

static void test_finite_black_gap_and_completion(void)
{
	reset(LED_CAP_RGB_PWM);
	(void)host_begin(LED_OWNER_IMU, LED_WAIT_STILL);
	(void)host_step(0);
	assert(host_value != 0);
	command(1, "led play success");
	struct led_preview_record record = preview();
	(void)host_step(0);
	assert(host_value == 0);
	(void)host_step(599);
	assert(host_value == 0 && preview().status == LED_PREVIEW_ACTIVE);
	(void)host_step(600);
	assert(host_value == 10000 && host_role == LED_ROLE_POSITIVE);
	command(1, "led list");
	assert(strstr(response, "style=success") != NULL);
	uint32_t end=record.origin_ms+600+led_behavior_get(LED_SUCCESS)->duration_ms;
	for (uint32_t time=601;time<end;++time) {
		(void)host_step(time);
		assert(preview().status==LED_PREVIEW_ACTIVE);
		uint32_t age=time-600;
		assert(host_value==(age<1400 && age%400<200 ? 10000 : 0));
	}
	(void)host_step(end);
	assert(preview().status == LED_PREVIEW_COMPLETED);
	struct led_policy_view real;
	led_policy_snapshot(&real);
	assert(real.winner.semantic == LED_WAIT_STILL);

	reset(LED_CAP_RGB_PWM);
	command(1, "led play success");
	(void)host_step(1200); /* Too little deadline remains for a complete envelope and start gap. */
	assert(preview().status == LED_PREVIEW_EXPIRED && host_role != LED_ROLE_POSITIVE);
}

static void test_replacement_and_deadlines(void)
{
	reset(LED_CAP_RGB_PWM);
	command(1, "led play success");
	(void)host_step(0);
	(void)host_step(600);
	uint32_t old_generation = preview().generation;
	command(1, "led color blue 3000");
	struct led_preview_record replacement = preview();
	assert(replacement.generation != old_generation && replacement.color_override == LED_PHYSICAL_BLUE);
	(void)host_step(2601); /* Superseded finite completion must not clear the replacement color. */
	assert(preview().generation == replacement.generation && preview().status == LED_PREVIEW_ACTIVE);
	command(1, "led status");
	assert(preview().expires_ms == replacement.expires_ms);
	(void)host_step(replacement.expires_ms - 1);
	assert(preview().status == LED_PREVIEW_ACTIVE);
	(void)host_step(replacement.expires_ms);
	assert(preview().status != LED_PREVIEW_ACTIVE);
	(void)host_step(replacement.expires_ms + 5000);
	assert(preview().status != LED_PREVIEW_ACTIVE);

	reset(LED_CAP_RGB_PWM);
	command(1, "led color white 250");
	uint32_t next = host_step(0);
	assert(next <= 250); /* A static preview cannot hide its expiry behind K_FOREVER. */
	(void)host_step(250);
	assert(preview().status != LED_PREVIEW_ACTIVE);
}

static void test_console_session_retirement(void)
{
	reset(LED_CAP_RGB_PWM);
	command(1, "led play charging 10000");
	led_debug_disconnect(1);
	assert(preview().status != LED_PREVIEW_ACTIVE);
	command(1, "led color red 1000"); /* Work parsed before disconnect, delivered late. */
	assert(preview().status != LED_PREVIEW_ACTIVE);
	led_debug_session_start(2);
	command(2, "led color green 1000");
	uint32_t generation = preview().generation;
	assert(preview().status == LED_PREVIEW_ACTIVE && preview().console_session == 2);
	led_debug_disconnect(1);
	assert(preview().generation == generation && preview().status == LED_PREVIEW_ACTIVE);
	led_debug_disconnect(2);
	assert(preview().status != LED_PREVIEW_ACTIVE);
}

static void expect_busy(void)
{
	command(1, "led play charging 10000");
	assert(strstr(response, "result=busy") != NULL);
	assert(preview().status != LED_PREVIEW_ACTIVE);
}

static void test_real_blockers_and_terminal_gate(void)
{
	reset(LED_CAP_RGB_PWM);
	led_fault_publish(LED_OWNER_TCAL, LED_FAULT_SAFETY, 0);
	expect_busy();
	reset(LED_CAP_RGB_PWM);
	led_fault_publish(LED_OWNER_SENSOR, LED_FAULT_SENSOR, 0);
	expect_busy();
	reset(LED_CAP_RGB_PWM);
	led_power_publish(LED_POWER_BATTERY, true);
	expect_busy();
	reset(LED_CAP_RGB_PWM);
	led_request_event(LED_OWNER_SYSTEM, led_request_id(), led_event_id(), LED_FAILED);
	expect_busy();
	reset(LED_CAP_RGB_PWM);
	led_identify();
	expect_busy();
	reset(LED_CAP_RGB_PWM);
	led_operation_publish(LED_OWNER_SYSTEM, true, false); /* OTA reservation/reboot lock, not only visible P6. */
	expect_busy();
	reset(LED_CAP_RGB_PWM);
	led_operation_publish(LED_OWNER_TCAL, false, true); /* Heater reservation/finalization. */
	expect_busy();
	reset(LED_CAP_RGB_PWM);
	assert(led_button_hold(led_button_input(),true,0)==LED_ADMITTED);
	expect_busy();
	reset(LED_CAP_RGB_PWM);
	led_quiesce();
	command(1, "led color red 1000");
	assert(strstr(response, "result=shutdown") != NULL && preview().status != LED_PREVIEW_ACTIVE);
	command(1, "led stop");
	struct led_policy_view real;
	led_policy_snapshot(&real);
	assert(real.quiesced);
}

static void test_preemption_never_resumes(void)
{
	for (unsigned cause = 0; cause < 8; cause++) {
		reset(LED_CAP_RGB_PWM);
		command(1, "led color blue 10000");
		(void)host_step(0);
		assert(preview().status == LED_PREVIEW_ACTIVE);
		switch (cause) {
		case 0: led_power_publish(LED_POWER_BATTERY, true); break;
		case 1: led_fault_publish(LED_OWNER_SENSOR, LED_FAULT_SENSOR, 0); break;
		case 2: led_identify(); break;
		case 3: led_operation_publish(LED_OWNER_SYSTEM, true, false); break;
		case 4: led_operation_publish(LED_OWNER_TCAL, false, true); break;
		case 5: led_fault_publish(LED_OWNER_SENSOR, LED_FAULT_SAFETY, 0); break;
		case 6:
			led_request_event(LED_OWNER_SYSTEM, led_request_id(), led_event_id(), LED_FAILED);
			break;
		case 7: assert(led_button_hold(led_button_input(),true,0)==LED_ADMITTED); break;
		}
		(void)host_step(1);
		assert(preview().status == LED_PREVIEW_PREEMPTED);
		led_power_publish(LED_POWER_BATTERY, false);
		led_fault_publish(LED_OWNER_SENSOR, LED_FAULT_NONE, 0);
		led_operation_publish(LED_OWNER_SYSTEM, false, false);
		led_operation_publish(LED_OWNER_TCAL, false, false);
		if (cause==7) assert(led_button_hold(engine.button_generation,false,0)==LED_ADMITTED);
		(void)host_step(7000);
		assert(preview().status == LED_PREVIEW_PREEMPTED);
	}

	reset(LED_CAP_RGB_PWM);
	command(1, "led color blue 10000");
	(void)host_step(0);
	led_quiesce();
	(void)host_step(1);
	assert(preview().status != LED_PREVIEW_ACTIVE && host_value == 0);
	command(1, "led stop");
	(void)host_step(2);
	assert(host_value == 0);
}

static void test_capabilities_and_no_business_effects(void)
{
	reset(LED_CAP_NO_LED);
	uint32_t initial_generation = preview().generation;
	command(1, "led play charging");
	assert(strstr(response, "result=unsupported") != NULL && preview().generation == initial_generation);
	command(1, "led color red");
	assert(strstr(response, "result=unsupported") != NULL && preview().generation == initial_generation);
	command(1, "led help");
	command(1, "led list");
	command(1, "led");
	command(1, "led status");
	command(1, "led stop");
	assert(preview().status != LED_PREVIEW_ACTIVE);
	reset(LED_CAP_MONO_GPIO);
	initial_generation = preview().generation;
	command(1, "led color blue");
	assert(strstr(response, "result=unsupported") != NULL && preview().generation == initial_generation);
	reset(LED_CAP_RGB_PWM);
	host_hardware.global_limit_pptt = 0;
	command(1, "led play charging");
	assert(preview().muted && strstr(response, "muted") != NULL);
	assert(host_hardware.global_limit_pptt == 0);
	struct led_preview_record muted = preview();
	uint32_t wake_ms = host_step(0);
	assert(wake_ms <= muted.expires_ms && host_value == 0 && !host_hardware.powered);
	(void)host_step(muted.expires_ms);
	assert(preview().status != LED_PREVIEW_ACTIVE && host_hardware.global_limit_pptt == 0);

	static const char *appearance[] = {
		"success", "safety_fault", "exit_pending", "heated_active", "ota_active", "identify"
	};
	for (size_t i = 0; i < sizeof(appearance) / sizeof(appearance[0]); i++) {
		reset(LED_CAP_RGB_PWM);
		char text[80];
		snprintf(text, sizeof(text), "led play %s", appearance[i]);
		command(1, text);
		assert(preview().status == LED_PREVIEW_ACTIVE);
		(void)host_step(120);
		struct led_policy_view real;
		led_policy_snapshot(&real);
		assert(!real.quiesced && !real.ota_active && !real.heated_active);
		assert(real.winner.priority >= LED_PRIORITY_TASK); /* Synthetic appearances never become real high-priority facts. */
		for (enum led_owner owner = LED_OWNER_IMU; owner < LED_OWNER_COUNT; owner++) {
			assert(real.event_remaining_ms[owner] == 0);
		}
	}
}

static void test_duration_boundary_and_uptime_wrap(void)
{
	reset(LED_CAP_RGB_PWM);
	command(1, "led color red 30000");
	struct led_preview_record record = preview();
	assert(record.status == LED_PREVIEW_ACTIVE && record.expires_ms - record.origin_ms == 30000);
	(void)host_step(record.expires_ms - 1);
	assert(preview().status == LED_PREVIEW_ACTIVE);
	(void)host_step(record.expires_ms);
	assert(preview().status != LED_PREVIEW_ACTIVE);

	reset(LED_CAP_RGB_PWM);
	host_now_ms = UINT32_MAX - 100U;
	led_debug_session_start(2);
	command(2, "led color amber 250");
	record = preview();
	assert(record.status == LED_PREVIEW_ACTIVE && record.expires_ms - record.origin_ms == 250);
	(void)host_step(UINT32_MAX);
	assert(preview().status == LED_PREVIEW_ACTIVE);
	(void)host_step(record.expires_ms);
	assert(preview().status != LED_PREVIEW_ACTIVE);
}

static void test_ready_preview_cannot_change_link_truth(void)
{
	reset(LED_CAP_RGB_PWM);
	struct led_connection_facts facts = {
		.healthy = false, .output_ready = true, .radio_required = true, .paired = true
	};
	led_connection_publish(&facts);
	host_step(200);
	assert(engine.winner.semantic==LED_RECONNECTING && !engine.ready);
	command(1,"led play ready 1000");
	assert(preview().status==LED_PREVIEW_ACTIVE);
	host_step(300);
	assert(!engine.ready);
	struct led_policy_view real;
	led_policy_snapshot(&real);
	assert(real.winner.semantic==LED_RECONNECTING);
	command(1,"led stop");
	host_step(301);
	assert(engine.winner.semantic==LED_RECONNECTING && !engine.ready);
	facts.healthy=true;
	led_connection_publish(&facts);
	host_step(302);
	assert(engine.winner.semantic==LED_READY && engine.ready && !host_value);
	host_step(10000);
	assert(engine.winner.semantic==LED_READY && host_value==10000);
	host_step(10199);
	assert(engine.winner.semantic==LED_READY && host_value==10000);
	host_step(10201);
	assert(engine.winner.semantic==LED_READY && !host_value);
}

static void test_status_reports_actual_clock(void)
{
	reset(LED_CAP_RGB_PWM);
	const struct led_connection_facts ready = {
		.healthy = true, .output_ready = true, .radio_required = true, .paired = true
	};
	led_connection_publish(&ready);
	(void)host_step(0);
	(void)host_step(600);
	command(1, "led status");
	assert(strstr(response, "time_source=local_monotonic") != NULL);
#if CONFIG_LED_NETWORK_SYNC
	host_network_available = true;
	host_network_raw = 32768;
	(void)host_step(host_now_ms+100);
	command(1, "led status");
	assert(strstr(response, "time_source=network_fresh") != NULL);
	host_network_available = false;
	(void)host_step(host_now_ms+100);
	command(1, "led status");
	assert(strstr(response, "time_source=network_held") != NULL);
#endif
	command(1, "led color red 1000");
	(void)host_step(host_now_ms+100);
	command(1, "led status");
	assert(strstr(response, "time_source=local_monotonic") != NULL);
}

static void test_physical_feedback_cannot_use_preview_color(void)
{
	reset(LED_CAP_RGB_PWM);
	command(1,"led color blue 10000");
	host_step(0); assert(host_color==LED_PHYSICAL_BLUE && host_value==10000);
	host_now_ms=100;
	uint32_t generation=led_button_input();
	assert(led_button_hold(generation,true,100)==LED_ADMITTED);
	host_step(100);
	assert(preview().status==LED_PREVIEW_PREEMPTED);
	assert(engine.winner.semantic==LED_BUTTON_HOLD && host_color==-1 && host_value==10000);
	expect_busy();
	host_step(600); assert(host_value==10000 && host_color==-1 && host_role==LED_ROLE_INTERACTION);
	host_step(1100); assert(host_value==0);
	struct led_token exit=led_begin(LED_OWNER_SYSTEM,led_request_id());
	assert(led_button_exit(exit,1,generation)==LED_ADMITTED);
	host_step(1100);
	assert(engine.winner.semantic==LED_MANUAL_EXIT && host_value==0 && host_color==-1 && engine.winner.origin_ms==1100);
	assert(led_state(exit,2,LED_NONE)==LED_ADMITTED);
	host_step(1101); assert(preview().status==LED_PREVIEW_PREEMPTED && host_color==-1);
	/* A nonbutton manual state also preempts an already active preview despite
	 * retaining task-priority descriptor metadata; it must not resume after cancellation. */
	reset(LED_CAP_RGB_PWM);
	command(1,"led color blue 10000");
	host_step(0);
	host_now_ms=100;
	exit=led_begin(LED_OWNER_SYSTEM,led_request_id());
	assert(led_state(exit,1,LED_MANUAL_EXIT)==LED_ADMITTED);
	host_step(100);
	assert(preview().status==LED_PREVIEW_PREEMPTED && engine.winner.semantic==LED_MANUAL_EXIT);
	assert(host_value==0 && host_color==-1 && host_role==LED_ROLE_INTERACTION);
	assert(led_state(exit,2,LED_NONE)==LED_ADMITTED);
	host_step(101); assert(preview().status==LED_PREVIEW_PREEMPTED && host_color==-1);
}

int main(void)
{
	test_null_snapshots_are_readonly();
	test_invalid_preserves_live_preview();
	test_query_and_stop_restore_current_truth();
	test_finite_black_gap_and_completion();
	test_replacement_and_deadlines();
	test_console_session_retirement();
	test_real_blockers_and_terminal_gate();
	test_preemption_never_resumes();
	test_capabilities_and_no_business_effects();
	test_duration_boundary_and_uptime_wrap();
	test_ready_preview_cannot_change_link_truth();
	test_status_reports_actual_clock();
	test_physical_feedback_cannot_use_preview_color();
	puts("actual LED debug parser/state: invalid, identity, expiry, priority, capability and isolation passed");
	return 0;
}
