#include "led_debug.h"

#ifdef CONFIG_LED_DEBUG
#include "led_internal.h"

#include <limits.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

static enum led_semantic find_semantic(const char *name)
{
	for (enum led_semantic semantic = LED_INPUT_ACK; semantic < LED_SEMANTIC_COUNT; semantic++) {
		if (strcmp(name, led_behavior_name(semantic)) == 0) {
			return semantic;
		}
	}
	return LED_NONE;
}

static const char *color_name(int color)
{
	switch (color) {
	case LED_PHYSICAL_RED:
		return "red";
	case LED_PHYSICAL_GREEN:
		return "green";
	case LED_PHYSICAL_BLUE:
		return "blue";
	case LED_PHYSICAL_AMBER:
		return "amber";
	case LED_PHYSICAL_WHITE:
		return "white";
	default:
		return "none";
	}
}

static int find_color(const char *name)
{
	static const enum led_physical_color colors[]
		= {LED_PHYSICAL_RED, LED_PHYSICAL_GREEN, LED_PHYSICAL_BLUE, LED_PHYSICAL_AMBER, LED_PHYSICAL_WHITE};
	for (size_t i = 0; i < sizeof(colors) / sizeof(colors[0]); i++) {
		if (strcmp(name, color_name(colors[i])) == 0) {
			return colors[i];
		}
	}
	return -1;
}

static const char *capability_name(enum led_capability capability)
{
	switch (capability) {
	case LED_CAP_NO_LED:
		return "no_led";
	case LED_CAP_MONO_GPIO:
		return "mono_gpio";
	case LED_CAP_MONO_PWM:
		return "mono_pwm";
	case LED_CAP_DUAL_DISCRETE:
		return "dual_discrete";
	case LED_CAP_RG_MIX:
		return "rg_mix";
	case LED_CAP_TRI_DISCRETE:
		return "tri_discrete";
	case LED_CAP_RGB_PWM:
		return "rgb_pwm";
	case LED_CAP_RGB_PIXEL:
		return "rgb_pixel";
	default:
		return "unknown";
	}
}

static const char *degradation_name(const struct led_hardware_info *hardware)
{
	switch (hardware->capability) {
	case LED_CAP_NO_LED:
		return "no_visual_output";
	case LED_CAP_MONO_GPIO:
		return "single_physical_color_explicit_gpio_envelope";
	case LED_CAP_MONO_PWM:
		return "single_physical_color";
	case LED_CAP_DUAL_DISCRETE:
	case LED_CAP_TRI_DISCRETE:
		return "fixed_physical_role_color_no_mixing";
	case LED_CAP_RG_MIX:
		return "blue_white_use_board_neutral";
	case LED_CAP_RGB_PWM:
	case LED_CAP_RGB_PIXEL:
		return "native_role_mapping";
	default:
		return "unknown";
	}
}

static const char *result_name(enum led_preview_result result)
{
	switch (result) {
	case LED_PREVIEW_ADMITTED:
		return "admitted";
	case LED_PREVIEW_UNSUPPORTED:
		return "unsupported";
	case LED_PREVIEW_BUSY:
		return "busy";
	case LED_PREVIEW_INVALID:
		return "invalid";
	case LED_PREVIEW_SHUTDOWN:
		return "shutdown";
	default:
		return "invalid";
	}
}

static const char *time_source_name(enum led_time_source source)
{
	switch (source) {
	case LED_TIME_LOCAL:
		return "local_monotonic";
	case LED_TIME_NETWORK_FRESH:
		return "network_fresh";
	case LED_TIME_NETWORK_HELD:
		return "network_held";
	default:
		return "unknown";
	}
}

static const char *status_name(enum led_preview_status status)
{
	switch (status) {
	case LED_PREVIEW_NONE:
		return "none";
	case LED_PREVIEW_ACTIVE:
		return "active";
	case LED_PREVIEW_COMPLETED:
		return "completed";
	case LED_PREVIEW_STOPPED:
		return "stopped";
	case LED_PREVIEW_PREEMPTED:
		return "preempted";
	case LED_PREVIEW_EXPIRED:
		return "expired";
	default:
		return "none";
	}
}

/* Unlike strtoul, this accepts only a complete unsigned decimal token, with
 * checked accumulation before multiplication. Invalid input is never clamped. */
static bool parse_duration(const char *token, uint32_t *duration_ms)
{
	uint32_t value = 0;
	if (token == NULL || *token == '\0') {
		return false;
	}
	for (const char *p = token; *p != '\0'; p++) {
		if (*p < '0' || *p > '9') {
			return false;
		}
		uint32_t digit = (uint32_t)(*p - '0');
		if (value > (UINT32_MAX - digit) / 10U) {
			return false;
		}
		value = value * 10U + digit;
	}
	if (value < 250U || value > 30000U) {
		return false;
	}
	*duration_ms = value;
	return true;
}

void led_debug_help(void)
{
	printk("led [help|list|status|stop]\n");
	printk("led play <semantic> [duration_ms]\n");
	printk("led color <red|green|blue|amber|white> [duration_ms]\n");
	printk("Finite semantic: one complete envelope, no duration argument.\n");
	printk("Loop/hold/color: decimal 250..30000 ms; previews never execute business operations.\n");
}

static void print_list(void)
{
	struct led_hardware_info hardware;
	led_hw_info(&hardware);
	printk("board=%s degradation=%s\n", capability_name(hardware.capability), degradation_name(&hardware));
	for (enum led_semantic semantic = LED_INPUT_ACK; semantic < LED_SEMANTIC_COUNT; semantic++) {
		const struct led_behavior *behavior = led_behavior_get(semantic);
		if (behavior == NULL) {
			continue;
		}
		printk(
			"%s extent=%s style=%s role=%s level=%s envelope_ms=%u board=%s\n",
			led_behavior_name(semantic),
			led_extent_name(behavior->extent),
			led_style_name(behavior->style),
			led_role_name(behavior->role),
			led_level_name(behavior->level),
			behavior->duration_ms,
			degradation_name(&hardware)
		);
	}
}

static uint32_t remaining_ms(uint32_t now_ms, uint32_t deadline_ms)
{
	int32_t remaining = (int32_t)(deadline_ms - now_ms);
	return remaining > 0 ? (uint32_t)remaining : 0;
}

static void print_status(void)
{
	struct led_policy_view real;
	struct led_hardware_info hardware;
	struct led_preview_record preview;
	led_policy_snapshot(&real);
	led_hw_info(&hardware);
	led_preview_snapshot(&preview);
	uint32_t now_ms = (uint32_t)k_uptime_get();
	printk(
		"hardware=%s global_limit_pptt=%u board_limit_pptt=%u visible_min_pptt=%u powered=%u\n",
		capability_name(hardware.capability),
		hardware.global_limit_pptt,
		hardware.board_limit_pptt,
		hardware.visible_min_pptt,
		hardware.powered
	);
	printk(
		"real_winner=%s owner=%u priority=P%u time_source=%s shutdown=%u ota=%u heated=%u\n",
		led_behavior_name(real.winner.semantic),
		real.winner.owner,
		real.winner.priority,
		time_source_name(real.time_source),
		real.quiesced,
		real.ota_active,
		real.heated_active
	);
	printk(
		"low=%s window_active=%u next_deadline_ms=%u remaining_ms=%u\n",
		real.low ? "active" : "inactive",
		real.low_active,
		real.low ? real.low_due_ms : 0,
		real.low ? remaining_ms(now_ms, real.low_due_ms) : 0
	);
	printk("driver_last_error=%d driver_acknowledgement=unknown\n", hardware.last_error);
	for (enum led_owner owner = LED_OWNER_IMU; owner < LED_OWNER_COUNT; owner++) {
		if (real.event_remaining_ms[owner] != 0) {
			printk("event_owner=%u remaining_ttl_ms=%u\n", owner, real.event_remaining_ms[owner]);
		}
	}
	printk(
		"preview_id=%u session=%u source=synthetic name=%s status=%s result=%s blocker=%s "
		"remaining_ms=%u muted=%u\n",
		preview.generation,
		preview.console_session,
		preview.color_override >= 0 ? color_name(preview.color_override) : led_behavior_name(preview.semantic),
		status_name(preview.status),
		result_name(preview.result),
		led_behavior_name(preview.blocker),
		preview.status == LED_PREVIEW_ACTIVE ? remaining_ms(now_ms, preview.expires_ms) : 0,
		preview.muted
	);
}

static void invalid(const char *name)
{
	printk("preview_id=0 name=%s result=invalid (existing preview unchanged)\n", name);
}

void led_debug_command(uint32_t console_session, size_t argc, char **argv)
{
	if (argc == 1 || (argc == 2 && strcmp(argv[1], "help") == 0)) {
		led_debug_help();
		return;
	}
	if (argc == 2 && strcmp(argv[1], "list") == 0) {
		print_list();
		return;
	}
	if (argc == 2 && strcmp(argv[1], "status") == 0) {
		print_status();
		return;
	}
	if (argc == 2 && strcmp(argv[1], "stop") == 0) {
		led_preview_stop();
		print_status();
		return;
	}
	if (argc < 3 || argc > 4) {
		invalid(argc > 1 ? argv[1] : "none");
		return;
	}

	enum led_semantic semantic = LED_NONE;
	int color = -1;
	uint32_t duration_ms;
	if (strcmp(argv[1], "play") == 0) {
		semantic = find_semantic(argv[2]);
		const struct led_behavior *behavior = led_behavior_get(semantic);
		if (behavior == NULL) {
			invalid(argv[2]);
			return;
		}
		if (behavior->extent == LED_FINITE) {
			if (argc != 3) {
				invalid(argv[2]);
				return;
			}
			duration_ms = 0;
		} else {
			duration_ms = behavior->extent == LED_HOLD
							? 3000U
							: (behavior->duration_ms < 1500U ? 3000U : behavior->duration_ms * 2U);
			if (duration_ms > 30000U || (argc == 4 && !parse_duration(argv[3], &duration_ms))) {
				invalid(argv[2]);
				return;
			}
		}
	} else if (strcmp(argv[1], "color") == 0) {
		color = find_color(argv[2]);
		if (color < 0) {
			invalid(argv[2]);
			return;
		}
		duration_ms = 2000;
		if (argc == 4 && !parse_duration(argv[3], &duration_ms)) {
			invalid(argv[2]);
			return;
		}
	} else {
		invalid(argv[1]);
		return;
	}

	struct led_preview_record record = {0};
	enum led_preview_result result = led_preview_start(console_session, semantic, color, duration_ms, &record);
	printk(
		"preview_id=%u name=%s deadline_ms=%u result=%s blocker=%s%s\n",
		record.generation,
		color >= 0 ? color_name(color) : led_behavior_name(semantic),
		record.expires_ms,
		result_name(result),
		led_behavior_name(record.blocker),
		record.muted ? " muted (not visible)" : ""
	);
}

void led_debug_session_start(uint32_t console_session)
{
	led_preview_session_start(console_session);
}

void led_debug_disconnect(uint32_t console_session)
{
	led_preview_disconnect(console_session);
}
#endif
