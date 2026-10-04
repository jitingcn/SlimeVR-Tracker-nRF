#include "led_internal.h"

/* Presentation descriptors carry no semantic identity. Truly identical facts
 * share readonly storage; the byte map retains their distinct business names. */
enum behavior_id {
	BEHAVIOR_ACK,
	BEHAVIOR_CANCELLED,
	BEHAVIOR_SUCCESS,
	BEHAVIOR_NEGATIVE,
	BEHAVIOR_PARTIAL,
	BEHAVIOR_PAIRING,
	BEHAVIOR_LINK,
	BEHAVIOR_READY,
	BEHAVIOR_STILL,
	BEHAVIOR_MOVE,
	BEHAVIOR_PROCESSING,
	BEHAVIOR_OTA,
	BEHAVIOR_HEATED,
	BEHAVIOR_MAINTENANCE,
	BEHAVIOR_TEST,
	BEHAVIOR_INITIALIZING,
	BEHAVIOR_CHARGING,
	BEHAVIOR_CHARGED,
	BEHAVIOR_EXTERNAL,
	BEHAVIOR_LOW,
	BEHAVIOR_FAULT,
	BEHAVIOR_SAFETY,
	BEHAVIOR_IDENTIFY,
	BEHAVIOR_EXIT,
	BEHAVIOR_BUTTON_HOLD,
	BEHAVIOR_MANUAL_EXIT,
	BEHAVIOR_COUNT
};
static const struct led_behavior behaviors[] = {
	[BEHAVIOR_ACK] = {
		.style = LED_STYLE_SHORT, .role = LED_ROLE_INTERACTION, .level = LED_LEVEL_TASK,
		.extent = LED_FINITE, .duration_ms = 800, .ttl_ms = 2000, .priority = LED_PRIORITY_EVENT, .result_rank = 2
	},
	[BEHAVIOR_CANCELLED] = {
		.style = LED_STYLE_SHORT, .role = LED_ROLE_NEUTRAL, .level = LED_LEVEL_TASK,
		.extent = LED_FINITE, .duration_ms = 800, .ttl_ms = 3000, .priority = LED_PRIORITY_EVENT, .result_rank = 2
	},
	[BEHAVIOR_SUCCESS] = {
		.style = LED_STYLE_SUCCESS, .role = LED_ROLE_POSITIVE, .level = LED_LEVEL_NOTICE,
		.extent = LED_FINITE, .duration_ms = 2000, .ttl_ms = 4000, .priority = LED_PRIORITY_EVENT, .result_rank = 1
	},
	[BEHAVIOR_NEGATIVE] = {
		.style = LED_STYLE_DOUBLE, .role = LED_ROLE_NEGATIVE, .level = LED_LEVEL_NOTICE,
		.extent = LED_FINITE, .duration_ms = 1200, .ttl_ms = 6000, .priority = LED_PRIORITY_EVENT
	},
	[BEHAVIOR_PARTIAL] = {
		.style = LED_STYLE_DOUBLE, .role = LED_ROLE_POWER, .level = LED_LEVEL_NOTICE,
		.extent = LED_FINITE, .duration_ms = 1200, .ttl_ms = 6000, .priority = LED_PRIORITY_EVENT
	},
	[BEHAVIOR_PAIRING] = {
		.style = LED_STYLE_LINK, .role = LED_ROLE_LINK, .level = LED_LEVEL_TASK,
		.extent = LED_LOOP, .duration_ms = 2000, .priority = LED_PRIORITY_PAIRING
	},
	[BEHAVIOR_LINK] = {
		.style = LED_STYLE_LINK, .role = LED_ROLE_LINK, .level = LED_LEVEL_TASK,
		.extent = LED_LOOP, .duration_ms = 2000, .priority = LED_PRIORITY_BACKGROUND
	},
	[BEHAVIOR_READY] = {
		.style = LED_STYLE_READY, .role = LED_ROLE_POSITIVE, .level = LED_LEVEL_BACKGROUND,
		.extent = LED_LOOP, .duration_ms = 10000, .priority = LED_PRIORITY_BACKGROUND
	},
	[BEHAVIOR_STILL] = {
		.style = LED_STYLE_SOLID, .role = LED_ROLE_INTERACTION, .level = LED_LEVEL_TASK,
		.extent = LED_HOLD, .priority = LED_PRIORITY_TASK, .task_rank = 2
	},
	[BEHAVIOR_MOVE] = {
		.style = LED_STYLE_MOVE, .role = LED_ROLE_ACTIVITY, .level = LED_LEVEL_TASK,
		.extent = LED_LOOP, .duration_ms = 2000, .priority = LED_PRIORITY_TASK, .task_rank = 2
	},
	[BEHAVIOR_PROCESSING] = {
		.style = LED_STYLE_BREATHE, .role = LED_ROLE_ACTIVITY, .level = LED_LEVEL_TASK,
		.extent = LED_LOOP, .duration_ms = 5000, .priority = LED_PRIORITY_TASK, .task_rank = 2
	},
	[BEHAVIOR_OTA] = {
		.style = LED_STYLE_BREATHE, .role = LED_ROLE_ACTIVITY, .level = LED_LEVEL_TASK,
		.extent = LED_LOOP, .duration_ms = 5000, .priority = LED_PRIORITY_TASK, .task_rank = 1
	},
	[BEHAVIOR_HEATED] = {
		.style = LED_STYLE_BREATHE, .role = LED_ROLE_POWER, .level = LED_LEVEL_TASK,
		.extent = LED_LOOP, .duration_ms = 5000, .priority = LED_PRIORITY_TASK, .task_rank = 3
	},
	[BEHAVIOR_MAINTENANCE] = {
		.style = LED_STYLE_BREATHE, .role = LED_ROLE_ACTIVITY, .level = LED_LEVEL_TASK,
		.extent = LED_LOOP, .duration_ms = 5000, .priority = LED_PRIORITY_TASK, .task_rank = 4
	},
	[BEHAVIOR_TEST] = {
		.style = LED_STYLE_BREATHE, .role = LED_ROLE_ACTIVITY, .level = LED_LEVEL_TASK,
		.extent = LED_LOOP, .duration_ms = 5000, .priority = LED_PRIORITY_TASK, .task_rank = 5
	},
	[BEHAVIOR_INITIALIZING] = {
		.style = LED_STYLE_BREATHE, .role = LED_ROLE_INTERACTION, .level = LED_LEVEL_TASK,
		.extent = LED_LOOP, .duration_ms = 5000, .priority = LED_PRIORITY_TASK, .task_rank = 6
	},
	[BEHAVIOR_CHARGING] = {
		.style = LED_STYLE_BREATHE, .role = LED_ROLE_POWER, .level = LED_LEVEL_TASK,
		.extent = LED_LOOP, .duration_ms = 5000, .priority = LED_PRIORITY_BACKGROUND
	},
	[BEHAVIOR_CHARGED] = {
		.style = LED_STYLE_SOLID, .role = LED_ROLE_POSITIVE, .level = LED_LEVEL_BACKGROUND,
		.extent = LED_HOLD, .priority = LED_PRIORITY_BACKGROUND
	},
	[BEHAVIOR_EXTERNAL] = {
		.style = LED_STYLE_SOLID, .role = LED_ROLE_NEUTRAL, .level = LED_LEVEL_BACKGROUND,
		.extent = LED_HOLD, .priority = LED_PRIORITY_BACKGROUND
	},
	[BEHAVIOR_LOW] = {
		.style = LED_STYLE_LOW, .role = LED_ROLE_POWER, .level = LED_LEVEL_NOTICE,
		.extent = LED_FINITE, .duration_ms = 2000, .priority = LED_PRIORITY_LOW
	},
	[BEHAVIOR_FAULT] = {
		.style = LED_STYLE_WARNING, .role = LED_ROLE_NEGATIVE, .level = LED_LEVEL_NOTICE,
		.extent = LED_LOOP, .duration_ms = 5000, .priority = LED_PRIORITY_FAULT
	},
	[BEHAVIOR_SAFETY] = {
		.style = LED_STYLE_WARNING, .role = LED_ROLE_NEGATIVE, .level = LED_LEVEL_NOTICE,
		.extent = LED_LOOP, .duration_ms = 5000, .priority = LED_PRIORITY_SAFETY
	},
	[BEHAVIOR_IDENTIFY] = {
		.style = LED_STYLE_MOVE, .role = LED_ROLE_INTERACTION, .level = LED_LEVEL_NOTICE,
		.extent = LED_FINITE, .duration_ms = 6000, .ttl_ms = 6000, .priority = LED_PRIORITY_IDENTIFY
	},
	[BEHAVIOR_EXIT] = {
		.style = LED_STYLE_EXIT, .role = LED_ROLE_NEUTRAL, .level = LED_LEVEL_TASK,
		.extent = LED_FINITE, .duration_ms = 800, .priority = LED_PRIORITY_TASK
	},
	[BEHAVIOR_BUTTON_HOLD] = {
		.style = LED_STYLE_BUTTON_HOLD, .role = LED_ROLE_INTERACTION, .level = LED_LEVEL_NOTICE,
		.extent = LED_HOLD, .priority = LED_PRIORITY_TASK
	},
	[BEHAVIOR_MANUAL_EXIT] = {
		.style = LED_STYLE_MANUAL_EXIT, .role = LED_ROLE_INTERACTION, .level = LED_LEVEL_NOTICE,
		.extent = LED_FINITE, .duration_ms = LED_MANUAL_EXIT_VISUAL_MS, .priority = LED_PRIORITY_TASK
	},
};
/* Dense semantic order makes adding any enum without a map entry a build error. */
static const uint8_t semantic_behaviors[] = {
	BEHAVIOR_ACK, /* LED_NONE is rejected before lookup */
	BEHAVIOR_ACK, /* LED_INPUT_ACK */
	BEHAVIOR_ACK, /* LED_ACCEPTED */
	BEHAVIOR_ACK, /* LED_STAGE_ACK */
	BEHAVIOR_CANCELLED,
	BEHAVIOR_SUCCESS,
	BEHAVIOR_NEGATIVE, /* LED_REJECTED */
	BEHAVIOR_NEGATIVE, /* LED_FAILED */
	BEHAVIOR_PARTIAL,
	BEHAVIOR_PARTIAL, /* LED_APPLIED_NOT_SAVED */
	BEHAVIOR_PAIRING,
	BEHAVIOR_LINK, /* LED_RECONNECTING */
	BEHAVIOR_LINK, /* LED_UNPAIRED_IDLE */
	BEHAVIOR_READY,
	BEHAVIOR_STILL, /* LED_WAIT_STILL */
	BEHAVIOR_STILL, /* LED_COLLECT_STILL */
	BEHAVIOR_MOVE, /* LED_WAIT_MOVE */
	BEHAVIOR_MOVE, /* LED_COLLECT_MOVE */
	BEHAVIOR_PROCESSING,
	BEHAVIOR_OTA,
	BEHAVIOR_HEATED,
	BEHAVIOR_MAINTENANCE,
	BEHAVIOR_TEST,
	BEHAVIOR_INITIALIZING,
	BEHAVIOR_CHARGING,
	BEHAVIOR_CHARGED,
	BEHAVIOR_EXTERNAL,
	BEHAVIOR_LOW,
	BEHAVIOR_FAULT, /* LED_SENSOR_MISSING */
	BEHAVIOR_FAULT, /* LED_SENSOR_FAULT */
	BEHAVIOR_FAULT, /* LED_BLOCKING_FAULT */
	BEHAVIOR_SAFETY,
	BEHAVIOR_IDENTIFY,
	BEHAVIOR_EXIT,
	BEHAVIOR_BUTTON_HOLD,
	BEHAVIOR_MANUAL_EXIT,
};
_Static_assert(sizeof(behaviors) / sizeof(behaviors[0]) == BEHAVIOR_COUNT, "LED behavior missing descriptor");
_Static_assert(sizeof(semantic_behaviors) == LED_SEMANTIC_COUNT, "LED semantic missing behavior mapping");

const struct led_behavior *led_behavior_get(enum led_semantic semantic)
{
	if (semantic <= LED_NONE || semantic >= LED_SEMANTIC_COUNT) {
		return NULL;
	}
	return &behaviors[semantic_behaviors[semantic]];
}

#ifdef CONFIG_LED_DEBUG
const char *led_behavior_name(enum led_semantic semantic)
{
	static const char *const names[] = {
		"none", "input_ack", "accepted", "stage_ack", "cancelled", "success", "rejected", "failed",
		"partial", "applied_not_saved", "pairing", "reconnecting", "unpaired_idle", "ready", "wait_still",
		"collect_still", "wait_move", "collect_move", "processing", "ota_active", "heated_active", "maintenance",
		"test_active", "initializing", "charging", "charged", "external_power_unknown", "low_battery",
		"sensor_missing", "sensor_fault", "blocking_fault", "safety_fault", "identify", "exit_pending",
		"button_hold", "manual_exit"
	};
	_Static_assert(sizeof(names) / sizeof(names[0]) == LED_SEMANTIC_COUNT, "LED semantic missing debug name");
	return semantic > LED_NONE && semantic < LED_SEMANTIC_COUNT ? names[semantic] : "none";
}
const char *led_style_name(enum led_style style)
{
	static const char *const names[] = {
		"short", "success", "double", "link", "ready", "solid", "move", "breathe", "low", "warning", "exit",
		"manual_exit", "button_hold"
	};
	_Static_assert(sizeof(names) / sizeof(names[0]) == LED_STYLE_COUNT, "LED style missing debug name");
	return style >= LED_STYLE_SHORT && style < LED_STYLE_COUNT ? names[style] : "invalid";
}
const char *led_role_name(enum led_role role)
{
	static const char *const names[LED_ROLE_COUNT] = {
		"link", "positive", "negative", "power", "neutral", "interaction", "activity"
	};
	return role >= LED_ROLE_LINK && role < LED_ROLE_COUNT ? names[role] : "invalid";
}
const char *led_extent_name(enum led_extent extent)
{
	static const char *const names[] = {"finite", "loop", "hold"};
	return extent >= LED_FINITE && extent <= LED_HOLD ? names[extent] : "invalid";
}
const char *led_level_name(enum led_level level)
{
	switch (level) {
	case LED_LEVEL_BACKGROUND:
		return "background";
	case LED_LEVEL_TASK:
		return "task";
	case LED_LEVEL_NOTICE:
		return "notice";
	default:
		return "invalid";
	}
}
#endif
