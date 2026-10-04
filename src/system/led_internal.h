#ifndef SLIMENRF_LED_INTERNAL_H
#define SLIMENRF_LED_INTERNAL_H

#include "led.h"
#include <stddef.h>

static inline bool led_owner_valid(enum led_owner owner)
{
	return owner >= LED_OWNER_IMU && owner < LED_OWNER_COUNT;
}

/* Lower numbers win; preview shares IDENTIFY's level but is selected after it. */
enum led_priority {
	LED_PRIORITY_SHUTDOWN = 0,
	LED_PRIORITY_SAFETY = 1,
	LED_PRIORITY_FAULT = 2,
	LED_PRIORITY_LOW = 3,
	LED_PRIORITY_EVENT = 4,
	LED_PRIORITY_IDENTIFY = 5,
	LED_PRIORITY_TASK = 6,
	LED_PRIORITY_PAIRING = 7,
	LED_PRIORITY_BACKGROUND = 8
};

/* One readonly source binds production semantics and optional debug names. */
enum led_style {
	LED_STYLE_SHORT,
	LED_STYLE_SUCCESS,
	LED_STYLE_DOUBLE,
	LED_STYLE_LINK,
	LED_STYLE_READY,
	LED_STYLE_SOLID,
	LED_STYLE_MOVE,
	LED_STYLE_BREATHE,
	LED_STYLE_LOW,
	LED_STYLE_WARNING,
	LED_STYLE_EXIT,
	LED_STYLE_MANUAL_EXIT,
	LED_STYLE_BUTTON_HOLD,
	LED_STYLE_COUNT
};
enum led_role {
	LED_ROLE_LINK,
	LED_ROLE_POSITIVE,
	LED_ROLE_NEGATIVE,
	LED_ROLE_POWER,
	LED_ROLE_NEUTRAL,
	LED_ROLE_INTERACTION,
	LED_ROLE_ACTIVITY,
	LED_ROLE_COUNT
};
/* Fade samples use the monotonic wall-clock grid, not producer wake times. */
#define LED_FRAME_MS 5U
#define LED_MANUAL_EXIT_VISUAL_MS 1500U
/* Fresh finite feedback needs a whole terminal-length black pause, including
 * replacement of a partly visible acknowledgement by an actual terminal. */
#define LED_FEEDBACK_START_GAP_MS 600U
enum led_level { LED_LEVEL_BACKGROUND = 2000, LED_LEVEL_TASK = 6000, LED_LEVEL_NOTICE = 10000 };
enum led_extent { LED_FINITE, LED_LOOP, LED_HOLD };
struct led_behavior {
	uint16_t duration_ms;
	uint16_t ttl_ms;
	uint16_t level;
	uint8_t style;
	uint8_t role;
	uint8_t extent;
	uint8_t priority;
	uint8_t result_rank;
	uint8_t task_rank;
};
const struct led_behavior *led_behavior_get(enum led_semantic semantic);
#ifdef CONFIG_LED_DEBUG
enum led_time_source { LED_TIME_LOCAL, LED_TIME_NETWORK_FRESH, LED_TIME_NETWORK_HELD };
const char *led_behavior_name(enum led_semantic semantic);
const char *led_style_name(enum led_style style);
const char *led_role_name(enum led_role role);
const char *led_extent_name(enum led_extent extent);
const char *led_level_name(enum led_level level);
#endif
/* Timeline envelope is 0..10000; next_ms is a relative edge/frame deadline.
 * Explicit GPIO replacements are selected here, not thresholded ramps. */
struct led_envelope {
	uint16_t value_pptt;
	uint32_t next_ms;
	bool complete;
	bool fading;
};
struct led_envelope led_timeline(enum led_style style, uint32_t elapsed_ms, bool gpio, bool finite);
struct led_envelope led_timeline_button_count(uint32_t count, uint32_t elapsed_ms);
struct led_envelope led_timeline_behavior(
	const struct led_behavior *behavior, uint32_t elapsed_ms, bool gpio, bool finite
);

struct led_selection {
	enum led_semantic semantic;
	enum led_owner owner;
	uint32_t origin_ms;
	uint32_t identity;
	uint32_t next_ms;
	uint32_t button_count;
	uint32_t button_generation; /* physical-hold provenance, zero otherwise */
	uint8_t priority;
	bool finite;
	bool network;
};
#ifdef CONFIG_LED_DEBUG
struct led_policy_view {
	struct led_selection winner;
	uint32_t low_due_ms;
	bool low;
	bool low_active;
	uint32_t event_remaining_ms[LED_OWNER_COUNT];
	bool quiesced;
	bool ota_active;
	bool heated_active;
	bool ready;
	uint32_t black_since_ms;
	enum led_time_source time_source;
};
void led_policy_snapshot(struct led_policy_view *view);
#endif

/* Hardware API: implementation owns explicit board mapping, driver/PWM/strip
 * selection, current limiting, unchanged-frame suppression and rail lifetime.
 * Only the existing LED worker may call init/write/off. */
enum led_capability {
	LED_CAP_NO_LED,
	LED_CAP_MONO_GPIO,
	LED_CAP_MONO_PWM,
	LED_CAP_DUAL_DISCRETE,
	LED_CAP_RG_MIX,
	LED_CAP_TRI_DISCRETE,
	LED_CAP_RGB_PWM,
	LED_CAP_RGB_PIXEL
};
enum led_physical_color {
	LED_PHYSICAL_RED,
	LED_PHYSICAL_GREEN,
	LED_PHYSICAL_BLUE,
	LED_PHYSICAL_AMBER,
	LED_PHYSICAL_WHITE
};
struct led_hardware_info {
	enum led_capability capability;
	uint16_t global_limit_pptt;
	uint16_t board_limit_pptt;
	uint16_t visible_min_pptt;
	bool gpio;
	bool powered;
	int last_error;
};
void led_hw_init(void);
void led_hw_off(void); /* always attempts power gate even after black-write error */
/* Optional pixel fade sample. Source is an opaque packed selection key;
 * origin distinguishes restarts. Slot equality suppresses duplicate producer
 * wakes, including across uint32_t wrap; skipped slots never accrue debt. */
struct led_fade_sample {
	uint64_t source;
	uint32_t origin_ms;
	uint32_t slot;
};
/* NULL keeps stateless weakest-channel mixture quantization, nearest pure/equal
 * components and unchanged-frame suppression. BREATHE and the physical-button
 * linear DOWN segments alone retain bounded carry after successful latches.
 * Logical nonzero keeps the rail on even if the physical frame is black. */
bool led_hw_write(
	enum led_role role,
	uint16_t level_pptt,
	uint16_t value_pptt,
	int color_override,
	const struct led_fade_sample *fade
);
void led_hw_info(struct led_hardware_info *info);
bool led_hw_color_supported(enum led_physical_color color);

#ifdef CONFIG_LED_DEBUG
/* Core owns preview state under the same lock as production admission. Debug
 * parser only validates names/tokens and formats these readonly snapshots. */
enum led_preview_result {
	LED_PREVIEW_ADMITTED,
	LED_PREVIEW_UNSUPPORTED,
	LED_PREVIEW_BUSY,
	LED_PREVIEW_INVALID,
	LED_PREVIEW_SHUTDOWN
};
enum led_preview_status {
	LED_PREVIEW_NONE,
	LED_PREVIEW_ACTIVE,
	LED_PREVIEW_COMPLETED,
	LED_PREVIEW_STOPPED,
	LED_PREVIEW_PREEMPTED,
	LED_PREVIEW_EXPIRED
};
struct led_preview_record {
	uint32_t generation;
	uint32_t console_session;
	enum led_semantic semantic;
	int color_override;
	uint32_t origin_ms;
	uint32_t expires_ms;
	enum led_preview_status status;
	enum led_preview_result result;
	enum led_semantic blocker;
	bool muted;
};
/* Parser gives 0 duration for finite (core derives envelope + start gap + 500ms);
 * loop/hold/color give checked window 250..30000 ms. color=-1 for semantic. */
enum led_preview_result led_preview_start(
	uint32_t console_session,
	enum led_semantic semantic,
	int color_override,
	uint32_t duration_ms,
	struct led_preview_record *record
);
void led_preview_stop(void);
void led_preview_session_start(uint32_t session);
void led_preview_disconnect(uint32_t session);
void led_preview_snapshot(struct led_preview_record *record);
#endif

#endif
