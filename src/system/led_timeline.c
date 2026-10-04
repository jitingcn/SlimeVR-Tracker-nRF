#include "led_internal.h"
#include <limits.h>

/* Explicit immutable envelopes, left-closed/right-open. Driver duration is not
 * appended to segments; worker evaluates current wall clock after every write. */
enum segment_kind { SEGMENT_ON, SEGMENT_OFF, SEGMENT_UP, SEGMENT_DOWN, SEGMENT_LINEAR_DOWN };
struct segment {
	uint16_t duration_ms;
	uint8_t kind;
};
struct timeline {
	const struct segment *segments;
	uint16_t period_ms;
	uint8_t count;
	bool hold;
};
static const struct segment short_ack[] = {{200, SEGMENT_ON}, {600, SEGMENT_OFF}};
static const struct segment success[] = {
	{200, SEGMENT_ON}, {200, SEGMENT_OFF}, {200, SEGMENT_ON}, {200, SEGMENT_OFF},
	{200, SEGMENT_ON}, {200, SEGMENT_OFF}, {200, SEGMENT_ON}, {600, SEGMENT_OFF}
};
static const struct segment double_ack[] = {{200, SEGMENT_ON}, {200, SEGMENT_OFF}, {200, SEGMENT_ON}, {600, SEGMENT_OFF}};
static const struct segment link[] = {{200, SEGMENT_ON}, {1800, SEGMENT_OFF}};
static const struct segment ready[] = {{200, SEGMENT_ON}, {9800, SEGMENT_OFF}};
static const struct segment move[] = {{200, SEGMENT_ON}, {200, SEGMENT_OFF}, {200, SEGMENT_ON}, {1400, SEGMENT_OFF}};
static const struct segment breathe[] = {{2000, SEGMENT_UP}, {2000, SEGMENT_DOWN}, {1000, SEGMENT_OFF}};
static const struct segment breathe_gpio[] = {{4000, SEGMENT_ON}, {1000, SEGMENT_OFF}};
static const struct segment low[] = {{100, SEGMENT_ON}, {150, SEGMENT_OFF}, {100, SEGMENT_ON}, {1650, SEGMENT_OFF}};
static const struct segment warning[] = {{800, SEGMENT_ON}, {200, SEGMENT_OFF}, {800, SEGMENT_ON}, {3200, SEGMENT_OFF}};
static const struct segment exit_ramp[] = {{600, SEGMENT_DOWN}, {200, SEGMENT_OFF}};
static const struct segment exit_gpio[] = {{600, SEGMENT_ON}, {200, SEGMENT_OFF}};
static const struct segment manual_exit_ramp[] = {{250, SEGMENT_OFF}, {1000, SEGMENT_LINEAR_DOWN}, {250, SEGMENT_OFF}};
static const struct segment manual_exit_gpio[] = {{250, SEGMENT_OFF}, {1000, SEGMENT_ON}, {250, SEGMENT_OFF}};
static const struct timeline timelines[] = {
	[LED_STYLE_SHORT] = {short_ack, 800, 2, false},
	[LED_STYLE_SUCCESS] = {success, 2000, 8, false},
	[LED_STYLE_DOUBLE] = {double_ack, 1200, 4, false},
	[LED_STYLE_LINK] = {link, 2000, 2, false},
	[LED_STYLE_READY] = {ready, 10000, 2, false},
	[LED_STYLE_SOLID] = {.hold = true},
	[LED_STYLE_MOVE] = {move, 2000, 4, false},
	[LED_STYLE_BREATHE] = {breathe, 5000, 3, false},
	[LED_STYLE_LOW] = {low, 2000, 4, false},
	[LED_STYLE_WARNING] = {warning, 5000, 4, false},
	[LED_STYLE_EXIT] = {exit_ramp, 800, 2, false},
	[LED_STYLE_MANUAL_EXIT] = {manual_exit_ramp, LED_MANUAL_EXIT_VISUAL_MS, 3, false},
	[LED_STYLE_BUTTON_HOLD] = {.hold = true},
};
_Static_assert(sizeof(timelines) / sizeof(timelines[0]) == LED_STYLE_COUNT, "LED style missing timeline");

/* Ordinary EXIT keeps its smoothstep easing; BREATHE uses the historical
 * piecewise electrical shape below. Neither is an optical gamma claim. */
static uint16_t ramp_value(uint32_t elapsed_ms, uint32_t duration_ms)
{
	uint64_t numerator = (uint64_t)elapsed_ms * elapsed_ms * (3U * duration_ms - 2U * elapsed_ms) * 10000U;
	uint64_t denominator = (uint64_t)duration_ms * duration_ms * duration_ms;
	return numerator / denominator;
}

/* Historical breathing anchors (normalized x,y): (0,0), (.4,.6),
 * (.6,.8), (.8,.95), (1,1). Rescale to the existing 2s half-ramp;
 * DOWN evaluates this same function at remaining time, never 1-f(elapsed). */
static uint16_t breathe_value(uint32_t elapsed_ms, uint32_t duration_ms)
{
	uint32_t value = elapsed_ms * 10000U / duration_ms;
	if (value < 4000) {
		return value * 3U / 2U;
	}
	if (value < 6000) {
		return value + 2000U;
	}
	if (value < 8000) {
		return (value - 6000U) * 3U / 4U + 8000U;
	}
	return (value - 8000U) / 4U + 9500U;
}

struct led_envelope led_timeline(enum led_style style, uint32_t elapsed_ms, bool gpio, bool finite)
{
	struct led_envelope out = {.next_ms = UINT32_MAX};
	if (style < LED_STYLE_SHORT || style >= LED_STYLE_COUNT) {
		out.complete = true;
		return out;
	}
	if (style == LED_STYLE_BUTTON_HOLD) {
		if (elapsed_ms < LED_BUTTON_HOLD_MS) {
			out.value_pptt = 10000;
			out.next_ms = LED_BUTTON_HOLD_MS - elapsed_ms;
			return out;
		}
		uint32_t exit_age = elapsed_ms - LED_BUTTON_HOLD_MS;
		if (exit_age < LED_MANUAL_EXIT_VISUAL_MS) {
			return led_timeline(LED_STYLE_MANUAL_EXIT, exit_age, gpio, true);
		}
		uint32_t phase = (exit_age - LED_MANUAL_EXIT_VISUAL_MS) % 1000U;
		out.value_pptt = phase < 500 ? 10000 : 0;
		out.next_ms = phase < 500 ? 500 - phase : 1000 - phase;
		return out;
	}
	const struct timeline *line = &timelines[style];
	if (line->hold) {
		out.value_pptt = 10000;
		return out;
	}
	if ((finite || style == LED_STYLE_MANUAL_EXIT) && elapsed_ms >= line->period_ms) {
		out.complete = true;
		return out;
	}
	uint32_t phase_ms = elapsed_ms % line->period_ms;
	const struct segment *segments = line->segments;
	uint8_t count = line->count;
	if (gpio) {
		if (style == LED_STYLE_BREATHE) {
			segments = breathe_gpio;
			count = 2;
		}
		if (style == LED_STYLE_EXIT) {
			segments = exit_gpio;
			count = 2;
		}
		if (style == LED_STYLE_MANUAL_EXIT) {
			segments = manual_exit_gpio;
			count = 3;
		}
	}
	for (uint8_t i = 0; i < count; ++i) {
		const struct segment *segment = &segments[i];
		if (phase_ms >= segment->duration_ms) {
			phase_ms -= segment->duration_ms;
			continue;
		}
		out.next_ms = segment->duration_ms - phase_ms;
		switch (segment->kind) {
		case SEGMENT_ON:
			out.value_pptt = 10000;
			break;
		case SEGMENT_OFF:
			break;
		case SEGMENT_UP:
			out.value_pptt = breathe_value(phase_ms, segment->duration_ms);
			out.fading = true;
			break;
		case SEGMENT_DOWN:
			out.value_pptt = style == LED_STYLE_BREATHE
				? breathe_value(segment->duration_ms - phase_ms, segment->duration_ms)
				: ramp_value(segment->duration_ms - phase_ms, segment->duration_ms);
			out.fading = true;
			break;
		case SEGMENT_LINEAR_DOWN:
			out.value_pptt = (segment->duration_ms - phase_ms) * 10000U / segment->duration_ms;
			out.fading = true;
			break;
		}
		if (out.fading && out.next_ms > LED_FRAME_MS) {
			out.next_ms = LED_FRAME_MS;
		}
		return out;
	}
	out.complete = true;
	return out;
}

struct led_envelope led_timeline_behavior(
	const struct led_behavior *behavior, uint32_t elapsed_ms, bool gpio, bool finite
)
{
	if (!behavior || (finite && elapsed_ms >= behavior->duration_ms)) {
		return (struct led_envelope){.next_ms = UINT32_MAX, .complete = true};
	}
	/* A semantic wall window may span several identical cycles (identify).
	 * Completion belongs to the descriptor, never to a special glyph. */
	return led_timeline(behavior->style, elapsed_ms, gpio, false);
}

struct led_envelope led_timeline_button_count(uint32_t count, uint32_t elapsed_ms)
{
	if (!count || count > LED_BUTTON_GROUP_MAX || elapsed_ms >= count * LED_BUTTON_CYCLE_MS) {
		return (struct led_envelope){.next_ms = UINT32_MAX, .complete = true};
	}
	/* Exact N-count confirmations stay at 2Hz, independent of finite ACK glyphs. */
	uint32_t phase_ms = elapsed_ms % LED_BUTTON_CYCLE_MS;
	return (struct led_envelope){
		.value_pptt = phase_ms < 80 ? 10000 : 0,
		.next_ms = phase_ms < 80 ? 80 - phase_ms : LED_BUTTON_CYCLE_MS - phase_ms
	};
}
