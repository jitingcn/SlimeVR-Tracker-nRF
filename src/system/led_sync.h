#ifndef SLIMENRF_LED_SYNC_H
#define SLIMENRF_LED_SYNC_H

#include <stdbool.h>
#include <stdint.h>

#define LED_SYNC_HZ 32768U
#define LED_SYNC_ACTIVE_PERIOD (10U * LED_SYNC_HZ)
#define LED_SYNC_ACTIVE_OFF 317850U /* ceil(9.7 * 32768) */

/* Raw receiver low32 time deliberately jumps at wrap; no private epoch. */
static inline uint32_t led_sync_phase(uint32_t ticks, uint32_t period)
{
	return ticks % period;
}

/* Never sleep past the common raw32 wrap discontinuity. */
static inline uint32_t led_sync_until(uint32_t ticks, uint32_t distance)
{
	uint32_t to_wrap = UINT32_MAX - ticks;
	return distance > to_wrap ? to_wrap + 1U : distance;
}

static inline int led_sync_pulse(uint32_t phase)
{
	int value = (int)((phase * 1000U) / (5U * LED_SYNC_HZ));
	value = value > 500 ? 1000 - value : value;
	if (value < 200) {
		return value * 30;
	} else if (value < 300) {
		return (value - 200) * 20 + 6000;
	} else if (value < 400) {
		return (value - 300) * 15 + 8000;
	}
	return (value - 400) * 5 + 9500;
}

struct led_sync_start {
	uint32_t entered;
	bool eligible;
	bool joined;
	bool started;
};

/* First illuminate only a full window after at least 9.7 local seconds.
 * Seeing the preceding off segment arms that window. Source changes never
 * reset the local holdoff; after joining they cannot blank subsequent cycles.
 */
static inline bool led_sync_active(struct led_sync_start *start,
	uint32_t local, uint32_t phase)
{
	if (!start->eligible && local - start->entered >= LED_SYNC_ACTIVE_OFF) {
		start->eligible = true;
	}
	/* Arm on the off segment whose end is not before the holdoff. This
	 * tolerates a scheduler waking just after the nominal boundary. */
	if (phase < LED_SYNC_ACTIVE_OFF) {
		start->joined = start->eligible ||
			local - start->entered + LED_SYNC_ACTIVE_OFF - phase >= LED_SYNC_ACTIVE_OFF;
	} else if (!start->eligible) {
		start->joined = false;
	}
	bool on = start->eligible && start->joined && phase >= LED_SYNC_ACTIVE_OFF;
	start->started |= on;
	return on;
}

#endif
