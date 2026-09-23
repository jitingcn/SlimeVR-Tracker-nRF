#ifndef LED_STRIP_FADE_H
#define LED_STRIP_FADE_H

#include <stdint.h>

/*
 * Smooth fading on an 8-bit LED strip.
 *
 * The LED code dims a pattern twice before quantizing it, once for the pattern
 * brightness and once for CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT, and the strip only
 * has 8 bits per channel. At the configured global brightness the range left
 * for a fade is therefore small (10 % of full brightness leaves about 26 levels
 * for the brightest channel and a handful for the others), while the breathing
 * and power-off ramps spend seconds inside that range: the fade visibly steps.
 *
 * Keep the fraction of a level that does not fit into a frame and carry it over
 * to the next one. The patterns that fade already refresh every 5 ms, far above
 * the rate the eye resolves, so consecutive frames are averaged by the eye and
 * the ramp becomes continuous while the average stays exactly at the requested
 * brightness. A pattern that changes its brightness on a tick slower than the
 * eye response would instead show the carry as flicker, so keep fading patterns
 * refreshing at least every ~10 ms.
 */
struct led_strip_fade {
	/* Fraction of a level left over from the previous frame, per channel. */
	int32_t error[3];
};

static inline void led_strip_fade_reset(struct led_strip_fade *fade)
{
	fade->error[0] = 0;
	fade->error[1] = 0;
	fade->error[2] = 0;
}

/*
 * Convert one channel of a frame (both factors in parts per ten-thousand, as
 * used by the LED code) into the 8-bit level for this frame.
 */
static inline uint8_t led_strip_fade_next(struct led_strip_fade *fade, int channel_pptt,
					  int value_pptt, int index)
{
	int32_t ideal = ((int32_t)channel_pptt * value_pptt + 5000) / 10000 * 255;
	int32_t accumulated = fade->error[index] + ideal;
	int32_t level = (accumulated + 5000) / 10000;

	if (level > 255) {
		level = 255;
	} else if (level < 0) {
		level = 0;
	}

	fade->error[index] = accumulated - level * 10000;

	return (uint8_t)level;
}

#endif /* LED_STRIP_FADE_H */
