/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Host test for the strip fade carry used by src/system/led.c.
 *
 * The behaviour that matters is perceptual: the strip only has 8 bits per
 * channel, so at the configured global brightness a fade has very few levels,
 * and the fade looks smooth only if the brightness the eye integrates over a
 * few frames stays on the requested ramp. Every check below therefore compares
 * the average of the emitted frames with the average of the requested levels
 * over the same window, never a single frame.
 */

#include <stdio.h>
#include <stdlib.h>

#include "../../../src/system/led_strip_fade.h"

/* Host builds have no Zephyr autoconf, so mirror the global brightness the SK
 * CheeseCake factory defconfig selects (10 %) under its production name. */
#define CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT 1000
#define PATTERN_REFRESH_MS 5        /* the fading patterns refresh every 5 ms */
#define EYE_WINDOW 40               /* 200 ms of frames */
#define HALF_LEVEL 5000             /* half of an 8-bit level in 1/10000 units */

static int failures;

static void check(int condition, const char *what, const char *detail, int expected, int actual)
{
	if (!condition) {
		printf("FAIL: %s (%s: expected %d, got %d)\n", what, detail, expected, actual);
		failures++;
	}
}

/* Requested level in the helper's units (1/10000 of an 8-bit level). */
static int32_t requested_x10000(int channel_pptt, int value_pptt)
{
	return ((int32_t)channel_pptt * value_pptt + 5000) / 10000 * 255;
}

/* Dimming as led_pin_set() does it, so the test follows the same rounding. */
static int dimmed_value_pptt(int value_pptt, int brightness_pptt)
{
	int dimmed;

	if (brightness_pptt > 10000) {
		brightness_pptt = 10000;
	} else if (brightness_pptt < 0) {
		brightness_pptt = 0;
	}

	dimmed = value_pptt * brightness_pptt / 10000;

	return dimmed * CONFIG_LED_GLOBAL_BRIGHTNESS_PPTT / 10000;
}

static void compare_windows(int32_t *levels, int32_t *requests)
{
	int64_t level_sum = 0;
	int64_t request_sum = 0;

	for (int i = 0; i < EYE_WINDOW; i++) {
		level_sum += levels[i];
		request_sum += requests[i];
	}

	int32_t level_mean = (int32_t)(level_sum * 10000 / EYE_WINDOW);
	int32_t request_mean = (int32_t)(request_sum / EYE_WINDOW);

	check(labs((long)(level_mean - request_mean)) <= HALF_LEVEL,
	      "integrated brightness follows the requested ramp", "delta", 0,
	      (int)(level_mean - request_mean));
}

static void test_exact_values_do_not_flicker(void)
{
	struct led_strip_fade fade = {0};

	for (int frame = 0; frame < 1000; frame++) {
		int full = led_strip_fade_next(&fade, 10000, 10000, 0);
		int black = led_strip_fade_next(&fade, 0, 10000, 1);
		int dark = led_strip_fade_next(&fade, 10000, 0, 2);

		check(full == 255, "full scale stays 255", "frame", 255, full);
		check(black == 0, "black channel stays 0", "frame", 0, black);
		check(dark == 0, "zero brightness stays 0", "frame", 0, dark);
	}

	check(fade.error[0] == 0 && fade.error[1] == 0 && fade.error[2] == 0,
	      "exact levels leave no carry behind", "carry", 0, (int)fade.error[0]);
}

static void test_average_matches_request(void)
{
	static const int channels[] = {10000, 8000, 5294, 3137, 2000, 1000, 1};
	static const int values[] = {1000, 5000, 9999, 10000};
	const int frames = 200000;

	for (size_t c = 0; c < sizeof(channels) / sizeof(channels[0]); c++) {
		for (size_t v = 0; v < sizeof(values) / sizeof(values[0]); v++) {
			struct led_strip_fade fade = {0};
			int64_t total = 0;
			int32_t requested = requested_x10000(channels[c], values[v]);

			for (int frame = 0; frame < frames; frame++) {
				total += led_strip_fade_next(&fade, channels[c], values[v], 0);
			}

			int32_t mean_x10000 = (int32_t)(total * 10000 / frames);

			check(labs((long)(mean_x10000 - requested)) <= HALF_LEVEL,
			      "average level matches the requested level", "channel/value",
			      (int)requested, (int)mean_x10000);
		}
	}
}

/*
 * The power-off fade steps the pattern brightness down every 5 ms, which is what
 * a user sees as the transition. Track the frames the eye would integrate and
 * make sure their brightness stays on the requested ramp, and that the emitted
 * level never jumps by more than one step within a frame.
 */
static void test_power_off_ramp_is_smooth(void)
{
	struct led_strip_fade fade = {0};
	int32_t levels[EYE_WINDOW] = {0};
	int32_t requests[EYE_WINDOW] = {0};
	int position = 0;
	int filled = 0;
	int last_level = -1;
	int32_t last_request = -1;

	for (int state = 0; state <= 202; state++) {
		int brightness_pptt = state == 0 ? 10000 : (202 - state) * 50;
		int value_pptt = dimmed_value_pptt(10000, brightness_pptt);
		int level = led_strip_fade_next(&fade, 10000, value_pptt, 0);
		int32_t request = requested_x10000(10000, value_pptt);

		if (last_level >= 0) {
			/* The ramp itself moves 1.27 levels per step here, so the level
			 * may follow by that much plus at most one dither step. */
			int32_t moved = labs((long)(request - last_request));
			int allowed = (int)((moved + 9999) / 10000) + 1;

			check(abs(level - last_level) <= allowed,
			      "level follows the ramp without extra steps", "state", allowed,
			      abs(level - last_level));
		}
		last_level = level;
		last_request = request;

		levels[position] = level;
		requests[position] = request;
		position = (position + 1) % EYE_WINDOW;
		if (filled < EYE_WINDOW) {
			filled++;
			continue;
		}

		compare_windows(levels, requests);
	}
}

/* A rising ramp must never go backwards once the eye has integrated it. */
static void test_rising_ramp_advances(void)
{
	struct led_strip_fade fade = {0};
	int32_t levels[EYE_WINDOW] = {0};
	int32_t requests[EYE_WINDOW] = {0};
	int position = 0;
	int filled = 0;
	int32_t previous_mean = -1;

	for (int step = 0; step <= 500; step++) {
		int value_pptt = dimmed_value_pptt(step * 20, 10000);
		int level = led_strip_fade_next(&fade, 8000, value_pptt, 0);

		levels[position] = level;
		requests[position] = requested_x10000(8000, value_pptt);
		position = (position + 1) % EYE_WINDOW;
		if (filled < EYE_WINDOW) {
			filled++;
			continue;
		}

		compare_windows(levels, requests);

		int64_t sum = 0;

		for (int i = 0; i < EYE_WINDOW; i++) {
			sum += levels[i];
		}

		int32_t mean_x10000 = (int32_t)(sum * 10000 / EYE_WINDOW);

		if (previous_mean >= 0) {
			check(mean_x10000 >= previous_mean, "rising ramp is non-decreasing", "step",
			      (int)previous_mean, (int)mean_x10000);
		}
		previous_mean = mean_x10000;
	}
}

int main(void)
{
	printf("fade carry, %d ms pattern refresh, %d ms integration window\n", PATTERN_REFRESH_MS,
	       EYE_WINDOW * PATTERN_REFRESH_MS);

	test_exact_values_do_not_flicker();
	test_average_matches_request();
	test_power_off_ramp_is_smooth();
	test_rising_ramp_advances();

	if (failures != 0) {
		printf("%d check(s) failed\n", failures);
		return 1;
	}

	printf("all checks passed\n");
	return 0;
}
