/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */
/* Host arithmetic proof; no optical/perceptual smoothness claim. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "../../../src/system/led_strip_fade.h"

static long double magnitude(long double value) { return value < 0 ? -value : value; }

static void scaling_precision_and_boundaries(void)
{
	static const uint16_t factors[][4] = {
		{10000,10000,10000,10000}, {1000,10000,6000,10000},
		{2000,10000,6000,10000}, {3333,7777,6000,4321},
		{1,10000,2000,1}, {0,10000,10000,10000}
	};
	static const uint32_t directions[] = {0, LED_DIRECTION_ONE, LED_DIRECTION_ONE / 3, 1234567};
	for (unsigned f = 0; f < sizeof(factors) / sizeof(factors[0]); f++) {
		uint64_t brightness = led_brightness_q32(factors[f][0], factors[f][1], factors[f][2], factors[f][3]);
		long double duty = 1;
		for (unsigned i = 0; i < 4; i++) { duty *= factors[f][i] / 10000.0L; }
		assert((long double)brightness / (1ULL << 32) <= duty + 1e-18L);
		assert(duty - (long double)brightness / (1ULL << 32) < 1.0L / (1ULL << 32) + 1e-18L);
		for (unsigned d = 0; d < sizeof(directions) / sizeof(directions[0]); d++) {
			long double ideal = duty * directions[d] / LED_DIRECTION_ONE * 255;
			uint8_t byte = led_strip_byte(brightness, directions[d]);
			assert(magnitude(byte - ideal) <= 0.5L + 255.0L / (1ULL << 32));
		}
	}
	assert(led_strip_byte(1ULL << 32, LED_DIRECTION_ONE) == 255);
}

static void physical_rounding_boundaries(void)
{
	/* Values immediately around the first physical half-LSB threshold.
	 * Sub-half targets remain truly black, not a long-run pulse average. */
	uint64_t below_half = ((1ULL << 31) - 1) / 255;
	uint64_t above_half = ((1ULL << 31) + 254) / 255;
	assert(led_strip_byte(below_half, LED_DIRECTION_ONE) == 0);
	assert(led_strip_byte(above_half, LED_DIRECTION_ONE) == 1);
	assert(led_strip_byte(0, LED_DIRECTION_ONE) == 0);
	assert(led_strip_byte(1ULL << 32, 0) == 0);
	uint8_t previous = 255;
	for (uint32_t step = 0; step <= 10000; step++) {
		uint64_t duty = (uint64_t)(10000 - step) * (1ULL << 32) / 10000;
		uint8_t byte = led_strip_byte(duty, LED_DIRECTION_ONE);
		assert(byte <= previous);
		long double ideal = (10000 - step) * 255.0L / 10000;
		assert(magnitude(byte - ideal) <= 0.5L + 255.0L / (1ULL << 32));
		previous = byte;
	}
	assert(previous == 0);
}

int main(void)
{
	scaling_precision_and_boundaries();
	physical_rounding_boundaries();
	puts("LED physical-byte precision/rounding/black/descending-boundary regressions passed");
	return 0;
}
