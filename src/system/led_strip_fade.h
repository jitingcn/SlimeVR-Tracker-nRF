#ifndef LED_STRIP_FADE_H
#define LED_STRIP_FADE_H

#include <stdint.h>

#define LED_DIRECTION_ONE (1U << 24)

/* Electrical duty in Q32. All four PPTT factors remain fractional: there is
 * no intermediate integer-PPTT brightness. Split before multiplication so the
 * largest intermediate is below 2^63, including all-full-scale inputs. */
static inline uint64_t led_brightness_q32(uint16_t global, uint16_t board, uint16_t level, uint16_t envelope)
{
	uint64_t product = (uint64_t)global * board * level * envelope;
	uint64_t scaled = (product / 1000000000U) * (1ULL << 32)
		+ (product % 1000000000U) * (1ULL << 32) / 1000000000U;
	return scaled / 10000000U;
}

/* Round once at the physical byte boundary. The Q32 duty * Q24 direction *
 * 255 product plus a half-byte fits uint64_t even at full scale. Stateless
 * callers retain nearest-byte rounding, including stable sub-LSB black. */
static inline uint8_t led_strip_byte(uint64_t brightness_q32, uint32_t direction_q24)
{
	return (brightness_q32 * direction_q24 * 255U + (1ULL << 55)) >> 56;
}

/* Pixel BREATHE and physical-button DOWN use temporal carry, preserving Q16
 * targets without floating point or saturation debt; the adapter commits this
 * small candidate only after a successful physical/cache latch. */
#define LED_FADE_ONE (1U << 16)
struct led_strip_fade {
	int32_t error[3];
};

static inline void led_strip_fade_targets(
	uint64_t brightness_q32, const uint32_t direction_q24[3],
	uint32_t maximum, uint32_t maximum_total, uint32_t target[3]
)
{
	uint32_t sum = 0, largest = 0;
	for (unsigned channel = 0; channel < 3; ++channel) {
		target[channel] = brightness_q32 * direction_q24[channel] * 255U >> 40;
		sum += target[channel];
		if (target[channel] > largest) {
			largest = target[channel];
		}
	}
	uint32_t limit = maximum * LED_FADE_ONE;
	uint32_t total_limit = maximum_total * LED_FADE_ONE;
	uint32_t numerator = 1, denominator = 1;
	if (largest > limit) {
		numerator = limit;
		denominator = largest;
	}
	if ((uint64_t)sum * numerator > (uint64_t)total_limit * denominator) {
		numerator = total_limit;
		denominator = sum;
	}
	if (numerator != denominator) {
		/* Project the fractional color onto the integer-cap simplex before
		 * quantization. Impossible brightness never enters the carry. */
		for (unsigned channel = 0; channel < 3; ++channel) {
			target[channel] = (uint64_t)target[channel] * numerator / denominator;
		}
	}
}

static inline void led_strip_fade_frame(
	const struct led_strip_fade *current, struct led_strip_fade *fade, const uint32_t target[3],
	uint32_t maximum, uint32_t maximum_total, uint32_t frame[3]
)
{
	int32_t sum = 0;
	uint32_t total = 0;
	for (unsigned channel = 0; channel < 3; ++channel) {
		int32_t accumulated = (int32_t)target[channel] + current->error[channel];
		sum += accumulated;
		int32_t rounded = (accumulated + (int32_t)(LED_FADE_ONE / 2)) / (int32_t)LED_FADE_ONE;
		frame[channel] = rounded < 0 ? 0 : (uint32_t)rounded > maximum ? maximum : (uint32_t)rounded;
		fade->error[channel] = accumulated - (int32_t)(frame[channel] * LED_FADE_ONE);
		total += frame[channel];
	}
	int32_t rounded_total = (sum + (int32_t)(LED_FADE_ONE / 2)) / (int32_t)LED_FADE_ONE;
	uint32_t desired = rounded_total < 0 ? 0
		: (uint32_t)rounded_total > maximum_total ? maximum_total : (uint32_t)rounded_total;
	/* Dependent nearest rounding keeps the total residual within half a byte.
	 * Correct the most over/under-served channel, not the hue or logical target.
	 * With three bounded residuals only a few adjacent-byte corrections occur. */
	while (total > desired) {
		unsigned chosen = 0;
		for (unsigned channel = 0; channel < 3; ++channel) {
			if (frame[channel] && (!frame[chosen] || fade->error[channel] < fade->error[chosen])) {
				chosen = channel;
			}
		}
		--frame[chosen];
		fade->error[chosen] += (int32_t)LED_FADE_ONE;
		--total;
	}
	while (total < desired) {
		unsigned chosen = 0;
		for (unsigned channel = 0; channel < 3; ++channel) {
			if (frame[channel] < maximum
				&& (frame[chosen] == maximum || fade->error[channel] > fade->error[chosen])) {
				chosen = channel;
			}
		}
		++frame[chosen];
		fade->error[chosen] -= (int32_t)LED_FADE_ONE;
		++total;
	}
	for (unsigned channel = 0; channel < 3; ++channel) {
		/* Bounded at the output boundary even during cap/near-black clipping.
		 * No rejected, missed or unrepresentable frame can build a backlog. */
		if (fade->error[channel] > (int32_t)LED_FADE_ONE) {
			fade->error[channel] = LED_FADE_ONE;
		}
		if (fade->error[channel] < -(int32_t)LED_FADE_ONE) {
			fade->error[channel] = -(int32_t)LED_FADE_ONE;
		}
	}
}

#endif /* LED_STRIP_FADE_H */
