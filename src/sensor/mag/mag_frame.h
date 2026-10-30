#ifndef SLIMENRF_SENSOR_MAG_FRAME
#define SLIMENRF_SENSOR_MAG_FRAME

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>

#define MAG_FRAME_MAX_BYTES 9

/* Continuous reads without a usable data-ready bit can repeat a sample. */
struct mag_frame_filter {
	uint8_t last[MAG_FRAME_MAX_BYTES];
	uint8_t length;
	int64_t accepted_ms;
	int64_t period_ms;
};

/* A non-finite or non-positive period never accepts an unchanged payload. */
static inline void mag_frame_reset(struct mag_frame_filter *filter, float period_s)
{
	filter->length = 0;
	filter->period_ms = period_s > 0.0f && period_s < 60.0f ? (int64_t)ceilf(period_s * 1000.0f) : 0;
}

/* An unchanged payload is new only after two periods, so a quantized static
 * field keeps producing samples while a repeated read does not. */
static inline bool mag_frame_is_new(struct mag_frame_filter *filter, const uint8_t *frame, uint8_t length)
{
	if (length == 0 || length > MAG_FRAME_MAX_BYTES)
		return false;
	int64_t now = k_uptime_get();
	if (filter->length == length && memcmp(filter->last, frame, length) == 0) {
		if (filter->period_ms == 0 || now - filter->accepted_ms < 2 * filter->period_ms)
			return false;
	} else {
		memcpy(filter->last, frame, length);
		filter->length = length;
	}
	filter->accepted_ms = now;
	return true;
}

#endif
