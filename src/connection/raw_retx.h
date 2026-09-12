#ifndef SLIMENRF_RAW_RETX_H
#define SLIMENRF_RAW_RETX_H

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>

/* Private connection queue. Caller holds irq_lock for every operation.
 * FIFO, duplicate requests are idempotent, full queue drops the new request.
 * Sequences are opaque 16-bit values, including across wire wraparound.
 */
#define RAW_RETX_MAX 16
struct raw_retx_requests {
	uint16_t sequence[RAW_RETX_MAX];
	uint8_t count;
};

static inline int raw_retx_submit(struct raw_retx_requests *requests, uint16_t sequence)
{
	for (uint8_t i = 0; i < requests->count; i++) {
		if (requests->sequence[i] == sequence) {
			return 0;
		}
	}
	if (requests->count == RAW_RETX_MAX) {
		return -ENOSPC;
	}
	requests->sequence[requests->count++] = sequence;
	return 0;
}

static inline bool raw_retx_take(struct raw_retx_requests *requests, uint16_t *sequence)
{
	if (!requests->count) {
		return false;
	}
	*sequence = requests->sequence[0];
	for (uint8_t i = 0; i + 1 < requests->count; i++) {
		requests->sequence[i] = requests->sequence[i + 1];
	}
	requests->count--;
	return true;
}

static inline void raw_retx_reset(struct raw_retx_requests *requests)
{
	requests->count = 0;
}

#endif
