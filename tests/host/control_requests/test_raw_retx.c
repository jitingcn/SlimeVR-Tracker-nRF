#include <assert.h>
#include <stdio.h>
#include "../../../src/connection/raw_retx.h"

int main(void)
{
	struct raw_retx_requests requests = {0};
	uint16_t sequence = 123;
	assert(!raw_retx_take(&requests, &sequence));
	assert(sequence == 123);

	/* Wrap is just another FIFO value; duplicates must not reorder requests. */
	assert(raw_retx_submit(&requests, UINT16_MAX) == 0);
	assert(raw_retx_submit(&requests, 0) == 0);
	assert(raw_retx_submit(&requests, UINT16_MAX) == 0);
	assert(raw_retx_take(&requests, &sequence) && sequence == UINT16_MAX);
	assert(raw_retx_take(&requests, &sequence) && sequence == 0);
	assert(!raw_retx_take(&requests, &sequence));

	for (uint16_t i = 0; i < RAW_RETX_MAX; i++) {
		assert(raw_retx_submit(&requests, i) == 0);
	}
	/* A duplicate is still accepted when full, but cannot displace history. */
	assert(raw_retx_submit(&requests, 0) == 0);
	assert(raw_retx_submit(&requests, 1000) == -ENOSPC);
	for (uint16_t i = 0; i < RAW_RETX_MAX; i++) {
		assert(raw_retx_take(&requests, &sequence) && sequence == i);
	}
	assert(!raw_retx_take(&requests, &sequence));
	assert(raw_retx_submit(&requests, 1000) == 0);
	assert(raw_retx_take(&requests, &sequence) && sequence == 1000);

	assert(raw_retx_submit(&requests, 55) == 0);
	raw_retx_reset(&requests);
	assert(!raw_retx_take(&requests, &sequence));
	assert(raw_retx_submit(&requests, 55) == 0);
	assert(raw_retx_take(&requests, &sequence) && sequence == 55);
	puts("raw retransmit queue: PASS");
	return 0;
}
