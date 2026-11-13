/* Host tests: receiver-presence link supervision policy. */
#include "connection/link_supervision.h"

#include <stdio.h>

static int failures;
#define CHECK(cond) do { if (!(cond)) { fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

static struct link_inputs in(uint32_t own, uint32_t heard, uint32_t relayed)
{
	struct link_inputs i = {.own_pong_age_ms = own, .heard_age_ms = heard, .relayed_age_ms = relayed};
	return i;
}

int main(void)
{
	struct link_inputs i;

	/* Own exchange working. */
	i = in(900, 900, LINK_AGE_NEVER);
	CHECK(link_classify(&i) == LINK_HEALTHY && link_delivering(&i) && !link_wants_presence(&i));

	/* Legacy rule searched after 3 lost PINGs / 4.5 s: now one fade of any
	 * length is held while the receiver is still heard. */
	i = in(4600, 300, LINK_AGE_NEVER);
	CHECK(link_classify(&i) == LINK_DEGRADED && link_wants_presence(&i));
	i = in(120000, 300, LINK_AGE_NEVER);
	CHECK(link_classify(&i) == LINK_DEGRADED);
	CHECK(!link_delivering(&i)); /* present but nobody relays it */
	i = in(120000, 300, 400);
	CHECK(link_delivering(&i)); /* relayed: LED healthy, no escalation */

	/* Nothing heard (single tracker, or every uplink blocked): hold the slot
	 * and sync for the holdover, then search. */
	i = in(LINK_UPLINK_HOLDOVER_MS - 1, LINK_UPLINK_HOLDOVER_MS - 1, LINK_AGE_NEVER);
	CHECK(link_classify(&i) == LINK_DEGRADED);
	i = in(LINK_UPLINK_HOLDOVER_MS, LINK_UPLINK_HOLDOVER_MS, LINK_AGE_NEVER);
	CHECK(link_classify(&i) == LINK_LOST && !link_wants_presence(&i));
	i = in(LINK_UPLINK_HOLDOVER_MS, LINK_PRESENCE_TIMEOUT_MS, LINK_AGE_NEVER);
	CHECK(link_classify(&i) == LINK_LOST);
	i = in(LINK_UPLINK_HOLDOVER_MS, LINK_PRESENCE_TIMEOUT_MS - 1, LINK_AGE_NEVER);
	CHECK(link_classify(&i) == LINK_DEGRADED);
	/* Stale relayed confirmation without presence never keeps a tracker. */
	i = in(LINK_UPLINK_HOLDOVER_MS, LINK_AGE_NEVER, 10);
	CHECK(link_classify(&i) == LINK_LOST && !link_delivering(&i));

	/* Budget: holdover below TDMA sync staleness (10 PING intervals) and
	 * the receiver's 15 s eviction grace. */
	CHECK(LINK_UPLINK_HOLDOVER_MS < 10U * 997U);
	CHECK(LINK_UPLINK_HOLDOVER_MS < 15000U);
	CHECK(LINK_DEGRADED_AFTER_MS > 2U * 997U);

	/* Passive sync gate. */
	CHECK(link_passive_sample_acceptable(0) && link_passive_sample_acceptable(-4));
	CHECK(!link_passive_sample_acceptable(5) && !link_passive_sample_acceptable(-32000));

	/* Search: every third probe returns home; the rest walk all candidates. */
	const uint32_t count = 51;
	uint8_t visited[51] = {0};
	for (uint32_t step = 0; step < 3 * count; step++) {
		if (link_search_step_is_home(step)) {
			continue;
		}
		uint32_t c = link_search_step_candidate(step, count);
		CHECK(c >= 1 && c < count);
		visited[c] = 1;
	}
	for (uint32_t c = 1; c < count; c++) {
		CHECK(visited[c]);
	}
	CHECK(link_search_step_is_home(0) && link_search_step_is_home(3) && !link_search_step_is_home(4));
	CHECK(link_search_step_candidate(1, count) == 1 && link_search_step_candidate(2, count) == 2
	      && link_search_step_candidate(4, count) == 3);

	if (failures) {
		fprintf(stderr, "link supervision: %d failure(s)\n", failures);
		return 1;
	}
	printf("link supervision: all tests passed\n");
	return 0;
}
