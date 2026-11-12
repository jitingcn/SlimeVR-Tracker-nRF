/* Receiver-presence link supervision.
 *
 * The receiver transmits PONGs at +28 dBm through a front-end module while a
 * tracker transmits at +8 dBm, so on a body the downlink survives obstructions
 * that break the uplink. The old rule (three lost own PINGs or 4.5 s without
 * an own PONG) treated every uplink fade as "receiver moved": it discarded time
 * sync, disabled TDMA and swept 51 channels, turning a short fade into a long
 * outage and eventually a membership eviction.
 *
 * A tracker now distinguishes three situations:
 *   HEALTHY   its own PING/PONG exchange works;
 *   DEGRADED  its own exchange fails, but the receiver is still evidently on
 *             this channel (overheard PONGs to other trackers) or the outage
 *             is younger than the uplink holdover. It keeps its slot and sync
 *             and keeps transmitting, so a peer can relay its data;
 *   LOST      neither its own nor any overheard PONG for long enough: only
 *             now is a channel search justified.
 *
 * Pure functions only, so the policy is host-testable.
 */
#ifndef SLIMENRF_LINK_SUPERVISION_H
#define SLIMENRF_LINK_SUPERVISION_H

#include <stdbool.h>
#include <stdint.h>

/* Two missed deterministic PING periods (<= 1 s each) plus scheduling slack. */
#define LINK_DEGRADED_AFTER_MS 2200U
/* Without presence evidence, hold slot and sync this long before searching.
 * Must stay below the TDMA sync staleness bound (10 PING intervals) and well
 * below the receiver's 15 s membership eviction grace. */
#define LINK_UPLINK_HOLDOVER_MS 8000U
/* Overheard receiver traffic younger than this proves presence. Other
 * trackers PING about once per second each, so several PONGs fit. */
#define LINK_PRESENCE_TIMEOUT_MS 4500U
/* A relayed-mask confirmation younger than this proves data delivery. */
#define LINK_RELAYED_TIMEOUT_MS 2000U
#define LINK_AGE_NEVER UINT32_MAX

enum link_state {
	LINK_HEALTHY,
	LINK_DEGRADED,
	LINK_LOST,
};

struct link_inputs {
	uint32_t own_pong_age_ms; /* since own PONG, or since session start */
	uint32_t heard_age_ms;    /* since any receiver PONG (own or overheard) */
	uint32_t relayed_age_ms;  /* since a relayed-mask bit named this tracker */
};

static inline enum link_state link_classify(const struct link_inputs *in)
{
	if (in->own_pong_age_ms < LINK_DEGRADED_AFTER_MS) {
		return LINK_HEALTHY;
	}
	if (in->own_pong_age_ms < LINK_UPLINK_HOLDOVER_MS || in->heard_age_ms < LINK_PRESENCE_TIMEOUT_MS) {
		return LINK_DEGRADED;
	}
	return LINK_LOST;
}

/* Data reaches the receiver: directly, or through a peer while present. */
static inline bool link_delivering(const struct link_inputs *in)
{
	enum link_state state = link_classify(in);
	return state == LINK_HEALTHY
		|| (state == LINK_DEGRADED && in->heard_age_ms < LINK_PRESENCE_TIMEOUT_MS
		    && in->relayed_age_ms < LINK_RELAYED_TIMEOUT_MS);
}

/* Passive PONG listening is worth its RX cost only once the own exchange has
 * actually failed, and is pointless once the tracker is searching anyway. */
static inline bool link_wants_presence(const struct link_inputs *in)
{
	return link_classify(in) == LINK_DEGRADED;
}

/* Overheard PONGs feed the clock filter only when they agree with the current
 * estimate. A large disagreement means a stale estimate or a misparsed frame;
 * the own exchange (with its RTT gate) remains the authority for big steps. */
#define LINK_PASSIVE_MAX_INNOVATION_TICKS 4

static inline bool link_passive_sample_acceptable(int64_t innovation_ticks)
{
	return innovation_ticks <= LINK_PASSIVE_MAX_INNOVATION_TICKS
		&& innovation_ticks >= -LINK_PASSIVE_MAX_INNOVATION_TICKS;
}

/* Blind search interleaves the home channel: an all-uplink obstruction (body
 * over the receiver) looks exactly like a channel move to every tracker, and
 * the home channel is where the receiver most likely still is. Every third
 * probe returns home; the remaining probes walk the ordinary candidate list. */
#define LINK_SEARCH_HOME_EVERY 3U

static inline bool link_search_step_is_home(uint32_t step)
{
	return step % LINK_SEARCH_HOME_EVERY == 0;
}

/* Index into the ordinary candidate list (1..count-1) for a non-home step. */
static inline uint32_t link_search_step_candidate(uint32_t step, uint32_t candidate_count)
{
	if (candidate_count <= 1) {
		return 0;
	}
	uint32_t walk = step - step / LINK_SEARCH_HOME_EVERY - 1U;
	return 1U + walk % (candidate_count - 1U);
}

#endif
