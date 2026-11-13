/* Relay compiled out (CONFIG_CONNECTION_RELAY unset): these session fixtures
 * exercise the direct-only paths of the extracted esb.c code. Relay roles,
 * raw-RX arbitration, presence and staged layouts are covered by
 * tests/host/relay against the same production functions. */
#ifndef RADIO_SESSIONS_RELAY_OFF_H
#define RADIO_SESSIONS_RELAY_OFF_H
#include "../../../src/connection/relay.h"
#include "../../../src/connection/link_supervision.h"
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
static bool layout_stale __attribute__((unused));
static inline bool esb_layout_granted(void) { return true; }
static inline int esb_air_claim(void) { return 0; }
static inline void esb_air_release(void) {}
static inline bool tdma_slot_tx_valid(uint8_t payload_length) { (void)payload_length; return true; }
/* No receiver presence and no relayed confirmations: the legacy PING-failure
 * escalation applies unchanged. */
static inline struct link_inputs esb_link_inputs_locked(uint32_t now)
{
	(void)now;
	return (struct link_inputs){.own_pong_age_ms = 0, .heard_age_ms = LINK_AGE_NEVER,
		.relayed_age_ms = LINK_AGE_NEVER};
}
static inline int esb_raw_rx_quiesce(void) { return 0; }
static inline void esb_raw_rx_resume(void) {}
static inline void esb_presence_reset(void) {}
static inline void esb_stage_from_trailer(const struct relay_pong_trailer *trailer, uint32_t t2)
{
	(void)trailer;
	(void)t2;
}
#ifndef IS_ENABLED
/* Only the relay option is queried by the extracted code; anything else
 * fails to compile rather than silently evaluating to 0. */
#define IS_ENABLED(option) IS_ENABLED_##option
#define IS_ENABLED_CONFIG_CONNECTION_RELAY 0
#endif
/* Own-PONG bookkeeping of the extracted receive path; never a trailer. */
static uint32_t receiver_heard_time __attribute__((unused));
static bool receiver_heard_seen __attribute__((unused));
static struct relay_pong_trailer own_trailer __attribute__((unused));
static bool own_trailer_valid __attribute__((unused));
static uint8_t own_pong_length __attribute__((unused));
static inline void esb_note_trailer_locked(const struct relay_pong_trailer *trailer, uint32_t now)
{
	(void)trailer;
	(void)now;
}
#endif
