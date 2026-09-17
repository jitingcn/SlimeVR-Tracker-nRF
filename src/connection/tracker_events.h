#ifndef TRACKER_EVENTS_H
#define TRACKER_EVENTS_H
#include <stdbool.h>
#include <stdint.h>
#include "tracker_event_protocol.h"

/*
 * Producers record bounded, best-effort telemetry under short internal locks.
 * Lock order is owner -> event; record while holding the owner lock, then call
 * tracker_events_notify() after releasing it. No producer sends or waits for RF.
 * The calibration owner retains the returned token through pending apply/storage;
 * token 0 is silent, and token-based updates safely ignore absent operations.
 */
/* Allocate an operation and record ACCEPTED, or return 0 when unavailable. */
uint16_t cal_event_accept(uint8_t kind);
/* Allocate and begin an automatic operation; kind includes its origin flag. */
uint16_t cal_event_begin(uint8_t kind, uint8_t phase, uint8_t detail);
/* Start an already accepted operation without allocating another token. */
void cal_event_start(uint16_t op, uint8_t phase, uint8_t detail);
/* Coalesced progress; STORAGE may supplement a terminal operation. */
void cal_event_step(uint16_t op, uint8_t phase, uint8_t detail);
/* Idempotent terminal result; SUCCESS means applied, not necessarily persisted. */
void cal_event_end(uint16_t op, uint8_t outcome, uint8_t phase, uint8_t reason);
/* Independent rejected-request token; does not terminate another request. */
void cal_event_reject(uint8_t kind, uint8_t reason);
/* Latch a success reason before submitting the owner's pending candidate. */
void cal_event_set_completion_reason(uint16_t op, uint8_t reason);
/* Latest value per rest kind, not a history of all observed edges. */
void tracker_event_set_state(uint8_t kind, uint8_t phase, uint8_t detail);
/* Independent action/intent; never merged with another notice. */
void tracker_event_notice(uint8_t kind, uint8_t phase, uint8_t detail);
/* Owner-lock-free wake only; does not transmit or guarantee delivery. */
void tracker_events_notify(void);
/* Sensor owner captures before acquisition; stale epochs cannot publish. */
uint32_t tracker_events_sensor_epoch(void);
/* Advance the epoch and invalidate both rest observations. Notify after unlock. */
void tracker_events_sensor_invalidate(uint8_t reason);
/* Only each domain's fresh input refreshes it; mismatched epochs are ignored. */
void tracker_events_observe_sensor(
	uint32_t epoch,
	bool tracker_fresh,
	bool tracker_rest,
	bool fusion_fresh,
	bool fusion_rest,
	uint8_t backend,
	uint32_t now_ms
);

/*
 * Single connection-thread sender: select copies an immutable packet/token.
 * Complete that selection before selecting again; success means local ESB
 * admission, never peer delivery. Failed admission preserves the repeat budget.
 * Transport readiness/OTA/test-rate gating remains the caller's responsibility.
 */
struct tracker_event_tx {
	uint8_t packet[TRACKER_EVENT_ESB_LEN];
	uint32_t token;
};
bool tracker_events_select(uint32_t now_ms, uint8_t tracker_id, struct tracker_event_tx *out);
void tracker_events_complete(uint32_t token, bool success, uint32_t now_ms);
/* Absolute uptime milliseconds, UINT32_MAX if no work. */
uint32_t tracker_events_deadline(uint32_t now_ms);
/*
 * Serialized thread-only identity reset; never call from RADIO IRQ.
 * Clears queued telemetry and tokens, rotates nonce, and invalidates sensor epoch.
 * Entropy failure disables events for the rest of this boot, not calibration.
 */
void tracker_events_session_changed(void);
#endif
