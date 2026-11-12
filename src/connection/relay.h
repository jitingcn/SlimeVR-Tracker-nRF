/* Tracker side of receiver-coordinated peer relay. See relay_protocol.h for
 * the topology and wire format, link_supervision.h for presence supervision.
 *
 * The relay thread (RELAY_THREAD_PRIORITY) owns peer and presence RX windows.
 * The connection thread owns v3 B control windows and repair submission.
 * Profile 1 keeps three capture frames and follows B watch/grant assignments.
 * Profile-0 compatibility:
 *   - head role: one short RX window in each assigned child's TDMA slot,
 *     keeping the child's latest pose frame until just before its next slot;
 *   - presence: while this tracker's own PING/PONG exchange fails, a few RX
 *     windows per second inside other trackers' PING windows, to overhear
 *     receiver PONGs (presence, time sync, relayed-delivery confirmation).
 * In profile 0 the connection thread consumes pending records in esb.c, either appended to
 * its own frame or as a relay-only bundle in its own or forwarding slot.
 */
#ifndef SLIMENRF_RELAY_H
#define SLIMENRF_RELAY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "relay_protocol.h"
#include "../frame_ledger.h"

/* Head lease: an assignment lives this long after the own PONG that carried
 * it. Own PONGs arrive about once per second; two misses retire the role. The
 * receiver waits longer than this before reusing a revoked forwarding slot. */
#define RELAY_HEAD_LEASE_MS 2500U
/* Presence windows while degraded: enough PONGs for sync, little RX cost. */
#define RELAY_PRESENCE_INTERVAL_MS 300U
/* Start HFCLK shortly before passive RX, rather than throughout a wait of
 * up to one PING period. A late clock start simply skips this RX window. */
#define RELAY_PRESENCE_CLOCK_LEAD_US 2000U
/* A child's frame may still be on air at its slot end: wait for END. */
#define RELAY_RX_TAIL_US 400U
/* A record is forwarded only while a full-size bundle sent now (call latency
 * + 40 us ramp + 72-byte airtime ~ 390 us = 12.8 ticks, +1 tick clock read)
 * still ends before the child's earliest next preamble (its next slot start
 * + TDMA_SLOT_TARGET_OFFSET + ramp), so a relayed copy always precedes the
 * child's next frame. The head's own slot may directly precede the child's
 * (every two-tracker layout); see budget.py for both bounds. */
#define RELAY_FORWARD_GUARD_TICKS 11U

#if defined(CONFIG_CONNECTION_RELAY)

void relay_on_control(const struct relay_control *control, uint8_t helper);
void relay_control_serviced(void);
bool relay_next_maintenance(uint32_t *target);
bool relay_take_maintenance(void);
extern uint32_t relay_maintenance_granted, relay_maintenance_late, relay_maintenance_lag_max;
bool relay_next_repair(uint32_t *target, uint8_t *slot);
void relay_repair_result(int result);
size_t relay_build_repair(uint8_t *buf, uint8_t slot);

/* ISR context: the trailer of this tracker's own extended PONG. */
void relay_on_own_trailer(const struct relay_pong_trailer *trailer);
/* Any context: radio session restarted, channel changed or link lost. */
void relay_reset(void);

/* Head role currently valid (lease, epoch, healthy link). */
bool relay_head_active(void);
/* Valid forwarding slot for this layout, or -1 (then bundle in own slot). */
int relay_forward_slot(void);
/* Unexpired child frames are waiting for forwarding. */
bool relay_records_pending(void);
/* Info capability flags, including a bounded request for forwarding capacity. */
uint8_t relay_info_caps(void);
/* Append pending records after an own frame of own_len bytes already at
 * buf[0] (own_len 0 = relay-only). capacity is the admitted payload budget.
 * Returns the bundle length, or 0 if no record was attached (buf unchanged). */
size_t relay_build_bundle(uint8_t *buf, size_t own_len, size_t capacity, uint8_t head_id);

/* Private diagnostic frame (RELAY_DIAG_*) when one is due, else 0. */
size_t relay_diag_build(uint8_t *buf, size_t capacity, uint8_t tracker_id);

void relay_print_status(void);

#else

static inline void relay_on_own_trailer(const struct relay_pong_trailer *trailer) { (void)trailer; }
static inline void relay_reset(void) {}
static inline void relay_control_serviced(void) {}
static inline void relay_on_control(const struct relay_control *c, uint8_t h)
{
	(void)c;
	(void)h;
}
static inline bool relay_next_maintenance(uint32_t *t) { (void)t; return false; }
static inline bool relay_take_maintenance(void) { return false; }
static inline bool relay_next_repair(uint32_t *t, uint8_t *s)
{
	(void)t;
	(void)s;
	return false;
}
static inline void relay_repair_result(int result) { (void)result; }
static inline size_t relay_build_repair(uint8_t *b, uint8_t s)
{
	(void)b;
	(void)s;
	return 0;
}
static inline bool relay_head_active(void) { return false; }
static inline int relay_forward_slot(void) { return -1; }
static inline bool relay_records_pending(void) { return false; }
static inline uint8_t relay_info_caps(void) { return 0; }
static inline size_t relay_build_bundle(uint8_t *buf, size_t own_len, size_t capacity, uint8_t head_id)
{
	(void)buf; (void)own_len; (void)capacity; (void)head_id;
	return 0;
}
static inline size_t relay_diag_build(uint8_t *buf, size_t capacity, uint8_t tracker_id)
{
	(void)buf; (void)capacity; (void)tracker_id;
	return 0;
}
static inline void relay_print_status(void) {}

#endif

#ifdef CONFIG_RELAY_FRAME_LEDGER
void relay_ledger_tick(uint32_t tick, uint8_t child, uint8_t event, int16_t detail);
#else
#define relay_ledger_tick(...) ((void)0)
#endif

#endif
