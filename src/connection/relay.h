/* Control feedback assigns helper watch windows and grants cached repairs. */
#ifndef SLIMENRF_RELAY_H
#define SLIMENRF_RELAY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "relay_protocol.h"
#include "../frame_ledger.h"

/* Presence windows while degraded: enough PONGs for sync, little RX cost. */
#define RELAY_PRESENCE_INTERVAL_MS 300U
/* Start HFCLK shortly before passive RX, rather than throughout a wait of
 * up to one PING period. A late clock start simply skips this RX window. */
#define RELAY_PRESENCE_CLOCK_LEAD_US 2000U
/* A child's frame may still be on air at its slot end: wait for END. */
#define RELAY_RX_TAIL_US 400U

#if defined(CONFIG_CONNECTION_RELAY)

void relay_on_control(const struct relay_control *control, uint8_t helper);
void relay_control_serviced(void);
bool relay_next_maintenance(uint32_t *target);
bool relay_take_maintenance(void);
extern uint32_t relay_maintenance_granted, relay_maintenance_late, relay_maintenance_lag_max;
bool relay_next_repair(uint32_t *target, uint8_t *slot);
void relay_repair_result(int result);
size_t relay_build_repair(uint8_t *buf, uint8_t slot);

/* Any context: radio session restarted, channel changed or link lost. */
void relay_reset(void);

/* A current control assignment permits helper listening. */
bool relay_head_active(void);

/* Advertise the single relay protocol. */
uint8_t relay_info_caps(void);

/* Private diagnostic frame (RELAY_DIAG_*) when one is due, else 0. */
size_t relay_diag_build(uint8_t *buf, size_t capacity, uint8_t tracker_id);

void relay_print_status(void);

#else

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

static inline uint8_t relay_info_caps(void) { return 0; }

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
