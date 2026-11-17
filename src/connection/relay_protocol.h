/* Shared relay protocol. Normal direct TDMA has no control broadcasts.
 * On demand, control feedback assigns helper watches and grants repair slots.
 * PONG layout trailers stage coordinated activation. Bytes 2..4 and 7..8 of
 * the trailer are reserved zero; helper assignments and delivery feedback
 * travel exclusively in control frames. Protocol version validates wire format.
 */
#ifndef SLIMENRF_RELAY_PROTOCOL_H
#define SLIMENRF_RELAY_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define RELAY_PROTOCOL_VERSION 5U
#define RELAY_MAX_CHILDREN 3U
#define RELAY_MAX_TRACKERS 16U

#define RELAY_FRAME_MAX 72U

/* Each control gets its own arming interval after the preceding RX window. */
#define RELAY_CONTROL_ARM_GUARD_TICKS 4U
#define RELAY_TX_OFFSET 2U /* preamble target inside a slot, as TDMA_SLOT_TARGET_OFFSET */

#define RELAY_CAP_HELPER 0x01U   /* can listen for and forward peers */
#define RELAY_CAP_PRESENCE 0x02U /* keeps sync from overheard PONGs */
#define RELAY_CAP_VERSION_SHIFT 4
#define RELAY_INFO_CAPS \
	(RELAY_CAP_HELPER | RELAY_CAP_PRESENCE | (RELAY_PROTOCOL_VERSION << RELAY_CAP_VERSION_SHIFT))
#define RELAY_INFO_CAPS_INDEX 7U /* data[7] of a 16-byte info frame */

#define RELAY_PONG_BASE_LEN 13U
#define RELAY_TRAILER_MAGIC 0x5AU
#define RELAY_TRAILER_LEN 10U
#define RELAY_PONG_EXT_LEN (RELAY_PONG_BASE_LEN + RELAY_TRAILER_LEN)
#define RELAY_TRAILER_MAGIC_PENDING 0x5BU
#define RELAY_TRAILER_PENDING_LEN 17U
#define RELAY_PONG_EXT_PENDING_LEN (RELAY_PONG_BASE_LEN + RELAY_TRAILER_PENDING_LEN)
#define RELAY_ACTIVATION_UNIT_TICKS 4U
/* Grant < announcement lead; the gap covers clock disagreement and a frame. */
#define RELAY_LAYOUT_GRANT_MS 3000U
#define RELAY_LAYOUT_ACTIVATION_MS 3500U

/* Ordinary pose traffic: a 17-byte normal frame or a composite. */
#define RELAY_NORMAL_FRAME_LEN 17U
#define RELAY_COMPOSITE_TYPE 0xFEU
#define RELAY_COMPOSITE_MIN_LEN 5U
#define RELAY_RECORD_MAX_LEN RELAY_DIRECT_MAX

/* Private relay diagnostics, tracker -> receiver, about once per second
 * while the tracker holds an assignment or runs presence windows, and only
 * when its receiver sends extended PONGs. Never forwarded to HID.
 *   [0] type  [1] id  [2] flags  [3] trailer epoch  [4] layout epoch
 *   [5] own slot  [6] total slots  [7] slot ticks  [8..9] children BE
 *   [10..11] member mask BE  [12..] RELAY_DIAG_COUNTERS BE16 counters
 * (wrapping), in enum relay_diag_counter order. */
#define RELAY_DIAG_TYPE 0xE7U
#define RELAY_DIAG_HEADER_LEN 12U
#define RELAY_DIAG_COUNTERS 16U
#define RELAY_DIAG_LEN (RELAY_DIAG_HEADER_LEN + 2U * RELAY_DIAG_COUNTERS)
#define RELAY_DIAG_INTERVAL_MS 1000U
#define RELAY_DIAG_F_ASSIGNED 0x01U  /* trailer with children received recently */
#define RELAY_DIAG_F_LEASE 0x02U     /* lease still fresh */
#define RELAY_DIAG_F_EPOCH 0x04U     /* trailer epoch == layout epoch */
#define RELAY_DIAG_F_HEAD 0x08U      /* head role active */
#define RELAY_DIAG_F_LAYOUT 0x10U    /* TDMA layout available */
#define RELAY_DIAG_LINK_SHIFT 5      /* 2 bits: enum link_state */

#define RELAY_CONTROL 0xE8U
#define RELAY_CONTROL_REPEAT 0xECU
#define RELAY_REPAIR 0xE9U
#define RELAY_PEER_REPORT 0xEAU
#define RELAY_LAYOUT 0xEBU
#define RELAY_CONTROL_LEN 72U
#define RELAY_LAYOUT_LEN 30U
#define RELAY_PROFILE_120HZ 1U
/* Runtime geometry belongs to one activated membership. Profile 0 keeps
 * its normal direct schedule. Period fields describe the base R only;
 * actual frame starts come from the current/next-R announcements. */
#if defined(__GNUC__)
#define RELAY_LAYOUT_INLINE static inline __attribute__((always_inline))
#else
#define RELAY_LAYOUT_INLINE static inline
#endif
struct relay_geometry {
	uint8_t direct, repairs, slot_ticks;
	uint16_t tps, numerator, denominator;
	uint8_t control_repeat_mode;
};
/* Receiver settings describe capacity. Frame geometry uses present members. */
struct relay_runtime_settings {
	uint8_t max_trackers, max_repairs, slot_ticks;
	uint16_t tps_min, tps_max;
	uint8_t repair_floor;
	uint8_t control_repeat_mode;
};

/* Targets guide the repair budget; minimum R and maintenance take priority. */
static inline bool relay_runtime_validate(struct relay_runtime_settings *settings, bool experimental)
{
	const struct relay_runtime_settings *s = settings;
	return s->max_trackers && s->max_trackers <= RELAY_MAX_TRACKERS && s->control_repeat_mode <= 2U && s->max_repairs >= 1U
		&& s->max_repairs <= 5U && s->repair_floor <= s->max_repairs
		&& (s->slot_ticks == 16U || (experimental && s->slot_ticks == 15U)) && s->tps_min && s->tps_min <= s->tps_max
		&& s->tps_max <= 32768U;
}

/* Adjacent repeat control frame has one full vacant slot after primary control frame for PRX restoration and
 * repeat preparation. Tail repeat control frame needs only its own occupied slot. */
static inline unsigned relay_control_repeat_slots(unsigned mode)
{
	return mode == 1U ? 2U : mode == 2U ? 1U : 0U;
}

/* Compute once for the staged member set, never from the configured member cap. */
static inline struct relay_runtime_settings relay_runtime_for_members(struct relay_runtime_settings s, uint8_t direct)
{
	unsigned ticks = 32768U / s.tps_min;
	unsigned slots = ticks > RELAY_CONTROL_ARM_GUARD_TICKS
		? (ticks - RELAY_CONTROL_ARM_GUARD_TICKS) / s.slot_ticks : 0U;
	unsigned budget = slots > direct + 1U ? slots - direct - 1U : 1U;
	unsigned repairs = budget;
	if (repairs > s.max_repairs) {
		repairs = s.max_repairs;
	}
	bool fixed = s.repair_floor == s.max_repairs;
	unsigned repeat_slots = ticks > 2U * RELAY_CONTROL_ARM_GUARD_TICKS
		? (ticks - 2U * RELAY_CONTROL_ARM_GUARD_TICKS) / s.slot_ticks : 0U;
	unsigned repeat_budget = repeat_slots > direct + 1U ? repeat_slots - direct - 1U : 0U;
	s.control_repeat_mode = repairs >= 3U
		&& repeat_budget >= relay_control_repeat_slots(s.control_repeat_mode) + 2U ? s.control_repeat_mode : 0U;
	if (s.control_repeat_mode) {
		repairs--;
		unsigned available = repeat_budget - relay_control_repeat_slots(s.control_repeat_mode);
		if (repairs > available) {
			repairs = available;
		}
	}
	s.max_repairs = repairs;
	if (fixed || s.repair_floor > repairs) {
		s.repair_floor = repairs;
	}
	if (s.control_repeat_mode && s.repair_floor < 2U) {
		s.repair_floor = 2U;
	}
	return s;
}

/* The cap grid is rational and tied to frame number. Occupied time can
 * lengthen any individual frame without losing fractional cap padding. */
static inline uint16_t relay_runtime_period(
	uint32_t frame, uint8_t direct, uint8_t repairs, uint8_t slot_ticks, uint16_t cap, uint8_t mode
)
{
	uint32_t occupied = (direct + 1U + repairs + relay_control_repeat_slots(mode)) * slot_ticks
		+ RELAY_CONTROL_ARM_GUARD_TICKS * (1U + (mode != 0U));
	uint32_t remainder = 32768U % cap;
	uint32_t phase = (frame % cap) * remainder % cap;
	uint32_t padding = 32768U / cap + (phase + remainder >= cap);
	return occupied > padding ? occupied : padding;
}

/* The control frame announces current R and the next two R counts. These determine three
 * future starts, so two missed control frames do not interrupt the known direct schedule.
 * A fourth future start is unknown. Differences handle RTC rollover. */
struct relay_runtime_anchor {
	uint32_t frame, start, next_start, following_start, third_start;
	uint8_t direct, slot_ticks;
	bool valid;
};

static inline void relay_runtime_anchor_set(
	struct relay_runtime_anchor *a,
	uint32_t frame,
	uint32_t preamble,
	uint8_t direct,
	uint8_t repairs,
	uint8_t next_repairs,
	uint8_t next2_repairs,
	uint8_t slot_ticks,
	uint16_t cap,
	uint8_t mode
)
{
	a->frame = frame;
	a->direct = direct;
	a->slot_ticks = slot_ticks;
	a->start = preamble - direct * slot_ticks - RELAY_CONTROL_ARM_GUARD_TICKS - RELAY_TX_OFFSET;
	a->next_start = a->start + relay_runtime_period(frame, direct, repairs, slot_ticks, cap, mode);
	a->following_start = a->next_start + relay_runtime_period(frame + 1U, direct, next_repairs, slot_ticks, cap, mode);
	a->third_start = a->following_start + relay_runtime_period(frame + 2U, direct, next2_repairs, slot_ticks, cap, mode);
	a->valid = true;
}


static inline bool relay_runtime_start(const struct relay_runtime_anchor *a, uint32_t frame, uint32_t *start)
{
	if (!a->valid || (uint32_t)(frame - a->frame) > 3U) {
		return false;
	}
	*start = frame == a->frame      ? a->start
		   : frame == a->frame + 1U ? a->next_start
		   : frame == a->frame + 2U ? a->following_start
									: a->third_start;
	return true;
}

static inline bool relay_runtime_direct_granted(const struct relay_runtime_anchor *a, uint32_t tick)
{
	return a->valid && (int32_t)(tick - a->start) >= 0
		&& (int32_t)(tick - (a->third_start + a->direct * a->slot_ticks)) < 0;
}

#define RELAY_REPAIR_MAX 5U

static inline bool
relay_geometry_make(struct relay_geometry *g, uint8_t direct, uint8_t repairs, uint8_t slot_ticks, uint16_t cap)
{
	if (!direct || direct > RELAY_MAX_TRACKERS || repairs > RELAY_REPAIR_MAX
		|| (slot_ticks != 15U && slot_ticks != 16U) || !cap || cap > 32768U) {
		return false;
	}
	uint32_t occupied = (direct + 1U + repairs) * slot_ticks + RELAY_CONTROL_ARM_GUARD_TICKS;
	uint16_t divisor = cap & (uint16_t)(0U - cap);
	*g = (struct relay_geometry){
		direct,
		repairs,
		slot_ticks,
		cap,
		occupied * cap >= 32768U ? occupied : 32768U / divisor,
		occupied * cap >= 32768U ? 1U : cap / divisor,
		0
	};
	return true;
}

/* The extra control frame consumes the repair slot traded away by the staged policy. */
static inline bool relay_geometry_make_control(
	struct relay_geometry *g,
	uint8_t direct,
	uint8_t repairs,
	uint8_t width,
	uint16_t cap,
	uint8_t control_repeat_mode
)
{
	if (control_repeat_mode > 2U || (control_repeat_mode && (repairs < 2U || repairs > 4U))) {
		return false;
	}
	if (!relay_geometry_make(g, direct, repairs, width, cap)) {
		return false;
	}
	unsigned occupied = (direct + 1U + repairs + relay_control_repeat_slots(control_repeat_mode)) * width
		+ RELAY_CONTROL_ARM_GUARD_TICKS * (1U + (control_repeat_mode != 0U));
	if (occupied * cap >= 32768U) {
		g->numerator = occupied;
		g->denominator = 1;
	}
	g->control_repeat_mode = control_repeat_mode;
	return true;
}

static inline uint32_t relay_control_repeat_offset(uint8_t direct, uint8_t repairs, uint8_t width, uint8_t mode)
{
	(void)direct;
	return mode == 1U ? 2U * width + RELAY_CONTROL_ARM_GUARD_TICKS
		: mode == 2U ? (repairs + 1U) * width + RELAY_CONTROL_ARM_GUARD_TICKS : 0U;
}

/* Private PONG geometry: bit7 private, bits6..3 direct-1, bits2..0 base R.
 * Width uses the existing pending width byte. The control frame supplies the exact TPS cap. */
static inline uint8_t relay_geometry_code(uint8_t direct, uint8_t repairs)
{
	return 0x80U | ((direct - 1U) << 3) | repairs;
}

static inline bool relay_geometry_from_bootstrap(uint8_t code, uint8_t slot_ticks, struct relay_geometry *g)
{
	return (code & 0x80U) && relay_geometry_make(g, ((code >> 3) & 15U) + 1U, code & 7U, slot_ticks, 120U);
}

#if defined(CONFIG_RELAY_RUNTIME_LAYOUT) || defined(CONFIG_RELAY_LAYOUT_FOLLOW_RECEIVER)
extern const struct relay_geometry *relay_geometry_active;
RELAY_LAYOUT_INLINE const struct relay_geometry *relay_geometry_current(void)
{
	return relay_geometry_active;
}
#else
/* Host allocator references use explicit constant geometry, independently of
 * the receiver's runtime configuration. There is no device layout table. */
#ifndef RELAY_HOST_DIRECT_SLOTS
#define RELAY_HOST_DIRECT_SLOTS 11U
#endif
#ifndef RELAY_HOST_REPAIR_SLOTS
#define RELAY_HOST_REPAIR_SLOTS 5U
#endif
#ifndef RELAY_HOST_SLOT_TICKS
#define RELAY_HOST_SLOT_TICKS 16U
#endif
#ifndef RELAY_HOST_TPS_MAX
#define RELAY_HOST_TPS_MAX 120U
#endif
#define RELAY_HOST_OCCUPIED \
	((RELAY_HOST_DIRECT_SLOTS + 1U + RELAY_HOST_REPAIR_SLOTS) * RELAY_HOST_SLOT_TICKS + RELAY_CONTROL_ARM_GUARD_TICKS)
#define RELAY_HOST_LONG (RELAY_HOST_OCCUPIED * RELAY_HOST_TPS_MAX >= 32768U)
#define RELAY_HOST_DIVISOR (RELAY_HOST_TPS_MAX & (0U - RELAY_HOST_TPS_MAX))
#define RELAY_HOST_NUMERATOR (RELAY_HOST_LONG ? RELAY_HOST_OCCUPIED : 32768U / RELAY_HOST_DIVISOR)
#define RELAY_HOST_DENOMINATOR (RELAY_HOST_LONG ? 1U : RELAY_HOST_TPS_MAX / RELAY_HOST_DIVISOR)
RELAY_LAYOUT_INLINE const struct relay_geometry *relay_geometry_current(void)
{
	static const struct relay_geometry fixture
		= {RELAY_HOST_DIRECT_SLOTS,
		   RELAY_HOST_REPAIR_SLOTS,
		   RELAY_HOST_SLOT_TICKS,
		   RELAY_HOST_TPS_MAX,
		   RELAY_HOST_NUMERATOR,
		   RELAY_HOST_DENOMINATOR,
		   0};
	return &fixture;
}
#endif
#if defined(CONFIG_RELAY_RUNTIME_LAYOUT) || defined(CONFIG_RELAY_LAYOUT_FOLLOW_RECEIVER)
#define RELAY_DIRECT_SLOTS (relay_geometry_current()->direct)
#define RELAY_REPAIR_SLOTS (relay_geometry_current()->repairs)
#define RELAY_SLOT_TICKS (relay_geometry_current()->slot_ticks)
#define RELAY_GRID_TICKS (relay_geometry_current()->numerator)
#define RELAY_GRID_FRAMES (relay_geometry_current()->denominator)
#else
#define RELAY_DIRECT_SLOTS RELAY_HOST_DIRECT_SLOTS
#define RELAY_REPAIR_SLOTS RELAY_HOST_REPAIR_SLOTS
#define RELAY_SLOT_TICKS RELAY_HOST_SLOT_TICKS
#define RELAY_GRID_TICKS RELAY_HOST_NUMERATOR
#define RELAY_GRID_FRAMES RELAY_HOST_DENOMINATOR
#endif
#define RELAY_TOTAL_SLOTS \
	(RELAY_DIRECT_SLOTS + 1U + RELAY_REPAIR_SLOTS + relay_control_repeat_slots(relay_geometry_current()->control_repeat_mode))

#define RELAY_TIMELINE_FRAMES 8U
struct relay_timeline {
	uint32_t frame, start[RELAY_TIMELINE_FRAMES], tag[RELAY_TIMELINE_FRAMES];
	uint8_t valid;
};

static inline void relay_timeline_clear(struct relay_timeline *t)
{
	memset(t, 0, sizeof(*t));
}

static inline void relay_timeline_put(struct relay_timeline *t, uint32_t frame, uint32_t start)
{
	unsigned slot = frame % RELAY_TIMELINE_FRAMES;
	t->start[slot] = start;
	t->tag[slot] = frame;
	t->valid |= 1U << slot;
}

static inline bool relay_timeline_start(const struct relay_timeline *t, uint32_t frame, uint32_t *start)
{
	unsigned slot = frame % RELAY_TIMELINE_FRAMES;
	if (!(t->valid & (1U << slot)) || t->tag[slot] != frame) {
		return false;
	}
	*start = t->start[slot];
	return true;
}

static inline void relay_timeline_announce(struct relay_timeline *t, const struct relay_runtime_anchor *a)
{
	t->frame = a->frame;
	relay_timeline_put(t, a->frame, a->start);
	relay_timeline_put(t, a->frame + 1U, a->next_start);
	relay_timeline_put(t, a->frame + 2U, a->following_start);
	relay_timeline_put(t, a->frame + 3U, a->third_start);
}

#if defined(CONFIG_RELAY_RUNTIME_LAYOUT)
extern struct relay_timeline relay_timeline_active;
extern struct relay_runtime_anchor relay_direct_anchor;
#endif
/* RX starts this many ticks before a peer's slot (or control frame): 40 us fast ramp-up, the
 * receiver settling before the preamble, and each tracker's receiver-time
 * estimate being off by a tick or two (head and child err independently).
 * Only the child's pipe is enabled, so an early window cannot capture the
 * previous slot's traffic. Hardware: 1 tick missed ~70% of child frames. */
#define RELAY_RX_LEAD_TICKS 4U
/* Guard after the direct window, and another before an optional repeat. */
#define RELAY_CONTROL_START (RELAY_DIRECT_SLOTS * RELAY_SLOT_TICKS + RELAY_CONTROL_ARM_GUARD_TICKS)
#define RELAY_CONTROL_PREAMBLE (RELAY_CONTROL_START + RELAY_TX_OFFSET)
#define RELAY_REPAIR_START \
	(RELAY_CONTROL_START + RELAY_SLOT_TICKS * (1U + (2U * (relay_geometry_current()->control_repeat_mode == 1U))) \
		+ RELAY_CONTROL_ARM_GUARD_TICKS * (relay_geometry_current()->control_repeat_mode == 1U))
#define RELAY_REPAIR_FIRST_SLOT (RELAY_DIRECT_SLOTS + 1U + (2U * (relay_geometry_current()->control_repeat_mode == 1U)))

/* Logical repair slot numbers exclude guard ticks; direct slots are unchanged. */
static inline uint32_t relay_slot_offset(unsigned slot)
{
	return slot >= RELAY_REPAIR_FIRST_SLOT
		? RELAY_REPAIR_START + (slot - RELAY_REPAIR_FIRST_SLOT) * RELAY_SLOT_TICKS
		: slot * RELAY_SLOT_TICKS;
}
#define RELAY_CAPTURE_TAIL_TICKS 2U
#define RELAY_CONTROL_PREP_LEAD_TICKS 3U
#define RELAY_REPAIR_HEADER_LEN 8U
#define RELAY_REPAIR_RECORD_META_LEN 4U
/* Ticks on air at 2 Mbps, rounded up: 83 bits of preamble, address, PCF and CRC. */
#define RELAY_AIR_TICKS(length) (((83U + 8U * (length)) * 32768U + 1999999U) / 2000000U)
#define RELAY_FRAME_TICKS_MAX ((RELAY_GRID_TICKS + RELAY_GRID_FRAMES - 1U) / RELAY_GRID_FRAMES)
#define RELAY_CONTROL_AIR_TICKS 11U /* 72-byte control frame on air: 329.5 us */
#define RELAY_DIRECT_MAX 60U
#define RELAY_LAST_DIRECT_MAX 32U
#define RELAY_CACHE_FRAMES 3U
#define RELAY_CACHE_MAX_FRAMES 5U
#define RELAY_OWNER_IDLE 0xFFU
#define RELAY_OWNER_MAINTENANCE 0xFEU
#define RELAY_OWNER_LAYOUT 0xFDU
#define RELAY_CONTROL_MAINTENANCE 0x01U
#define RELAY_CONTROL_PENDING 0x02U
#define RELAY_TRAILER_PROFILE_MAGIC 0x5CU

enum relay_diag_counter {
	RELAY_DIAG_WINDOWS,      /* head RX windows opened */
	RELAY_DIAG_NO_WINDOW,    /* head loop found no schedulable child window */
	RELAY_DIAG_BUSY,         /* raw RX refused (ESB busy, writer, blocked) */
	RELAY_DIAG_RX_ANY,       /* any address match seen in a head window */
	RELAY_DIAG_CRC_FAIL,     /* head window frame failed CRC */
	RELAY_DIAG_CAPTURES,     /* child pose frames stored */
	RELAY_DIAG_FOREIGN,      /* valid frames that were not the child's pose */
	RELAY_DIAG_FORWARDED,    /* repair records submitted */
	RELAY_DIAG_EXPIRED,      /* records dropped unsent */
	RELAY_DIAG_PRESENCE,     /* presence windows opened */
	RELAY_DIAG_PRESENCE_RX,  /* overheard PONGs accepted */
	RELAY_DIAG_LEASE_EXPIRED,
	RELAY_DIAG_LEN_FAIL,     /* CRC ok but length 0 or > RELAY_FRAME_MAX */
	RELAY_DIAG_RSSI,         /* high byte: last failed frame -dBm, low: last capture */
	RELAY_DIAG_START_LATE,   /* worst RX start after its target since last report, ticks */
	RELAY_DIAG_LENGTHS,      /* high byte: last failed length byte, low: last capture */
};

struct relay_pong_trailer {
	uint8_t epoch;
	uint16_t member_mask;
	/* Pending layout (RELAY_TRAILER_MAGIC_PENDING), see above. */
	bool pending;
	uint8_t pending_epoch;
	uint8_t pending_total;
	uint8_t pending_slot_ticks;
	uint16_t pending_member_mask;
	uint16_t activation_units;
};

struct relay_record_view {
	const uint8_t *frame;
	uint8_t length;
	uint8_t rssi;
};

/* Same polynomial and bit order as Zephyr crc8_ccitt(). */
#if defined(__GNUC__)
/* CRC inlining/unrolling is part of the measured receiver frozen encoder. */
__attribute__((always_inline))
#endif
static inline uint8_t relay_crc8(uint8_t value, const uint8_t *data, size_t length)
{
	static const uint8_t table[256] = {
		0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
		0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
		0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
		0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
		0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
		0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
		0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
		0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
		0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
		0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
		0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
		0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
		0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
		0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
		0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
		0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3,
	};
#if defined(__GNUC__) && !defined(__clang__)
/* Retained with the linked encoder model; no isolated unroll-only A/B. */
#pragma GCC unroll 32
#endif
	for (size_t i = 0; i < length; i++) {
		value = table[value ^ data[i]];
	}
	return value;
}

/* True for frames a head may forward for tracker_id: the ordinary pose frames
 * the receiver sequences, never PING, OTA, raw collection or private events. */
static inline bool relay_frame_is_pose(const uint8_t *frame, uint8_t length, uint8_t tracker_id)
{
	if (length < 2 || frame[1] != tracker_id || tracker_id >= RELAY_MAX_TRACKERS) {
		return false;
	}
	if (frame[0] == RELAY_COMPOSITE_TYPE) {
		return length >= RELAY_COMPOSITE_MIN_LEN && length <= RELAY_FRAME_MAX;
	}
	return length == RELAY_NORMAL_FRAME_LEN && frame[0] < 0x10U;
}

/* Writes RELAY_TRAILER_LEN, or RELAY_TRAILER_PENDING_LEN bytes when a layout
 * change is pending; returns the length written. */
static inline size_t relay_trailer_encode(uint8_t *out, const struct relay_pong_trailer *trailer)
{
	out[0] = trailer->pending ? ((trailer->pending_total & 0x80U)
		? RELAY_TRAILER_PROFILE_MAGIC : RELAY_TRAILER_MAGIC_PENDING) : RELAY_TRAILER_MAGIC;
	out[1] = trailer->epoch;
	out[2] = 0;
	out[3] = 0;
	out[4] = 0;
	out[5] = (uint8_t)(trailer->member_mask >> 8);
	out[6] = (uint8_t)trailer->member_mask;
	out[7] = 0;
	out[8] = 0;
	if (!trailer->pending) {
		out[9] = relay_crc8(0x07, out, RELAY_TRAILER_LEN - 1);
		return RELAY_TRAILER_LEN;
	}
	out[9] = trailer->pending_epoch;
	out[10] = trailer->pending_total;
	out[11] = trailer->pending_slot_ticks;
	out[12] = (uint8_t)(trailer->pending_member_mask >> 8);
	out[13] = (uint8_t)trailer->pending_member_mask;
	out[14] = (uint8_t)(trailer->activation_units >> 8);
	out[15] = (uint8_t)trailer->activation_units;
	out[16] = relay_crc8(0x07, out, RELAY_TRAILER_PENDING_LEN - 1);
	return RELAY_TRAILER_PENDING_LEN;
}

/* PONG lengths that carry a trailer. */
static inline bool relay_pong_is_extended(uint8_t length)
{
	return length == RELAY_PONG_EXT_LEN || length == RELAY_PONG_EXT_PENDING_LEN;
}

static inline uint8_t relay_mask_count(uint16_t mask)
{
	uint8_t count = 0;
	for (; mask; mask &= (uint16_t)(mask - 1U)) {
		count++;
	}
	return count;
}

/* Decodes a trailer of `length` bytes (PONG length - 13). */
static inline bool relay_trailer_decode_len(const uint8_t *in, uint8_t length, struct relay_pong_trailer *trailer)
{
	bool pending = length == RELAY_TRAILER_PENDING_LEN && (in[0] == RELAY_TRAILER_MAGIC_PENDING || in[0] == RELAY_TRAILER_PROFILE_MAGIC);
	if (!pending && !(length == RELAY_TRAILER_LEN && in[0] == RELAY_TRAILER_MAGIC)) {
		return false;
	}
	if (in[2] || in[3] || in[4] || in[7] || in[8]
	    || in[length - 1U] != relay_crc8(0x07, in, length - 1U)) {
		return false;
	}
	trailer->epoch = in[1];
	trailer->member_mask = (uint16_t)(((uint16_t)in[5] << 8) | in[6]);
	trailer->pending = pending;
	trailer->pending_epoch = pending ? in[9] : 0;
	trailer->pending_total = pending ? in[10] : 0;
	trailer->pending_slot_ticks = pending ? in[11] : 0;
	trailer->pending_member_mask = pending ? (uint16_t)(((uint16_t)in[12] << 8) | in[13]) : 0;
	trailer->activation_units = pending ? (uint16_t)(((uint16_t)in[14] << 8) | in[15]) : 0;
	if (pending && in[0] == RELAY_TRAILER_PROFILE_MAGIC) {
		struct relay_geometry g;
		if (!relay_geometry_from_bootstrap(trailer->pending_total, trailer->pending_slot_ticks, &g)
			|| !trailer->pending_member_mask || relay_mask_count(trailer->pending_member_mask) != g.direct) {
			return false;
		}
	}
	return true;
}

static inline bool relay_trailer_decode(const uint8_t in[RELAY_TRAILER_LEN], struct relay_pong_trailer *trailer)
{
	return relay_trailer_decode_len(in, RELAY_TRAILER_LEN, trailer);
}

/* Plan in the trailing vacant direct slots when they provide enough lead.
 * This avoids blocking RX IRQs during real direct traffic. Freeze, launch and
 * the short-lead rejection remain unchanged. A full layout keeps 120 ticks. */
static inline uint32_t relay_plan_lead(uint16_t members)
{
	uint32_t present = relay_mask_count(members);
	uint32_t vacant = present < RELAY_DIRECT_SLOTS ? RELAY_DIRECT_SLOTS - present : 0;
	uint32_t lead = vacant * RELAY_SLOT_TICKS + RELAY_TX_OFFSET;
	return lead >= 32U && lead < 120U ? lead : 120U;
}

/* Slot of a direct member: its rank among the members (receiver id order). */
static inline int relay_member_slot(uint16_t member_mask, uint8_t tracker_id)
{
	if (tracker_id >= RELAY_MAX_TRACKERS || !(member_mask & (1U << tracker_id))) {
		return -1;
	}
	return __builtin_popcount(member_mask & ((1U << tracker_id) - 1U));
}

/* A helper can overhear a child unless their direct slots are adjacent: the
 * helper is still turning its radio around for its own slot. */
static inline bool relay_can_overhear(uint16_t member_mask, uint8_t helper, uint8_t child)
{
	int hs = relay_member_slot(member_mask, helper);
	int cs = relay_member_slot(member_mask, child);
	return hs >= 0 && cs >= 0 && (hs - cs > 1 || cs - hs > 1);
}

/* RSSI (positive dBm magnitude) averaged in eighths with weight 1/8. */
static inline uint16_t relay_rssi_avg_x8(uint16_t avg_x8, uint8_t rssi)
{
	return avg_x8 ? (7U * avg_x8 + 8U * rssi) / 8U : 8U * rssi;
}

struct relay_grant {
	uint8_t owner;
	uint16_t children;
};

struct relay_control {
	uint8_t direct_count, slot_ticks;
	uint16_t tps_max;
	uint8_t epoch;
	uint8_t repair_count;
	uint32_t session;
	uint32_t frame;
	uint32_t preamble_ticks;
	uint16_t members;
	uint16_t expected;
	uint16_t missing[RELAY_CACHE_FRAMES];
	uint16_t primary_valid;
	uint16_t backup_valid;
	uint8_t helpers[RELAY_MAX_TRACKERS];
	struct relay_grant grants[RELAY_REPAIR_MAX];
	uint16_t survey;
	uint8_t maintenance_owner;
	uint16_t watch_generation;
	uint8_t profile;
	uint8_t flags;
	uint16_t missing_extra[2];
	bool on_demand;
	uint8_t control_repeat_mode;
};
/* Keep the static control layout unchanged, including its hot-path offsets. */
#define relay_control_missing(c, age) (*((unsigned)(age) < 3U ? &(c)->missing[(age) % 3U] : &(c)->missing_extra[(age) - 3U]))

static inline unsigned relay_control_cache_frames(const struct relay_control *c)
{
	return c->on_demand ? RELAY_CACHE_MAX_FRAMES : RELAY_CACHE_FRAMES;
}

struct relay_layout_descriptor {
	uint8_t direct_count;
	uint16_t tps_max;
	uint32_t session;
	uint8_t epoch;
	uint8_t profile;
	uint16_t members;
	uint16_t expected;
	uint8_t slot_ticks;
	uint8_t repair_count;
	uint32_t origin_ticks;
	uint32_t activation_ticks;
	uint16_t frame_seed;
	uint8_t flags;
};

struct relay_repair_record {
	uint16_t capture_frame;
	struct relay_record_view source;
};

struct relay_repair {
	uint8_t helper;
	uint8_t epoch;
	uint32_t session;
	bool overflow;
	uint8_t count;
	struct relay_repair_record records[RELAY_MAX_CHILDREN];
};

#define RELAY_PEER_REPORT_ENTRIES 8U
#define RELAY_PEER_OBSERVATIONS 120U
struct relay_peer_observation {
	uint8_t child;
	uint16_t window_end;
	uint8_t expected;
	uint8_t heard;
	uint8_t rssi;
	uint8_t flags;
};

struct relay_peer_report {
	uint8_t helper;
	uint8_t epoch;
	uint32_t session;
	uint8_t count;
	struct relay_peer_observation entries[RELAY_PEER_REPORT_ENTRIES];
};

static inline uint16_t relay_get_be16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] << 8 | p[1]);
}

static inline uint32_t relay_get_be32(const uint8_t *p)
{
	return (uint32_t)relay_get_be16(p) << 16 | relay_get_be16(p + 2);
}

static inline void relay_put_be16(uint8_t *p, uint16_t value)
{
	p[0] = (uint8_t)(value >> 8);
	p[1] = (uint8_t)value;
}

static inline void relay_put_be32(uint8_t *p, uint32_t value)
{
	relay_put_be16(p, (uint16_t)(value >> 16));
	relay_put_be16(p + 2, (uint16_t)value);
}

RELAY_LAYOUT_INLINE bool relay_geometry_valid(
	uint8_t profile,
	uint16_t members,
	uint16_t expected,
	uint8_t slot_ticks,
	uint8_t repairs,
	const struct relay_geometry *g
)
{
	return g && profile == RELAY_PROFILE_120HZ && members && g->direct >= 1 && g->direct <= RELAY_MAX_TRACKERS
		&& g->repairs <= RELAY_REPAIR_MAX && (g->slot_ticks == 15U || g->slot_ticks == 16U) && g->tps
		&& g->denominator && repairs <= RELAY_REPAIR_MAX && relay_mask_count(members) <= g->direct
		&& !(expected & ~members) && slot_ticks == g->slot_ticks;
}

RELAY_LAYOUT_INLINE bool
relay_profile_valid(uint8_t profile, uint16_t members, uint16_t expected, uint8_t slot_ticks, uint8_t repairs)
{
	return relay_geometry_valid(profile, members, expected, slot_ticks, repairs, relay_geometry_current());
}

#define RELAY_CONTROL_NEXT_R_SHIFT 2U
#define RELAY_CONTROL_NEXT_R_MASK 0x1cU
#define RELAY_CONTROL_NEXT2_R_SHIFT 5U
#define RELAY_CONTROL_NEXT2_R_MASK 0xe0U
static inline uint8_t relay_control_next_repairs(const struct relay_control *c)
{
	return (c->flags & RELAY_CONTROL_NEXT_R_MASK) >> RELAY_CONTROL_NEXT_R_SHIFT;
}

static inline uint8_t relay_control_next2_repairs(const struct relay_control *c)
{
	return (c->flags & RELAY_CONTROL_NEXT2_R_MASK) >> RELAY_CONTROL_NEXT2_R_SHIFT;
}

/* Set during planning, before the immutable header CRC. The frozen path only
 * ORs the existing maintenance flag and retains both future announcements. */
static inline void relay_control_set_next_repairs(struct relay_control *c, uint8_t repairs)
{
	c->flags = (c->flags & ~RELAY_CONTROL_NEXT_R_MASK) | (repairs << RELAY_CONTROL_NEXT_R_SHIFT);
}

static inline void relay_control_set_next2_repairs(struct relay_control *c, uint8_t repairs)
{
	c->flags = (c->flags & ~RELAY_CONTROL_NEXT2_R_MASK) | (repairs << RELAY_CONTROL_NEXT2_R_SHIFT);
}

static inline bool relay_control_valid(const struct relay_control *c)
{
	struct relay_geometry g;
	if (!relay_geometry_make_control(
			&g,
			c->direct_count ? c->direct_count : relay_mask_count(c->members),
			c->repair_count,
			c->slot_ticks ? c->slot_ticks : RELAY_SLOT_TICKS,
			c->tps_max ? c->tps_max : relay_geometry_current()->tps,
			c->control_repeat_mode
		)
		|| !relay_geometry_valid(c->profile, c->members, c->expected, g.slot_ticks, c->repair_count, &g)
		|| (c->direct_count && relay_mask_count(c->members) != c->direct_count)
		|| (c->control_repeat_mode
			&& (relay_control_next_repairs(c) < 2U || relay_control_next2_repairs(c) < 2U
				|| relay_control_next_repairs(c) > 4U || relay_control_next2_repairs(c) > 4U))
		|| relay_control_next_repairs(c) > RELAY_REPAIR_MAX || relay_control_next2_repairs(c) > RELAY_REPAIR_MAX
		|| (c->maintenance_owner >= RELAY_MAX_TRACKERS && c->maintenance_owner != RELAY_OWNER_IDLE)
		|| ((c->primary_valid | c->backup_valid | c->survey) & ~c->members)) {
		return false;
	}
	for (uint8_t i = 0; i < RELAY_MAX_TRACKERS; i++) {
		uint16_t bit = (uint16_t)(1U << i);
		uint8_t primary = c->helpers[i] & 15U, backup = c->helpers[i] >> 4;
		if (((c->primary_valid & bit) && (primary == i || !(c->members & (1U << primary))))
		    || ((c->backup_valid & bit) && (backup == i || !(c->members & (1U << backup))))
		    || ((c->primary_valid & c->backup_valid & bit) && primary == backup)) {
			return false;
		}
	}
	for (uint8_t i = 0; i < relay_control_cache_frames(c); i++) {
		if (relay_control_missing(c, i) & ~c->members) {
			return false;
		}
	}
	for (uint8_t i = 0; i < RELAY_REPAIR_MAX; i++) {
		const struct relay_grant *g = &c->grants[i];
		if (i >= c->repair_count && g->owner != RELAY_OWNER_IDLE) {
			return false;
		}
		if (g->owner < RELAY_MAX_TRACKERS) {
			if (!(c->members & (1U << g->owner)) || (g->children & ~c->members)
			    || (g->children & (1U << g->owner))) {
				return false;
			}
		} else if (g->children || (g->owner != RELAY_OWNER_IDLE
			&& g->owner != RELAY_OWNER_MAINTENANCE && g->owner != RELAY_OWNER_LAYOUT)) {
			return false;
		}
	}
	return true;
}

enum relay_control_offset {
	RELAY_CONTROL_OFF_TYPE = 0,
	RELAY_CONTROL_OFF_VERSION = 1,
	RELAY_CONTROL_OFF_EPOCH = 2,
	RELAY_CONTROL_OFF_REPAIR_COUNT = 3,
	RELAY_CONTROL_OFF_SESSION = 4,
	RELAY_CONTROL_OFF_FRAME = 8,
	RELAY_CONTROL_OFF_PREAMBLE = 12,
	RELAY_CONTROL_OFF_MEMBERS = 16,
	RELAY_CONTROL_OFF_EXPECTED = 18,
	RELAY_CONTROL_OFF_MISSING = 20,
	RELAY_CONTROL_OFF_PRIMARY = 26,
	RELAY_CONTROL_OFF_BACKUP = 28,
	RELAY_CONTROL_OFF_HELPERS = 30,
	RELAY_CONTROL_OFF_OWNERS = 46,
	RELAY_CONTROL_OFF_OWNER_SECOND = 47,
	RELAY_CONTROL_OFF_CHILDREN = 51,
	RELAY_CONTROL_OFF_CHILDREN_LOW = 52,
	RELAY_CONTROL_OFF_SURVEY = 61,
	RELAY_CONTROL_OFF_MAINTENANCE = 63,
	RELAY_CONTROL_OFF_WATCH = 64,
	RELAY_CONTROL_OFF_PROFILE = 66,
	RELAY_CONTROL_OFF_FLAGS = 67,
	RELAY_CONTROL_OFF_RESERVED = 68,
	RELAY_CONTROL_OFF_RESERVED_SECOND = 69,
	RELAY_CONTROL_OFF_RESERVED_THIRD = 70,
	RELAY_CONTROL_OFF_CRC = 71,
};
_Static_assert(RELAY_CONTROL_OFF_OWNERS + RELAY_REPAIR_MAX == RELAY_CONTROL_OFF_CHILDREN, "grant owner layout");
_Static_assert(
	RELAY_CONTROL_OFF_CHILDREN + 2 * RELAY_REPAIR_MAX == RELAY_CONTROL_OFF_SURVEY,
	"grant children layout"
);
_Static_assert(RELAY_CONTROL_OFF_HELPERS + RELAY_MAX_TRACKERS == RELAY_CONTROL_OFF_OWNERS, "helper layout");
_Static_assert(RELAY_CONTROL_OFF_CRC + 1 == RELAY_CONTROL_LEN, "control wire size");

static inline size_t relay_control_encode(uint8_t out[RELAY_CONTROL_LEN], const struct relay_control *c)
{
	memset(out, 0, RELAY_CONTROL_LEN);
	out[RELAY_CONTROL_OFF_TYPE] = RELAY_CONTROL;
	out[RELAY_CONTROL_OFF_VERSION] = RELAY_PROTOCOL_VERSION;
	out[RELAY_CONTROL_OFF_EPOCH] = c->epoch;
	out[RELAY_CONTROL_OFF_REPAIR_COUNT]
		= (c->on_demand ? 0x80U : 0U) | c->repair_count | ((relay_mask_count(c->members) - 1U) << 3);
	relay_put_be32(out + RELAY_CONTROL_OFF_SESSION, c->session);
	relay_put_be32(out + RELAY_CONTROL_OFF_FRAME, c->frame);
	relay_put_be32(out + RELAY_CONTROL_OFF_PREAMBLE, c->preamble_ticks);
	relay_put_be16(out + RELAY_CONTROL_OFF_MEMBERS, c->members);
	relay_put_be16(out + RELAY_CONTROL_OFF_EXPECTED, c->expected);
	for (uint8_t i = 0; i < RELAY_CACHE_FRAMES; i++) {
		relay_put_be16(out + RELAY_CONTROL_OFF_MISSING + 2U * i, relay_control_missing(c, i));
	}
	relay_put_be16(out + RELAY_CONTROL_OFF_PRIMARY, c->primary_valid);
	relay_put_be16(out + RELAY_CONTROL_OFF_BACKUP, c->backup_valid);
	memcpy(out + RELAY_CONTROL_OFF_HELPERS, c->helpers, RELAY_MAX_TRACKERS);
	for (uint8_t i = 0; i < RELAY_REPAIR_MAX; i++) {
		out[RELAY_CONTROL_OFF_OWNERS + i] = c->grants[i].owner;
		relay_put_be16(out + RELAY_CONTROL_OFF_CHILDREN + 2U * i, c->grants[i].children);
	}
	relay_put_be16(out + RELAY_CONTROL_OFF_SURVEY, c->survey);
	out[RELAY_CONTROL_OFF_MAINTENANCE] = c->maintenance_owner;
	relay_put_be16(out + RELAY_CONTROL_OFF_WATCH, c->watch_generation);
	out[RELAY_CONTROL_OFF_PROFILE] = c->profile;
	out[RELAY_CONTROL_OFF_FLAGS] = c->flags;
	out[68] = (c->slot_ticks ? c->slot_ticks : RELAY_SLOT_TICKS) | (c->control_repeat_mode << 6);
	relay_put_be16(out + 69, c->tps_max ? c->tps_max : relay_geometry_current()->tps);
	if (c->on_demand) {
		relay_put_be16(out + 63, relay_control_missing(c, 3));
		relay_put_be16(out + 65, relay_control_missing(c, 4));
		out[68] = (c->slot_ticks == 15U) | ((c->maintenance_owner < 16U ? c->maintenance_owner : 16U) << 1)
				| (c->control_repeat_mode << 6);
	}
	out[RELAY_CONTROL_OFF_CRC] = relay_crc8(7, out, 71);
	return RELAY_CONTROL_LEN;
}

/* Patch a pre-encoded frame after the mask freeze. The caller precomputes
 * CRCs for the immutable prefix and gaps while planning. CRC is linear. */
struct relay_control_crc_plan {
	uint8_t prefix, watch, layout, geometry;
};

static inline struct relay_control_crc_plan relay_control_plan_crc(const uint8_t *out)
{
	/* Fold immutable watch bytes into the static suffix before freeze. */
	uint8_t tail = relay_crc8(0, out + 64, 3);
	uint8_t zero = 0;
	tail = relay_crc8(tail, &zero, 1);
	tail = relay_crc8(tail, out + 68, 3);
	uint8_t watch = relay_crc8(0, out + 26, 20);
	const uint8_t geometry[3] = {(out[68] & 0xc1U) | 32U, out[69], out[70]};
	if (!(out[3] & 0x80U)) {
		const uint8_t zeros[25] = {0};
		tail ^= relay_crc8(watch, zeros, sizeof(zeros));
	}
	return (
		struct relay_control_crc_plan
	){relay_crc8(7, out, 20), watch, relay_crc8(0, out + 64, 3), (out[3] & 0x80U) ? relay_crc8(0, geometry, 3) : tail};
}

#if defined(__GNUC__) && !defined(__clang__)
/* Frozen encoder: round 11 linked model and DWT evidence. Retain pending A/B. */
__attribute__((optimize("O3")))
#endif
static inline void relay_control_finish_static_encode(
	uint8_t *out,
	const struct relay_control *c,
	struct relay_control_crc_plan crc,
	bool write_grants
)
{
	static const uint8_t shift7[256]
		= {0x0,  0xdf, 0xb9, 0x66, 0x75, 0xaa, 0xcc, 0x13, 0xea, 0x35, 0x53, 0x8c, 0x9f, 0x40, 0x26, 0xf9, 0xd3, 0xc,
		   0x6a, 0xb5, 0xa6, 0x79, 0x1f, 0xc0, 0x39, 0xe6, 0x80, 0x5f, 0x4c, 0x93, 0xf5, 0x2a, 0xa1, 0x7e, 0x18, 0xc7,
		   0xd4, 0xb,  0x6d, 0xb2, 0x4b, 0x94, 0xf2, 0x2d, 0x3e, 0xe1, 0x87, 0x58, 0x72, 0xad, 0xcb, 0x14, 0x7,  0xd8,
		   0xbe, 0x61, 0x98, 0x47, 0x21, 0xfe, 0xed, 0x32, 0x54, 0x8b, 0x45, 0x9a, 0xfc, 0x23, 0x30, 0xef, 0x89, 0x56,
		   0xaf, 0x70, 0x16, 0xc9, 0xda, 0x5,  0x63, 0xbc, 0x96, 0x49, 0x2f, 0xf0, 0xe3, 0x3c, 0x5a, 0x85, 0x7c, 0xa3,
		   0xc5, 0x1a, 0x9,  0xd6, 0xb0, 0x6f, 0xe4, 0x3b, 0x5d, 0x82, 0x91, 0x4e, 0x28, 0xf7, 0xe,  0xd1, 0xb7, 0x68,
		   0x7b, 0xa4, 0xc2, 0x1d, 0x37, 0xe8, 0x8e, 0x51, 0x42, 0x9d, 0xfb, 0x24, 0xdd, 0x2,  0x64, 0xbb, 0xa8, 0x77,
		   0x11, 0xce, 0x8a, 0x55, 0x33, 0xec, 0xff, 0x20, 0x46, 0x99, 0x60, 0xbf, 0xd9, 0x6,  0x15, 0xca, 0xac, 0x73,
		   0x59, 0x86, 0xe0, 0x3f, 0x2c, 0xf3, 0x95, 0x4a, 0xb3, 0x6c, 0xa,  0xd5, 0xc6, 0x19, 0x7f, 0xa0, 0x2b, 0xf4,
		   0x92, 0x4d, 0x5e, 0x81, 0xe7, 0x38, 0xc1, 0x1e, 0x78, 0xa7, 0xb4, 0x6b, 0xd,  0xd2, 0xf8, 0x27, 0x41, 0x9e,
		   0x8d, 0x52, 0x34, 0xeb, 0x12, 0xcd, 0xab, 0x74, 0x67, 0xb8, 0xde, 0x1,  0xcf, 0x10, 0x76, 0xa9, 0xba, 0x65,
		   0x3,  0xdc, 0x25, 0xfa, 0x9c, 0x43, 0x50, 0x8f, 0xe9, 0x36, 0x1c, 0xc3, 0xa5, 0x7a, 0x69, 0xb6, 0xd0, 0xf,
		   0xf6, 0x29, 0x4f, 0x90, 0x83, 0x5c, 0x3a, 0xe5, 0x6e, 0xb1, 0xd7, 0x8,  0x1b, 0xc4, 0xa2, 0x7d, 0x84, 0x5b,
		   0x3d, 0xe2, 0xf1, 0x2e, 0x48, 0x97, 0xbd, 0x62, 0x4,  0xdb, 0xc8, 0x17, 0x71, 0xae, 0x57, 0x88, 0xee, 0x31,
		   0x22, 0xfd, 0x9b, 0x44};

	static const uint8_t shift4[256]
		= {0x0,  0x16, 0x2c, 0x3a, 0x58, 0x4e, 0x74, 0x62, 0xb0, 0xa6, 0x9c, 0x8a, 0xe8, 0xfe, 0xc4, 0xd2, 0x67, 0x71,
		   0x4b, 0x5d, 0x3f, 0x29, 0x13, 0x5,  0xd7, 0xc1, 0xfb, 0xed, 0x8f, 0x99, 0xa3, 0xb5, 0xce, 0xd8, 0xe2, 0xf4,
		   0x96, 0x80, 0xba, 0xac, 0x7e, 0x68, 0x52, 0x44, 0x26, 0x30, 0xa,  0x1c, 0xa9, 0xbf, 0x85, 0x93, 0xf1, 0xe7,
		   0xdd, 0xcb, 0x19, 0xf,  0x35, 0x23, 0x41, 0x57, 0x6d, 0x7b, 0x9b, 0x8d, 0xb7, 0xa1, 0xc3, 0xd5, 0xef, 0xf9,
		   0x2b, 0x3d, 0x7,  0x11, 0x73, 0x65, 0x5f, 0x49, 0xfc, 0xea, 0xd0, 0xc6, 0xa4, 0xb2, 0x88, 0x9e, 0x4c, 0x5a,
		   0x60, 0x76, 0x14, 0x2,  0x38, 0x2e, 0x55, 0x43, 0x79, 0x6f, 0xd,  0x1b, 0x21, 0x37, 0xe5, 0xf3, 0xc9, 0xdf,
		   0xbd, 0xab, 0x91, 0x87, 0x32, 0x24, 0x1e, 0x8,  0x6a, 0x7c, 0x46, 0x50, 0x82, 0x94, 0xae, 0xb8, 0xda, 0xcc,
		   0xf6, 0xe0, 0x31, 0x27, 0x1d, 0xb,  0x69, 0x7f, 0x45, 0x53, 0x81, 0x97, 0xad, 0xbb, 0xd9, 0xcf, 0xf5, 0xe3,
		   0x56, 0x40, 0x7a, 0x6c, 0xe,  0x18, 0x22, 0x34, 0xe6, 0xf0, 0xca, 0xdc, 0xbe, 0xa8, 0x92, 0x84, 0xff, 0xe9,
		   0xd3, 0xc5, 0xa7, 0xb1, 0x8b, 0x9d, 0x4f, 0x59, 0x63, 0x75, 0x17, 0x1,  0x3b, 0x2d, 0x98, 0x8e, 0xb4, 0xa2,
		   0xc0, 0xd6, 0xec, 0xfa, 0x28, 0x3e, 0x4,  0x12, 0x70, 0x66, 0x5c, 0x4a, 0xaa, 0xbc, 0x86, 0x90, 0xf2, 0xe4,
		   0xde, 0xc8, 0x1a, 0xc,  0x36, 0x20, 0x42, 0x54, 0x6e, 0x78, 0xcd, 0xdb, 0xe1, 0xf7, 0x95, 0x83, 0xb9, 0xaf,
		   0x7d, 0x6b, 0x51, 0x47, 0x25, 0x33, 0x9,  0x1f, 0x64, 0x72, 0x48, 0x5e, 0x3c, 0x2a, 0x10, 0x6,  0xd4, 0xc2,
		   0xf8, 0xee, 0x8c, 0x9a, 0xa0, 0xb6, 0x3,  0x15, 0x2f, 0x39, 0x5b, 0x4d, 0x77, 0x61, 0xb3, 0xa5, 0x9f, 0x89,
		   0xeb, 0xfd, 0xc7, 0xd1};

	static const uint8_t shift20[256] = {
		0x00, 0x2c, 0x58, 0x74, 0xb0, 0x9c, 0xe8, 0xc4, 0x67, 0x4b, 0x3f, 0x13, 0xd7, 0xfb, 0x8f, 0xa3,
		0xce, 0xe2, 0x96, 0xba, 0x7e, 0x52, 0x26, 0x0a, 0xa9, 0x85, 0xf1, 0xdd, 0x19, 0x35, 0x41, 0x6d,
		0x9b, 0xb7, 0xc3, 0xef, 0x2b, 0x07, 0x73, 0x5f, 0xfc, 0xd0, 0xa4, 0x88, 0x4c, 0x60, 0x14, 0x38,
		0x55, 0x79, 0x0d, 0x21, 0xe5, 0xc9, 0xbd, 0x91, 0x32, 0x1e, 0x6a, 0x46, 0x82, 0xae, 0xda, 0xf6,
		0x31, 0x1d, 0x69, 0x45, 0x81, 0xad, 0xd9, 0xf5, 0x56, 0x7a, 0x0e, 0x22, 0xe6, 0xca, 0xbe, 0x92,
		0xff, 0xd3, 0xa7, 0x8b, 0x4f, 0x63, 0x17, 0x3b, 0x98, 0xb4, 0xc0, 0xec, 0x28, 0x04, 0x70, 0x5c,
		0xaa, 0x86, 0xf2, 0xde, 0x1a, 0x36, 0x42, 0x6e, 0xcd, 0xe1, 0x95, 0xb9, 0x7d, 0x51, 0x25, 0x09,
		0x64, 0x48, 0x3c, 0x10, 0xd4, 0xf8, 0x8c, 0xa0, 0x03, 0x2f, 0x5b, 0x77, 0xb3, 0x9f, 0xeb, 0xc7,
		0x62, 0x4e, 0x3a, 0x16, 0xd2, 0xfe, 0x8a, 0xa6, 0x05, 0x29, 0x5d, 0x71, 0xb5, 0x99, 0xed, 0xc1,
		0xac, 0x80, 0xf4, 0xd8, 0x1c, 0x30, 0x44, 0x68, 0xcb, 0xe7, 0x93, 0xbf, 0x7b, 0x57, 0x23, 0x0f,
		0xf9, 0xd5, 0xa1, 0x8d, 0x49, 0x65, 0x11, 0x3d, 0x9e, 0xb2, 0xc6, 0xea, 0x2e, 0x02, 0x76, 0x5a,
		0x37, 0x1b, 0x6f, 0x43, 0x87, 0xab, 0xdf, 0xf3, 0x50, 0x7c, 0x08, 0x24, 0xe0, 0xcc, 0xb8, 0x94,
		0x53, 0x7f, 0x0b, 0x27, 0xe3, 0xcf, 0xbb, 0x97, 0x34, 0x18, 0x6c, 0x40, 0x84, 0xa8, 0xdc, 0xf0,
		0x9d, 0xb1, 0xc5, 0xe9, 0x2d, 0x01, 0x75, 0x59, 0xfa, 0xd6, 0xa2, 0x8e, 0x4a, 0x66, 0x12, 0x3e,
		0xc8, 0xe4, 0x90, 0xbc, 0x78, 0x54, 0x20, 0x0c, 0xaf, 0x83, 0xf7, 0xdb, 0x1f, 0x33, 0x47, 0x6b,
		0x06, 0x2a, 0x5e, 0x72, 0xb6, 0x9a, 0xee, 0xc2, 0x61, 0x4d, 0x39, 0x15, 0xd1, 0xfd, 0x89, 0xa5,
	};
	for (unsigned age = 0; age < RELAY_CACHE_FRAMES; age++) {
		relay_put_be16(out + RELAY_CONTROL_OFF_MISSING + 2U * age, relay_control_missing(c, age));
	}
	for (unsigned i = 0; write_grants && i < RELAY_REPAIR_MAX; i++) {
		out[RELAY_CONTROL_OFF_OWNERS + i] = c->grants[i].owner;
		relay_put_be16(out + RELAY_CONTROL_OFF_CHILDREN + 2U * i, c->grants[i].children);
	}
	relay_put_be16(out + RELAY_CONTROL_OFF_SURVEY, c->survey);
	out[RELAY_CONTROL_OFF_MAINTENANCE] = c->maintenance_owner;
	out[RELAY_CONTROL_OFF_FLAGS] = c->flags;
	uint8_t v = relay_crc8(crc.prefix, out + RELAY_CONTROL_OFF_MISSING, 6);
	v = shift20[v];
	v = relay_crc8(v, out + RELAY_CONTROL_OFF_OWNERS, 18);
	out[RELAY_CONTROL_OFF_CRC] = shift7[v] ^ shift4[c->flags] ^ crc.geometry;
}

static inline void relay_control_finish_encode(
	uint8_t *out,
	const struct relay_control *c,
	struct relay_control_crc_plan crc,
	bool write_grants
)
{
	if (!c->on_demand) {
		relay_control_finish_static_encode(out, c, crc, write_grants);
		return;
	}

	static const uint8_t shift3[256] = {
		0x00, 0x6b, 0xd6, 0xbd, 0xab, 0xc0, 0x7d, 0x16, 0x51, 0x3a, 0x87, 0xec, 0xfa, 0x91, 0x2c, 0x47, 0xa2, 0xc9,
		0x74, 0x1f, 0x09, 0x62, 0xdf, 0xb4, 0xf3, 0x98, 0x25, 0x4e, 0x58, 0x33, 0x8e, 0xe5, 0x43, 0x28, 0x95, 0xfe,
		0xe8, 0x83, 0x3e, 0x55, 0x12, 0x79, 0xc4, 0xaf, 0xb9, 0xd2, 0x6f, 0x04, 0xe1, 0x8a, 0x37, 0x5c, 0x4a, 0x21,
		0x9c, 0xf7, 0xb0, 0xdb, 0x66, 0x0d, 0x1b, 0x70, 0xcd, 0xa6, 0x86, 0xed, 0x50, 0x3b, 0x2d, 0x46, 0xfb, 0x90,
		0xd7, 0xbc, 0x01, 0x6a, 0x7c, 0x17, 0xaa, 0xc1, 0x24, 0x4f, 0xf2, 0x99, 0x8f, 0xe4, 0x59, 0x32, 0x75, 0x1e,
		0xa3, 0xc8, 0xde, 0xb5, 0x08, 0x63, 0xc5, 0xae, 0x13, 0x78, 0x6e, 0x05, 0xb8, 0xd3, 0x94, 0xff, 0x42, 0x29,
		0x3f, 0x54, 0xe9, 0x82, 0x67, 0x0c, 0xb1, 0xda, 0xcc, 0xa7, 0x1a, 0x71, 0x36, 0x5d, 0xe0, 0x8b, 0x9d, 0xf6,
		0x4b, 0x20, 0x0b, 0x60, 0xdd, 0xb6, 0xa0, 0xcb, 0x76, 0x1d, 0x5a, 0x31, 0x8c, 0xe7, 0xf1, 0x9a, 0x27, 0x4c,
		0xa9, 0xc2, 0x7f, 0x14, 0x02, 0x69, 0xd4, 0xbf, 0xf8, 0x93, 0x2e, 0x45, 0x53, 0x38, 0x85, 0xee, 0x48, 0x23,
		0x9e, 0xf5, 0xe3, 0x88, 0x35, 0x5e, 0x19, 0x72, 0xcf, 0xa4, 0xb2, 0xd9, 0x64, 0x0f, 0xea, 0x81, 0x3c, 0x57,
		0x41, 0x2a, 0x97, 0xfc, 0xbb, 0xd0, 0x6d, 0x06, 0x10, 0x7b, 0xc6, 0xad, 0x8d, 0xe6, 0x5b, 0x30, 0x26, 0x4d,
		0xf0, 0x9b, 0xdc, 0xb7, 0x0a, 0x61, 0x77, 0x1c, 0xa1, 0xca, 0x2f, 0x44, 0xf9, 0x92, 0x84, 0xef, 0x52, 0x39,
		0x7e, 0x15, 0xa8, 0xc3, 0xd5, 0xbe, 0x03, 0x68, 0xce, 0xa5, 0x18, 0x73, 0x65, 0x0e, 0xb3, 0xd8, 0x9f, 0xf4,
		0x49, 0x22, 0x34, 0x5f, 0xe2, 0x89, 0x6c, 0x07, 0xba, 0xd1, 0xc7, 0xac, 0x11, 0x7a, 0x3d, 0x56, 0xeb, 0x80,
		0x96, 0xfd, 0x40, 0x2b,
	};
	static const uint8_t shift20[256] = {
		0x00, 0x2c, 0x58, 0x74, 0xb0, 0x9c, 0xe8, 0xc4, 0x67, 0x4b, 0x3f, 0x13, 0xd7, 0xfb, 0x8f, 0xa3, 0xce, 0xe2,
		0x96, 0xba, 0x7e, 0x52, 0x26, 0x0a, 0xa9, 0x85, 0xf1, 0xdd, 0x19, 0x35, 0x41, 0x6d, 0x9b, 0xb7, 0xc3, 0xef,
		0x2b, 0x07, 0x73, 0x5f, 0xfc, 0xd0, 0xa4, 0x88, 0x4c, 0x60, 0x14, 0x38, 0x55, 0x79, 0x0d, 0x21, 0xe5, 0xc9,
		0xbd, 0x91, 0x32, 0x1e, 0x6a, 0x46, 0x82, 0xae, 0xda, 0xf6, 0x31, 0x1d, 0x69, 0x45, 0x81, 0xad, 0xd9, 0xf5,
		0x56, 0x7a, 0x0e, 0x22, 0xe6, 0xca, 0xbe, 0x92, 0xff, 0xd3, 0xa7, 0x8b, 0x4f, 0x63, 0x17, 0x3b, 0x98, 0xb4,
		0xc0, 0xec, 0x28, 0x04, 0x70, 0x5c, 0xaa, 0x86, 0xf2, 0xde, 0x1a, 0x36, 0x42, 0x6e, 0xcd, 0xe1, 0x95, 0xb9,
		0x7d, 0x51, 0x25, 0x09, 0x64, 0x48, 0x3c, 0x10, 0xd4, 0xf8, 0x8c, 0xa0, 0x03, 0x2f, 0x5b, 0x77, 0xb3, 0x9f,
		0xeb, 0xc7, 0x62, 0x4e, 0x3a, 0x16, 0xd2, 0xfe, 0x8a, 0xa6, 0x05, 0x29, 0x5d, 0x71, 0xb5, 0x99, 0xed, 0xc1,
		0xac, 0x80, 0xf4, 0xd8, 0x1c, 0x30, 0x44, 0x68, 0xcb, 0xe7, 0x93, 0xbf, 0x7b, 0x57, 0x23, 0x0f, 0xf9, 0xd5,
		0xa1, 0x8d, 0x49, 0x65, 0x11, 0x3d, 0x9e, 0xb2, 0xc6, 0xea, 0x2e, 0x02, 0x76, 0x5a, 0x37, 0x1b, 0x6f, 0x43,
		0x87, 0xab, 0xdf, 0xf3, 0x50, 0x7c, 0x08, 0x24, 0xe0, 0xcc, 0xb8, 0x94, 0x53, 0x7f, 0x0b, 0x27, 0xe3, 0xcf,
		0xbb, 0x97, 0x34, 0x18, 0x6c, 0x40, 0x84, 0xa8, 0xdc, 0xf0, 0x9d, 0xb1, 0xc5, 0xe9, 0x2d, 0x01, 0x75, 0x59,
		0xfa, 0xd6, 0xa2, 0x8e, 0x4a, 0x66, 0x12, 0x3e, 0xc8, 0xe4, 0x90, 0xbc, 0x78, 0x54, 0x20, 0x0c, 0xaf, 0x83,
		0xf7, 0xdb, 0x1f, 0x33, 0x47, 0x6b, 0x06, 0x2a, 0x5e, 0x72, 0xb6, 0x9a, 0xee, 0xc2, 0x61, 0x4d, 0x39, 0x15,
		0xd1, 0xfd, 0x89, 0xa5,
	};
	for (unsigned age = 0; age < RELAY_CACHE_FRAMES; age++) {
		relay_put_be16(out + RELAY_CONTROL_OFF_MISSING + 2U * age, relay_control_missing(c, age));
	}
	for (unsigned i = 0; write_grants && i < RELAY_REPAIR_MAX; i++) {
		out[RELAY_CONTROL_OFF_OWNERS + i] = c->grants[i].owner;
		relay_put_be16(out + RELAY_CONTROL_OFF_CHILDREN + 2U * i, c->grants[i].children);
	}
	relay_put_be16(out + RELAY_CONTROL_OFF_SURVEY, c->survey);
	out[RELAY_CONTROL_OFF_FLAGS] = c->flags;
	if (c->on_demand) {
		relay_put_be16(out + 63, relay_control_missing(c, 3));
		relay_put_be16(out + 65, relay_control_missing(c, 4));
		unsigned owner = c->maintenance_owner < 16U ? c->maintenance_owner : 16U;
		out[68] = (c->slot_ticks == 15U) | (owner << 1) | (c->control_repeat_mode << 6);
		uint8_t v = relay_crc8(crc.prefix, out + 20, 6);
		v = shift20[v] ^ crc.watch;
		v = relay_crc8(v, out + 46, 22);
		out[71] = shift3[v] ^ crc.geometry ^ shift3[(owner ^ 16U) << 1];
		return;
	}
	out[RELAY_CONTROL_OFF_MAINTENANCE] = c->maintenance_owner;

	uint8_t v = relay_crc8(crc.prefix, out + RELAY_CONTROL_OFF_MISSING, 6);
	v = shift20[v] ^ crc.watch;
	v = relay_crc8(v, out + RELAY_CONTROL_OFF_OWNERS, 18);
	v = shift3[v] ^ crc.layout;
	v = relay_crc8(v, out + RELAY_CONTROL_OFF_FLAGS, 1);
	out[RELAY_CONTROL_OFF_CRC] = shift3[v] ^ crc.geometry;
}

static inline bool relay_control_decode(const uint8_t *in, size_t length, struct relay_control *c)
{
	if (length != RELAY_CONTROL_LEN
		|| (in[RELAY_CONTROL_OFF_TYPE] != RELAY_CONTROL && in[RELAY_CONTROL_OFF_TYPE] != 0xecU)
		|| in[RELAY_CONTROL_OFF_VERSION] != RELAY_PROTOCOL_VERSION
		|| (!(in[3] & 0x80U) && (in[68] & 63U) != 15U && (in[68] & 63U) != 16U)
		|| ((in[3] & 0x80U) && ((in[68] & 63U) >> 1) > 16U) || !relay_get_be16(in + 69)
		|| relay_get_be16(in + 69) > 32768U || in[RELAY_CONTROL_OFF_CRC] != relay_crc8(7, in, 71)) {
		return false;
	}
	c->direct_count = ((in[3] >> 3) & 15U) + 1U;
	c->slot_ticks = in[68] & 63U;
	c->control_repeat_mode = in[68] >> 6;
	c->tps_max = relay_get_be16(in + 69);
	c->epoch = in[RELAY_CONTROL_OFF_EPOCH];
	c->repair_count = in[RELAY_CONTROL_OFF_REPAIR_COUNT] & 7U;
	c->session = relay_get_be32(in + RELAY_CONTROL_OFF_SESSION);
	c->frame = relay_get_be32(in + RELAY_CONTROL_OFF_FRAME);
	c->preamble_ticks = relay_get_be32(in + RELAY_CONTROL_OFF_PREAMBLE);
	c->members = relay_get_be16(in + RELAY_CONTROL_OFF_MEMBERS);
	c->expected = relay_get_be16(in + RELAY_CONTROL_OFF_EXPECTED);
	for (uint8_t i = 0; i < RELAY_CACHE_FRAMES; i++) {
		relay_control_missing(c, i) = relay_get_be16(in + RELAY_CONTROL_OFF_MISSING + 2U * i);
	}
	c->primary_valid = relay_get_be16(in + RELAY_CONTROL_OFF_PRIMARY);
	c->backup_valid = relay_get_be16(in + RELAY_CONTROL_OFF_BACKUP);
	memcpy(c->helpers, in + RELAY_CONTROL_OFF_HELPERS, RELAY_MAX_TRACKERS);
	for (uint8_t i = 0; i < RELAY_REPAIR_MAX; i++) {
		c->grants[i].owner = in[RELAY_CONTROL_OFF_OWNERS + i];
		c->grants[i].children = relay_get_be16(in + RELAY_CONTROL_OFF_CHILDREN + 2U * i);
	}
	c->survey = relay_get_be16(in + RELAY_CONTROL_OFF_SURVEY);
	c->maintenance_owner = in[RELAY_CONTROL_OFF_MAINTENANCE];
	c->watch_generation = relay_get_be16(in + RELAY_CONTROL_OFF_WATCH);
	c->profile = in[RELAY_CONTROL_OFF_PROFILE];
	c->flags = in[RELAY_CONTROL_OFF_FLAGS];
	c->on_demand = (in[3] & 0x80U) != 0;
	if (in[0] == 0xecU) {
		if (!c->control_repeat_mode) {
			return false;
		}
		c->preamble_ticks -= relay_control_repeat_offset(
			c->direct_count,
			c->repair_count,
			(in[3] & 0x80U) ? ((in[68] & 1U) ? 15U : 16U) : c->slot_ticks,
			c->control_repeat_mode
		);
	}
	relay_control_missing(c, 3) = relay_control_missing(c, 4) = 0;
	if (c->on_demand) {
		relay_control_missing(c, 3) = relay_get_be16(in + 63);
		relay_control_missing(c, 4) = relay_get_be16(in + 65);
		c->slot_ticks = (in[68] & 1U) ? 15 : 16;
		c->profile = RELAY_PROFILE_120HZ;
		c->watch_generation = 0;
		c->maintenance_owner = ((in[68] & 63U) >> 1) < 16U ? (in[68] & 63U) >> 1 : RELAY_OWNER_IDLE;
	}

	return relay_control_valid(c);
}

/* Records are picked by the tracker's own cache; only the frame bound is checked. */
static inline size_t relay_repair_encode(uint8_t out[RELAY_FRAME_MAX], const struct relay_repair *r)
{
	size_t pos = 8;
	for (uint8_t i = 0; i < r->count; i++) {
		const struct relay_repair_record *record = &r->records[i];
		uint8_t n = record->source.length;
		if (pos + 4U + n > RELAY_FRAME_MAX) {
			return 0;
		}
		relay_put_be16(out + pos, record->capture_frame);
		out[pos + 2] = n;
		out[pos + 3] = record->source.rssi;
		memcpy(out + pos + 4, record->source.frame, n);
		pos += 4U + n;
	}
	out[0] = RELAY_REPAIR;
	out[1] = r->helper;
	out[2] = r->epoch;
	out[3] = r->count | (r->overflow ? 0x80U : 0U);
	relay_put_be32(out + 4, r->session);
	return pos;
}

static inline bool relay_repair_decode(const uint8_t *in, size_t length, struct relay_repair *r)
{
	if (length < 8 || length > RELAY_FRAME_MAX || in[0] != RELAY_REPAIR
	    || in[1] >= RELAY_MAX_TRACKERS || (in[3] & 0x7CU) || !(in[3] & 3U)) {
		return false;
	}
	r->helper = in[1];
	r->epoch = in[2];
	r->count = in[3] & 3U;
	r->overflow = (in[3] & 0x80U) != 0;
	r->session = relay_get_be32(in + 4);
	size_t pos = 8;
	for (uint8_t i = 0; i < r->count; i++) {
		if (pos + 4U > length) {
			return false;
		}
		struct relay_repair_record *record = &r->records[i];
		record->capture_frame = relay_get_be16(in + pos);
		record->source.length = in[pos + 2];
		record->source.rssi = in[pos + 3];
		record->source.frame = in + pos + 4;
		uint8_t n = record->source.length;
		if (n < 2 || n > RELAY_DIRECT_MAX || pos + 4U + n > length
		    || record->source.frame[1] == r->helper
		    || !relay_frame_is_pose(record->source.frame, n, record->source.frame[1])) {
			return false;
		}
		for (uint8_t j = 0; j < i; j++) {
			if (r->records[j].source.frame[1] == record->source.frame[1]
			    && r->records[j].capture_frame == record->capture_frame) {
				return false;
			}
		}
		pos += 4U + n;
	}
	return pos == length;
}

static inline bool relay_capture_age(uint16_t current, uint16_t captured)
{
	return (uint16_t)(current - captured) < RELAY_CACHE_FRAMES;
}

static inline size_t relay_layout_encode(uint8_t out[RELAY_LAYOUT_LEN], const struct relay_layout_descriptor *d)
{
	struct relay_geometry geometry;
	uint8_t direct = d->direct_count ? d->direct_count : relay_mask_count(d->members);
	uint16_t cap = d->tps_max ? d->tps_max : relay_geometry_current()->tps;
	if (d->flags || !relay_geometry_make(&geometry, direct, d->repair_count, d->slot_ticks, cap)
		|| !relay_geometry_valid(d->profile, d->members, d->expected, d->slot_ticks, d->repair_count, &geometry)
		|| direct != relay_mask_count(d->members)) {
		return 0;
	}
	out[0] = RELAY_LAYOUT;
	out[1] = RELAY_PROTOCOL_VERSION;
	relay_put_be32(out + 2, d->session);
	out[6] = d->epoch;
	out[7] = d->profile;
	relay_put_be16(out + 8, d->members);
	relay_put_be16(out + 10, d->expected);
	out[12] = d->slot_ticks;
	out[13] = d->repair_count;
	relay_put_be32(out + 14, d->origin_ticks);
	relay_put_be32(out + 18, d->activation_ticks);
	relay_put_be16(out + 22, d->frame_seed);
	out[24] = d->flags;
	out[25] = direct;
	relay_put_be16(out + 26, cap);
	out[28] = 0;
	out[29] = relay_crc8(7, out, 29);
	return RELAY_LAYOUT_LEN;
}

static inline bool relay_layout_decode(const uint8_t *in, size_t length, struct relay_layout_descriptor *d)
{
	if (length != RELAY_LAYOUT_LEN || in[0] != RELAY_LAYOUT || in[1] != RELAY_PROTOCOL_VERSION || in[24] || in[28]
		|| in[29] != relay_crc8(7, in, 29)) {
		return false;
	}
	d->direct_count = in[25];
	d->tps_max = relay_get_be16(in + 26);
	d->session = relay_get_be32(in + 2);
	d->epoch = in[6];
	d->profile = in[7];
	d->members = relay_get_be16(in + 8);
	d->expected = relay_get_be16(in + 10);
	d->slot_ticks = in[12];
	d->repair_count = in[13];
	d->origin_ticks = relay_get_be32(in + 14);
	d->activation_ticks = relay_get_be32(in + 18);
	d->frame_seed = relay_get_be16(in + 22);
	d->flags = in[24];
	struct relay_geometry geometry;
	return relay_geometry_make(&geometry, d->direct_count, d->repair_count, d->slot_ticks, d->tps_max)
		&& d->direct_count == relay_mask_count(d->members)
		&& relay_geometry_valid(d->profile, d->members, d->expected, d->slot_ticks, d->repair_count, &geometry);
}

static inline size_t relay_peer_report_encode(uint8_t out[RELAY_FRAME_MAX], const struct relay_peer_report *r)
{
	out[0] = RELAY_PEER_REPORT;
	out[1] = r->helper;
	out[2] = r->epoch;
	out[3] = r->count;
	relay_put_be32(out + 4, r->session);
	for (uint8_t i = 0; i < r->count; i++) {
		const struct relay_peer_observation *e = &r->entries[i];
		uint8_t *p = out + 8 + 7U * i;
		p[0] = e->child;
		relay_put_be16(p + 1, e->window_end);
		p[3] = e->expected;
		p[4] = e->heard;
		p[5] = e->rssi;
		p[6] = e->flags;
	}
	size_t length = 9U + 7U * r->count;
	out[length - 1] = relay_crc8(7, out, length - 1);
	return length;
}

static inline bool relay_peer_report_decode(const uint8_t *in, size_t length, struct relay_peer_report *r)
{
	if (length < 9 || in[0] != RELAY_PEER_REPORT || in[1] >= RELAY_MAX_TRACKERS
	    || !in[3] || in[3] > RELAY_PEER_REPORT_ENTRIES || length != 9U + 7U * in[3]
	    || in[length - 1] != relay_crc8(7, in, length - 1)) {
		return false;
	}
	r->helper = in[1];
	r->epoch = in[2];
	r->count = in[3];
	r->session = relay_get_be32(in + 4);
	uint16_t seen = 0;
	for (uint8_t i = 0; i < r->count; i++) {
		const uint8_t *p = in + 8 + 7U * i;
		if (p[0] >= RELAY_MAX_TRACKERS || p[0] == r->helper || (seen & (1U << p[0]))
		    || !p[3] || p[3] > RELAY_PEER_OBSERVATIONS || p[4] > p[3] || p[6]) {
			return false;
		}
		seen |= (uint16_t)(1U << p[0]);
		r->entries[i] = (struct relay_peer_observation){p[0], relay_get_be16(p + 1), p[3], p[4], p[5], p[6]};
	}
	return true;
}

/* Explicit grid primitives also support the initial acquisition frame, before
 * the primary control frame supplies the authoritative variable-length timeline. */
/* Tick subtraction stays modular across a receiver-clock rollover. */
#if defined(__GNUC__)
__attribute__((always_inline))
#endif
static inline uint32_t relay_frame_start_grid(uint32_t origin, uint32_t frame, uint32_t numerator, uint32_t denominator)
{
	return origin + (frame / denominator) * numerator + (frame % denominator) * numerator / denominator;
}
RELAY_LAYOUT_INLINE uint32_t relay_frame_start(uint32_t origin, uint32_t frame)
{
#if defined(CONFIG_RELAY_RUNTIME_LAYOUT)
	uint32_t start;
	if (relay_timeline_start(&relay_timeline_active, frame, &start)) {
		return start;
	}
#endif
	return relay_frame_start_grid(origin, frame, RELAY_GRID_TICKS, RELAY_GRID_FRAMES);
}

#if defined(__GNUC__)
__attribute__((always_inline))
#endif
static inline uint32_t
relay_frame_at_tick_grid(uint32_t origin, uint32_t tick, uint32_t numerator, uint32_t denominator)
{
	uint32_t delta = tick - origin;
	return (delta / numerator) * denominator + ((delta % numerator) * denominator + denominator - 1U) / numerator;
}
RELAY_LAYOUT_INLINE uint32_t relay_frame_at_tick(uint32_t origin, uint32_t tick)
{
#if defined(CONFIG_RELAY_RUNTIME_LAYOUT)
	if (relay_timeline_active.valid) {
		uint32_t anchor = relay_timeline_active.frame;
		for (int offset = 3; offset >= -3; offset--) {
			uint32_t candidate = anchor + offset, at;
			if (relay_timeline_start(&relay_timeline_active, candidate, &at) && (int32_t)((uint32_t)tick - at) >= 0) {
				return candidate;
			}
		}
	}
#endif
	return relay_frame_at_tick_grid(origin, tick, RELAY_GRID_TICKS, RELAY_GRID_FRAMES);
}

/* Extend a nearby wrapping tick from a recent full-frame control frame anchor.
 * The anchor's modulo-RELAY_GRID_FRAMES phase preserves the exact fractional grid. */
#if defined(__GNUC__)
__attribute__((always_inline))
#endif
static inline uint32_t
relay_frame_near_grid(uint32_t frame, uint32_t start, uint32_t tick, uint32_t numerator, uint32_t denominator)
{
	/* Divide the signed displacement before adding the bounded anchor phase.
	 * This preserves INT32_MIN/MAX behavior without two 64-bit divisions on
	 * every tracker admission. Remainder plus phase fits within two periods. */
	int32_t displacement = (int32_t)(tick - start);
	int32_t blocks = displacement / (int32_t)numerator;
	int32_t remainder = displacement % (int32_t)numerator;
	remainder += (int32_t)((frame % denominator) * numerator / denominator);
	if (remainder < 0) {
		remainder += numerator;
		blocks--;
	} else if ((uint32_t)remainder >= numerator) {
		remainder -= numerator;
		blocks++;
	}
	return frame - frame % denominator + (uint32_t)blocks * denominator
		 + ((uint32_t)remainder * denominator + denominator - 1U) / numerator;
}
RELAY_LAYOUT_INLINE uint32_t relay_frame_near(uint32_t frame, uint32_t start, uint32_t tick)
{
#if defined(CONFIG_RELAY_RUNTIME_LAYOUT)
	if (relay_timeline_active.valid) {
		uint32_t anchor = relay_timeline_active.frame;
		for (int offset = 3; offset >= -3; offset--) {
			uint32_t candidate = anchor + offset, at;
			if (relay_timeline_start(&relay_timeline_active, candidate, &at) && (int32_t)(tick - at) >= 0) {
				return candidate;
			}
		}
	}
#endif
	return relay_frame_near_grid(frame, start, tick, RELAY_GRID_TICKS, RELAY_GRID_FRAMES);
}

static inline uint32_t relay_frame_start_near(uint32_t frame, uint32_t start, uint32_t target)
{
#if defined(CONFIG_RELAY_RUNTIME_LAYOUT)
	uint32_t at;
	if (relay_timeline_start(&relay_timeline_active, target, &at)) {
		return at;
	}
#endif
	return start + relay_frame_start_grid(0, target, RELAY_GRID_TICKS, RELAY_GRID_FRAMES)
		 - relay_frame_start_grid(0, frame, RELAY_GRID_TICKS, RELAY_GRID_FRAMES);
}

#if defined(__GNUC__)
__attribute__((always_inline))
#endif
static inline uint32_t
relay_frame_at_tick64_grid(uint64_t origin, uint64_t tick, uint32_t numerator, uint32_t denominator)
{
	uint64_t delta = tick - origin;
	return (uint32_t)((delta / numerator) * denominator
					  + ((delta % numerator) * denominator + denominator - 1U) / numerator);
}
RELAY_LAYOUT_INLINE uint32_t relay_frame_at_tick64(uint64_t origin, uint64_t tick)
{
#if defined(CONFIG_RELAY_RUNTIME_LAYOUT)
	if (relay_timeline_active.valid) {
		uint32_t anchor = relay_timeline_active.frame;
		for (int offset = 3; offset >= -3; offset--) {
			uint32_t candidate = anchor + offset, at;
			if (relay_timeline_start(&relay_timeline_active, candidate, &at) && (int32_t)((uint32_t)tick - at) >= 0) {
				return candidate;
			}
		}
	}
#endif
	return relay_frame_at_tick64_grid(origin, tick, RELAY_GRID_TICKS, RELAY_GRID_FRAMES);
}

/* The control frame's preamble and the rational-grid frame start are distinct coordinates. */
static inline uint32_t relay_control_start(uint32_t preamble)
{
	return preamble - RELAY_CONTROL_PREAMBLE;
}

static inline uint32_t relay_control_origin(uint32_t preamble, uint32_t frame)
{
	return relay_control_start(preamble) - relay_frame_start_grid(0, frame, RELAY_GRID_TICKS, RELAY_GRID_FRAMES);
}

static inline uint32_t relay_repair_start(uint32_t preamble, unsigned slot)
{
	return relay_control_start(preamble) + RELAY_REPAIR_START + slot * RELAY_SLOT_TICKS;
}

static inline uint32_t relay_frame_phase(uint32_t origin, uint32_t frame, uint32_t tick)
{
	return tick - relay_frame_start(origin, frame);
}

/* Caller first rejects phases before the repair region. */
static inline unsigned relay_repair_slot(uint32_t phase)
{
	return (phase - RELAY_REPAIR_START) / RELAY_SLOT_TICKS;
}

static inline uint8_t relay_direct_payload_max(uint8_t profile, uint8_t slot)
{
	return profile == RELAY_PROFILE_120HZ
		? (slot == RELAY_DIRECT_SLOTS - 1U ? RELAY_LAST_DIRECT_MAX : RELAY_DIRECT_MAX)
		: RELAY_RECORD_MAX_LEN;
}

#endif
