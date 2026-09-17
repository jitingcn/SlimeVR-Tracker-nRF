#ifndef TRACKER_EVENT_PROTOCOL_H
#define TRACKER_EVENT_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define TRACKER_EVENT_ESB_TYPE 0xE0
#define TRACKER_EVENT_VERSION 1
#define TRACKER_EVENT_MAGIC_0 0x43
#define TRACKER_EVENT_MAGIC_1 0x45
#define TRACKER_EVENT_ESB_LEN 17
#define TRACKER_EVENT_BODY_LEN 15
#define TRACKER_EVENT_HID_LEN 16
#define TRACKER_EVENT_MAX_TRACKERS 16
#define CAL_EVENT_ORIGIN_AUTO 0x80
#define TRACKER_EVENT_KIND_MASK 0x7F
#define TRACKER_EVENT_MAX_REPEATS 3
#define TRACKER_EVENT_SPACING_MS 100U
#define TRACKER_EVENT_CAL_HEARTBEAT_MS 2000U
#define TRACKER_EVENT_CAL_SILENCE_MS 10000U
#define TRACKER_EVENT_REST_HEARTBEAT_MS 5000U
#define TRACKER_EVENT_REST_STALE_MS 15000U
#define TRACKER_EVENT_LEASE_MS 15000U
#define TRACKER_EVENT_SENSOR_FRESH_MS 1000U
#define TRACKER_EVENT_TTL_MS 15000U
#define RCV_HID_OP_TRACKER_EVENTS 224
#define RCV_HID_OP_TRACKER_EVENT 225
#define RCV_HID_OP_TRACKER_OBSERVATION 226

enum {
	CAL_EVENT_NONE = 0,
	CAL_EVENT_ACCEPTED = 1,
	CAL_EVENT_BEGIN = 2,
	CAL_EVENT_STEP = 3,
	CAL_EVENT_END = 4,
	CAL_EVENT_REJECTED = 5,
	CAL_EVENT_STATE = 6,
	CAL_EVENT_NOTICE = 7
};

enum {
	CAL_OUTCOME_NONE = 0,
	CAL_OUTCOME_SUCCESS = 1,
	CAL_OUTCOME_FAILED = 2,
	CAL_OUTCOME_CANCELLED = 3,
	CAL_OUTCOME_SKIPPED = 4,
	CAL_OUTCOME_UNKNOWN = 5
};

enum {
	CAL_PHASE_NONE = 0,
	CAL_PHASE_IDENTIFY = 1,
	CAL_PHASE_WAIT_STILL = 2,
	CAL_PHASE_SENSOR_RETRIM = 3,
	CAL_PHASE_COLLECT = 4,
	CAL_PHASE_WAIT_POSE = 5,
	CAL_PHASE_CAPTURE_POSE = 6,
	CAL_PHASE_POSE_DONE = 7,
	CAL_PHASE_RETRY = 8,
	CAL_PHASE_WAIT_ROTATION = 9,
	CAL_PHASE_RECORD_ROTATION = 10,
	CAL_PHASE_FIT = 11,
	CAL_PHASE_FREEZE = 12,
	CAL_PHASE_VALIDATE = 13,
	CAL_PHASE_PROBATION = 14,
	CAL_PHASE_APPLY_PENDING = 15,
	CAL_PHASE_APPLIED = 16,
	CAL_PHASE_CONFIRM = 17,
	CAL_PHASE_STORAGE = 18,
	CAL_PHASE_COVERAGE = 19
};

enum {
	CAL_REASON_NONE = 0,
	CAL_REASON_BUSY = 1,
	CAL_REASON_UNSUPPORTED = 2,
	CAL_REASON_INVALID_ARGUMENT = 3,
	CAL_REASON_SENSOR_UNAVAILABLE = 4,
	CAL_REASON_MOTION = 5,
	CAL_REASON_SAMPLE_TIMEOUT = 6,
	CAL_REASON_INSUFFICIENT_SAMPLES = 7,
	CAL_REASON_POSE_TIMEOUT = 8,
	CAL_REASON_FIT_ERROR = 9,
	CAL_REASON_INVALID_MODEL = 10,
	CAL_REASON_QUALITY = 11,
	CAL_REASON_NO_BENEFIT = 12,
	CAL_REASON_ENVIRONMENT_ONLY = 13,
	CAL_REASON_DISABLED = 14,
	CAL_REASON_REPLACED = 15,
	CAL_REASON_POWER_DOWN = 16,
	CAL_REASON_RESET = 17,
	CAL_REASON_STORAGE_ERROR = 18,
	CAL_REASON_CANDIDATE_REJECTED = 19,
	CAL_REASON_NO_TCAL_COVERAGE = 20,
	CAL_REASON_START_TIMEOUT = 21,
	CAL_REASON_RECORD_TIMEOUT = 22,
	CAL_REASON_TEMPERATURE = 23,
	CAL_REASON_RADIAL = 24,
	CAL_REASON_DIP = 25,
	CAL_REASON_COVERAGE = 26,
	CAL_REASON_EXPIRED = 27,
	CAL_REASON_INVALID_SAMPLE = 28,
	CAL_REASON_OVERFLOW = 29,
	CAL_REASON_TRANSPORT_SILENCE = 30,
	CAL_REASON_SESSION_CHANGED = 31,
	CAL_REASON_PARTIAL = 32
};

enum {
	CAL_KIND_IMU_ZRO = 1,
	CAL_KIND_ACCEL_POSES = 2,
	CAL_KIND_MAG_MANUAL = 3,
	CAL_KIND_MAG_ONLINE = 4,
	CAL_KIND_GYRO_SENS = 5,
	CAL_KIND_TCAL_BOOT = 6,
	CAL_KIND_TCAL_RUNTIME = 7,
	TRACKER_EVENT_KIND_TRACKER_REST = 0x20,
	TRACKER_EVENT_KIND_FUSION_REST = 0x21,
	TRACKER_EVENT_KIND_POWER = 0x30,
	TRACKER_EVENT_KIND_BUTTON = 0x31
};

enum { TRACKER_REST_NOT_REST = 0, TRACKER_REST_REST = 1, TRACKER_REST_UNKNOWN = 2 };

enum {
	TRACKER_REST_OBSERVED = 0,
	TRACKER_REST_RESET = 1,
	TRACKER_REST_SUSPENDED = 2,
	TRACKER_REST_NO_FRESH_FRAME = 3,
	TRACKER_REST_INITIALIZING = 4
};

enum { FUSION_REST_NOT_DETECTED = 0, FUSION_REST_DETECTED = 1, FUSION_REST_UNKNOWN = 2, FUSION_REST_UNAVAILABLE = 3 };

enum { FUSION_BACKEND_UNKNOWN = 0, FUSION_BACKEND_VQF = 1, FUSION_BACKEND_EQF = 2 };

enum {
	POWER_WILL_WOM = 1,
	POWER_WILL_SHUTDOWN = 2,
	POWER_REASON_UNKNOWN = 0,
	POWER_WOM_NORMAL = 1,
	POWER_WOM_FORCED = 2,
	BUTTON_CLICK_GROUP = 1
};

struct tracker_event {
	uint32_t nonce;
	uint16_t event_seq, operation_id;
	uint8_t tracker_id, kind, event, outcome, phase, detail;
};

static inline bool tracker_event_is_calibration(uint8_t kind)
{
	uint8_t k = kind & TRACKER_EVENT_KIND_MASK;
	return k >= 1 && k <= 7;
}

static inline uint8_t tracker_event_kind_filter(uint8_t kind)
{
	if (tracker_event_is_calibration(kind)) {
		return 1;
	}
	switch (kind) {
	case 0x20:
		return 2;
	case 0x21:
		return 4;
	case 0x30:
		return 8;
	case 0x31:
		return 16;
	default:
		return 0;
	}
}

static inline bool tracker_event_valid(const struct tracker_event *e)
{
	if (!e || !e->nonce || e->tracker_id >= 16) {
		return false;
	}
	if (tracker_event_is_calibration(e->kind)) {
		if (!e->operation_id) {
			return false;
		}
		switch (e->event) {
		case CAL_EVENT_ACCEPTED:
		case CAL_EVENT_BEGIN:
		case CAL_EVENT_STEP:
			return e->outcome == CAL_OUTCOME_NONE;
		case CAL_EVENT_END:
			return e->outcome >= CAL_OUTCOME_SUCCESS && e->outcome <= CAL_OUTCOME_SKIPPED;
		case CAL_EVENT_REJECTED:
			return e->outcome == CAL_OUTCOME_FAILED;
		default:
			return false;
		}
	}
	if (e->operation_id || e->outcome) {
		return false;
	}
	switch (e->kind) {
	case TRACKER_EVENT_KIND_TRACKER_REST:
		return e->event == CAL_EVENT_STATE && e->phase <= 2 && e->detail <= 4;
	case TRACKER_EVENT_KIND_FUSION_REST:
		return e->event == CAL_EVENT_STATE && e->phase <= 3 && e->detail <= 2;
	case TRACKER_EVENT_KIND_POWER:
		return e->event == CAL_EVENT_NOTICE && ((e->phase == 1 && e->detail <= 2) || (e->phase == 2 && e->detail == 0));
	case TRACKER_EVENT_KIND_BUTTON:
		return e->event == CAL_EVENT_NOTICE && e->phase == 1 && e->detail != 0;
	default:
		return false;
	}
}

static inline uint16_t tracker_event_get16(const uint8_t *p)
{
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t tracker_event_get32(const uint8_t *p)
{
	return (uint32_t)tracker_event_get16(p) | ((uint32_t)tracker_event_get16(p + 2) << 16);
}

static inline void tracker_event_put16(uint8_t *p, uint16_t x)
{
	p[0] = x;
	p[1] = x >> 8;
}

static inline void tracker_event_put32(uint8_t *p, uint32_t x)
{
	tracker_event_put16(p, x);
	tracker_event_put16(p + 2, x >> 16);
}

static inline bool tracker_event_encode(uint8_t out[TRACKER_EVENT_ESB_LEN], const struct tracker_event *e)
{
	if (!tracker_event_valid(e)) {
		return false;
	}
	out[0] = TRACKER_EVENT_ESB_TYPE;
	out[1] = e->tracker_id;
	out[2] = TRACKER_EVENT_VERSION;
	out[3] = e->event | (e->outcome << 4);
	tracker_event_put32(out + 4, e->nonce);
	tracker_event_put16(out + 8, e->event_seq);
	tracker_event_put16(out + 10, e->operation_id);
	out[12] = e->kind;
	out[13] = e->phase;
	out[14] = e->detail;
	out[15] = TRACKER_EVENT_MAGIC_0;
	out[16] = TRACKER_EVENT_MAGIC_1;
	return true;
}

static inline bool tracker_event_decode(const uint8_t *p, size_t len, struct tracker_event *e)
{
	if (!p || !e || len != TRACKER_EVENT_ESB_LEN || p[0] != TRACKER_EVENT_ESB_TYPE || p[2] != TRACKER_EVENT_VERSION
		|| p[15] != TRACKER_EVENT_MAGIC_0 || p[16] != TRACKER_EVENT_MAGIC_1) {
		return false;
	}
	*e = (struct tracker_event){.tracker_id = p[1],
								.nonce = tracker_event_get32(p + 4),
								.event_seq = tracker_event_get16(p + 8),
								.operation_id = tracker_event_get16(p + 10),
								.kind = p[12],
								.event = p[3] & 15,
								.outcome = p[3] >> 4,
								.phase = p[13],
								.detail = p[14]};
	return tracker_event_valid(e);
}

static inline bool tracker_event_decode_body(uint8_t tracker, const uint8_t *p, size_t len, struct tracker_event *e)
{
	if (!p || len != TRACKER_EVENT_BODY_LEN) {
		return false;
	}
	uint8_t packet[TRACKER_EVENT_ESB_LEN] = {TRACKER_EVENT_ESB_TYPE, tracker};
	memcpy(packet + 2, p, TRACKER_EVENT_BODY_LEN);
	return tracker_event_decode(packet, TRACKER_EVENT_ESB_LEN, e);
}

/* Observations deliberately allow UNKNOWN, which is never legal tracker wire. */
static inline void
tracker_event_encode_hid(uint8_t out[TRACKER_EVENT_HID_LEN], const struct tracker_event *e, uint8_t opcode)
{
	out[0] = 251;
	out[1] = 0;
	out[2] = opcode;
	out[3] = e->event | (e->outcome << 4);
	out[4] = e->tracker_id;
	tracker_event_put32(out + 5, e->nonce);
	tracker_event_put16(out + 9, e->event_seq);
	tracker_event_put16(out + 11, e->operation_id);
	out[13] = e->kind;
	out[14] = e->phase;
	out[15] = e->detail;
}

#endif
