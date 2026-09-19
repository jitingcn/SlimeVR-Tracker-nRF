#include "tracker_events.h"
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <limits.h>
LOG_MODULE_REGISTER(tracker_events);

extern void connection_tracker_event_wake(void);
#define QUEUE_SIZE 32
#define OP_SIZE 8
struct record {
	struct tracker_event event;
	uint32_t token, born, sent;
	uint8_t copies, limit;
	bool valid;
};
struct operation {
	struct record latest;
	uint32_t born, changed, progress_sent, heartbeat, retry_at;
	uint16_t id;
	uint8_t kind, phase, detail, completion_reason, retry_reason;
	bool active, visible, candidate, progress_valid, retry_valid;
};
struct state_slot {
	struct record published;
	uint32_t changed, last_observed, last_publish;
	uint8_t phase, detail;
	bool candidate, observed, publish_valid;
};
static struct k_spinlock event_lock;
static struct record queue[QUEUE_SIZE];
static struct operation operations[OP_SIZE];
static struct state_slot states[2];
/* Physical-boot observations survive identity changes only until admission. */
static struct {
	uint32_t at, token[2];
	uint8_t phase;
	bool scheduled, retired, pending[2];
} startup;
static uint32_t nonce, serial, epoch = 1, next_send, diagnostics_at;
static uint32_t drops, coalesces, suppressions;
static uint16_t sequence, next_operation;
static uint8_t queue_rr, op_rr, state_rr;
static bool send_gate, notify_pending, entropy_failed;
/* One connection owner performs selection and its synchronous ESB admission. */
static struct record selected;
static bool due(uint32_t now, uint32_t at)
{
	return (int32_t)(now - at) >= 0;
}
static uint32_t later(uint32_t a, uint32_t b)
{
	return (int32_t)(a - b) >= 0 ? a : b;
}
static uint32_t token_next(void)
{
	if (!++serial) {
		++serial;
	}
	return serial;
}
static struct operation *lookup(uint16_t id)
{
	if (!id) {
		return NULL;
	}
	for (unsigned i = 0; i < OP_SIZE; i++) {
		if (operations[i].id == id) {
			return &operations[i];
		}
	}
	return NULL;
}
static struct record materialize(
	uint16_t op,
	uint8_t kind,
	uint8_t event,
	uint8_t outcome,
	uint8_t phase,
	uint8_t detail,
	uint32_t now,
	uint8_t limit
)
{
	return (struct record){
		.event
		= {.nonce = nonce,
		   .event_seq = sequence++,
		   .operation_id = op,
		   .kind = kind,
		   .event = event,
		   .outcome = outcome,
		   .phase = phase,
		   .detail = detail},
		.token = token_next(),
		.born = now,
		.limit = limit,
		.valid = true
	};
}
static bool startup_pending(const struct record *r)
{
	return (startup.pending[0] && startup.token[0] == r->token)
		|| (startup.pending[1] && startup.token[1] == r->token);
}
static void enqueue(struct record record)
{
	unsigned victim = 0;
	bool found = false;
	for (unsigned i = 0; i < QUEUE_SIZE; i++) {
		if (!queue[i].valid) {
			victim = i;
			found = true;
			break;
		}
		if (startup_pending(&queue[i])) {
			continue;
		}
		bool power = queue[i].event.kind == TRACKER_EVENT_KIND_POWER;
		bool victim_power = queue[victim].event.kind == TRACKER_EVENT_KIND_POWER;
		if (startup_pending(&queue[victim]) || (!power && victim_power)
			|| (power == victim_power
				&& ((queue[i].copies && !queue[victim].copies)
					|| ((!!queue[i].copies == !!queue[victim].copies)
						&& (int32_t)(queue[i].token - queue[victim].token) < 0)))) {
			victim = i;
		}
	}
	if (!found) {
		drops++;
		/* Ordinary telemetry must not evict a power notice's repeat budget. */
		if (queue[victim].event.kind == TRACKER_EVENT_KIND_POWER
			&& record.event.kind != TRACKER_EVENT_KIND_POWER) {
			return;
		}
	}
	queue[victim] = record;
	notify_pending = true;
}
static struct operation *allocate(uint8_t kind, uint32_t now)
{
	if (!nonce || !tracker_event_is_calibration(kind)) {
		return NULL;
	}
	unsigned victim = 0;
	for (unsigned i = 0; i < OP_SIZE; i++) {
		if (!operations[i].id) {
			victim = i;
			break;
		}
		if ((!operations[i].active && operations[victim].active)
			|| (operations[i].active == operations[victim].active
				&& (int32_t)(operations[i].born - operations[victim].born) < 0)) {
			victim = i;
		}
	}
	if (operations[victim].active) {
		drops++;
	}
	do {
		if (!++next_operation) {
			++next_operation;
		}
	} while (lookup(next_operation));
	operations[victim]
		= (struct operation){.id = next_operation, .kind = kind, .active = true, .born = now, .heartbeat = now + 2000};
	return &operations[victim];
}
static bool hidden_online(const struct operation *op)
{
	return op->kind == (CAL_KIND_MAG_ONLINE | CAL_EVENT_ORIGIN_AUTO) && !op->visible;
}
static void publish_operation(
	struct operation *op,
	uint8_t event,
	uint8_t outcome,
	uint8_t phase,
	uint8_t detail,
	uint32_t now,
	uint8_t limit
)
{
	struct record r = materialize(op->id, op->kind, event, outcome, phase, detail, now, limit);
	enqueue(r);
	if (op->active) {
		op->latest = r;
	}
	op->visible = true;
}
uint16_t cal_event_accept(uint8_t kind)
{
	uint32_t now = k_uptime_get_32();
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	struct operation *op = allocate(kind, now);
	uint16_t id = op ? op->id : 0;
	if (op) {
		publish_operation(op, CAL_EVENT_ACCEPTED, 0, 0, 0, now, 3);
	}
	k_spin_unlock(&event_lock, key);
	return id;
}
uint16_t cal_event_begin(uint8_t kind, uint8_t phase, uint8_t detail)
{
	uint32_t now = k_uptime_get_32();
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	struct operation *op = allocate(kind, now);
	uint16_t id = op ? op->id : 0;
	if (op) {
		op->phase = phase;
		op->detail = detail;
		op->changed = now;
		if (!hidden_online(op)) {
			publish_operation(op, CAL_EVENT_BEGIN, 0, phase, detail, now, 1);
		}
		notify_pending = true;
	}
	k_spin_unlock(&event_lock, key);
	return id;
}
void cal_event_start(uint16_t id, uint8_t phase, uint8_t detail)
{
	uint32_t now = k_uptime_get_32();
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	struct operation *op = lookup(id);
	if (op && op->active) {
		op->phase = phase;
		op->detail = detail;
		publish_operation(op, CAL_EVENT_BEGIN, 0, phase, detail, now, 1);
	}
	k_spin_unlock(&event_lock, key);
}
void cal_event_step(uint16_t id, uint8_t phase, uint8_t detail)
{
	uint32_t now = k_uptime_get_32();
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	struct operation *op = lookup(id);
	if (op && (op->active || phase == CAL_PHASE_STORAGE)) {
		if (phase == CAL_PHASE_RETRY && op->retry_valid && op->retry_reason == detail
			&& !due(now, op->retry_at + 1000)) {
			suppressions++;
			k_spin_unlock(&event_lock, key);
			return;
		}
		if (phase == CAL_PHASE_RETRY) {
			op->retry_valid = true;
			op->retry_reason = detail;
			op->retry_at = now;
		}
		if (phase == CAL_PHASE_STORAGE && detail == CAL_REASON_STORAGE_ERROR) {
			enqueue(materialize(id, op->kind, CAL_EVENT_STEP, 0, phase, detail, now, 3));
		} else if (op->phase != phase || op->detail != detail) {
			if (op->candidate) {
				coalesces++;
			} else {
				notify_pending = true;
			}
			if (op->latest.valid && op->latest.event.event == CAL_EVENT_STEP && !op->latest.copies) {
				op->latest.valid = false;
			}
			op->phase = phase;
			op->detail = detail;
			op->changed = now;
			op->candidate = true;
		}
	}
	k_spin_unlock(&event_lock, key);
}
void cal_event_set_completion_reason(uint16_t id, uint8_t reason)
{
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	struct operation *op = lookup(id);
	if (op && op->active) {
		op->completion_reason = reason;
	}
	k_spin_unlock(&event_lock, key);
}
void cal_event_end(uint16_t id, uint8_t outcome, uint8_t phase, uint8_t reason)
{
	uint32_t now = k_uptime_get_32();
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	struct operation *op = lookup(id);
	if (op && op->active && outcome >= CAL_OUTCOME_SUCCESS && outcome <= CAL_OUTCOME_SKIPPED) {
		bool suppress = hidden_online(op) && outcome != CAL_OUTCOME_SUCCESS;
		op->active = false;
		op->candidate = false;
		op->latest.valid = false;
		if (outcome == CAL_OUTCOME_SUCCESS && reason == 0) {
			reason = op->completion_reason;
		}
		if (suppress) {
			suppressions++;
		} else {
			publish_operation(op, CAL_EVENT_END, outcome, phase, reason, now, 3);
		}
	}
	k_spin_unlock(&event_lock, key);
}
void cal_event_reject(uint8_t kind, uint8_t reason)
{
	uint32_t now = k_uptime_get_32();
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	struct operation *op = allocate(kind, now);
	if (op) {
		op->active = false;
		publish_operation(op, CAL_EVENT_REJECTED, CAL_OUTCOME_FAILED, 0, reason, now, 3);
	}
	k_spin_unlock(&event_lock, key);
}
static void set_state(unsigned i, uint8_t phase, uint8_t detail, uint32_t now)
{
	struct state_slot *s = &states[i];
	if (s->candidate && s->phase == phase && s->detail == detail) {
		return;
	}
	if (s->published.valid && s->published.event.phase == phase && s->published.event.detail == detail) {
		if (s->candidate) {
			coalesces++;
		}
		s->candidate = false;
		s->phase = phase;
		s->detail = detail;
		return;
	}
	if (s->candidate) {
		coalesces++;
	}
	s->phase = phase;
	s->detail = detail;
	s->changed = now;
	s->candidate = true;
	notify_pending = true;
	/* A changed value must immediately stop all repetition of the old value. */
	s->published.copies = s->published.limit;
}
void tracker_event_set_state(uint8_t kind, uint8_t phase, uint8_t detail)
{
	if (kind != 0x20 && kind != 0x21) {
		return;
	}
	uint32_t now = k_uptime_get_32();
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	struct tracker_event e = {.nonce = nonce, .kind = kind, .event = CAL_EVENT_STATE, .phase = phase, .detail = detail};
	if (tracker_event_valid(&e)) {
		set_state(kind - 0x20, phase, detail, now);
	}
	k_spin_unlock(&event_lock, key);
}
void tracker_event_notice(uint8_t kind, uint8_t phase, uint8_t detail)
{
	uint32_t now = k_uptime_get_32();
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	struct tracker_event e
		= {.nonce = nonce, .kind = kind, .event = CAL_EVENT_NOTICE, .phase = phase, .detail = detail};
	if (tracker_event_valid(&e)) {
		if (kind == TRACKER_EVENT_KIND_POWER && (phase == POWER_WILL_SHUTDOWN || phase == POWER_WILL_REBOOT)) {
			/* A startup generated after this intent could incorrectly supersede it.
			 * Already materialized records retain their older ordering identity. */
			startup.retired = true;
			startup.pending[0] = startup.pending[1] = false;
		}
		if (kind == TRACKER_EVENT_KIND_POWER && phase == POWER_WOM_CANCELLED) {
			for (unsigned i = 0; i < QUEUE_SIZE; i++) {
				if (queue[i].valid && queue[i].event.kind == TRACKER_EVENT_KIND_POWER
					&& queue[i].event.phase == POWER_WILL_WOM) {
					queue[i].valid = false;
				}
			}
		}
		enqueue(materialize(0, kind, CAL_EVENT_NOTICE, 0, phase, detail, now, 3));
	}
	k_spin_unlock(&event_lock, key);
}
void tracker_events_schedule_boot(bool wake, bool watchdog_reset)
{
	uint32_t now = k_uptime_get_32();
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	if (!startup.scheduled && !startup.retired && !entropy_failed) {
		startup.scheduled = true;
		startup.at = now + TRACKER_EVENT_BOOT_DELAY_MS;
		startup.phase = wake ? POWER_WAKE : POWER_BOOT;
		startup.pending[0] = true;
		startup.pending[1] = watchdog_reset;
		notify_pending = true;
	}
	k_spin_unlock(&event_lock, key);
	tracker_events_notify();
}
uint32_t tracker_events_sensor_epoch(void)
{
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	uint32_t result = epoch;
	k_spin_unlock(&event_lock, key);
	return result;
}
void tracker_events_sensor_invalidate(uint8_t reason)
{
	uint32_t now = k_uptime_get_32();
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	if (!++epoch) {
		++epoch;
	}
	for (unsigned i = 0; i < 2; i++) {
		states[i].observed = false;
		states[i].published.valid = false;
		states[i].candidate = false;
		if (nonce) {
			set_state(i, 2, i ? states[i].detail : reason, now);
		}
	}
	k_spin_unlock(&event_lock, key);
}
void tracker_events_observe_sensor(
	uint32_t frame_epoch,
	bool tracker_fresh,
	bool tracker_rest,
	bool fusion_fresh,
	bool fusion_rest,
	uint8_t backend,
	uint32_t now
)
{
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	if (nonce && frame_epoch == epoch) {
		if (tracker_fresh) {
			states[0].observed = true;
			states[0].last_observed = now;
			set_state(0, tracker_rest ? 1 : 0, 0, now);
		}
		if (backend != FUSION_BACKEND_VQF && backend != FUSION_BACKEND_EQF) {
			states[1].observed = false;
			set_state(1, 3, 0, now);
		} else if (fusion_fresh) {
			states[1].observed = true;
			states[1].last_observed = now;
			set_state(1, fusion_rest ? 1 : 0, backend, now);
		}
	}
	k_spin_unlock(&event_lock, key);
}
/* Called only with event_lock. All generated records receive publication-order sequence numbers. */
static void maintain(uint32_t now)
{
	for (unsigned i = 0; i < QUEUE_SIZE; i++) {
		if (queue[i].valid && !startup_pending(&queue[i]) && due(now, queue[i].born + TRACKER_EVENT_TTL_MS)) {
			queue[i].valid = false;
			drops++;
		}
	}
	if (nonce && startup.scheduled && due(now, startup.at)) {
		for (unsigned i = 0; i < 2; i++) {
			if (startup.pending[i] && !startup.token[i]) {
				struct record r = materialize(
					0, TRACKER_EVENT_KIND_POWER, CAL_EVENT_NOTICE, 0,
					i ? POWER_WATCHDOG_RESET : startup.phase, 0, now, 3
				);
				startup.token[i] = r.token;
				enqueue(r);
			}
		}
	}
	for (unsigned i = 0; i < OP_SIZE; i++) {
		struct operation *op = &operations[i];
		if (op->active && hidden_online(op) && due(now, op->born + 1000)) {
			publish_operation(op, CAL_EVENT_BEGIN, 0, op->phase, op->detail, now, 1);
			op->candidate = false;
		}
	}
	for (unsigned i = 0; i < 2; i++) {
		struct state_slot *s = &states[i];
		if (s->observed && due(now, s->last_observed + 1000)) {
			s->observed = false;
			set_state(i, 2, i ? s->detail : TRACKER_REST_NO_FRESH_FRAME, now);
		}
		if (s->published.valid && s->published.copies < s->published.limit && due(now, s->published.born + 15000)) {
			s->published.copies = s->published.limit;
			drops++;
		}
	}
}
/* Power selection includes repeats, ahead of all ordinary records. Every valid
 * queue entry has copies < limit: completion invalidates exhausted entries. */
static struct record *oldest_first(bool power)
{
	struct record *best = NULL;
	for (unsigned i = 0; i < QUEUE_SIZE; i++) {
		struct record *r = &queue[i];
		if (!r->valid || (power ? r->event.kind != TRACKER_EVENT_KIND_POWER : r->copies != 0)) {
			continue;
		}
		bool urgent = power && (r->event.phase == POWER_WILL_SHUTDOWN || r->event.phase == POWER_WILL_REBOOT);
		bool best_urgent = best && power
			&& (best->event.phase == POWER_WILL_SHUTDOWN || best->event.phase == POWER_WILL_REBOOT);
		if (!best || (urgent && !best_urgent)
			|| (urgent == best_urgent && (int32_t)(r->token - best->token) < 0)) {
			best = r;
		}
	}
	return best;
}
static struct record *progress(struct operation *op, uint32_t now)
{
	op->latest = materialize(op->id, op->kind, CAL_EVENT_STEP, 0, op->phase, op->detail, now, 1);
	op->candidate = false;
	return &op->latest;
}
static struct record *choose(uint32_t now)
{
	struct record *r = oldest_first(true);
	if (r) {
		return r;
	}
	r = oldest_first(false);
	if (r) {
		return r;
	}
	/* Due heartbeat classes compete by deadline rather than by domain. */
	struct operation *heartbeat_op = NULL;
	struct state_slot *heartbeat_state = NULL;
	uint32_t oldest = now;
	for (unsigned i = 0; i < OP_SIZE; i++) {
		struct operation *op = &operations[i];
		if (!op->active || !op->visible || (!op->latest.valid && !op->candidate) || !due(now, op->heartbeat)) {
			continue;
		}
		if (op->candidate && op->progress_valid && !due(now, op->progress_sent + 500)) {
			continue;
		}
		if (!heartbeat_op || (int32_t)(op->heartbeat - oldest) < 0) {
			heartbeat_op = op;
			oldest = op->heartbeat;
		}
	}
	for (unsigned i = 0; i < 2; i++) {
		struct state_slot *s = &states[i];
		if (s->observed && !s->candidate && s->published.valid && s->published.copies
			&& due(now, s->published.sent + 5000)
			&& ((!heartbeat_op && !heartbeat_state) || (int32_t)(s->published.sent + 5000 - oldest) < 0)) {
			heartbeat_op = NULL;
			heartbeat_state = s;
			oldest = s->published.sent + 5000;
		}
	}
	if (heartbeat_op) {
		return heartbeat_op->candidate ? progress(heartbeat_op, now) : &heartbeat_op->latest;
	}
	if (heartbeat_state) {
		return &heartbeat_state->published;
	}
	for (unsigned n = 0; n < QUEUE_SIZE; n++) {
		unsigned i = (queue_rr + n) % QUEUE_SIZE;
		if (queue[i].valid && queue[i].copies && queue[i].copies < queue[i].limit) {
			queue_rr = (i + 1) % QUEUE_SIZE;
			return &queue[i];
		}
	}
	for (unsigned n = 0; n < 2; n++) {
		unsigned i = (state_rr + n) % 2;
		struct state_slot *s = &states[i];
		if (s->candidate && (!s->publish_valid || due(now, s->last_publish + 500))) {
			s->published = materialize(0, 0x20 + i, CAL_EVENT_STATE, 0, s->phase, s->detail, now, 3);
			s->candidate = false;
			state_rr = (i + 1) % 2;
			return &s->published;
		}
	}
	for (unsigned n = 0; n < OP_SIZE; n++) {
		unsigned i = (op_rr + n) % OP_SIZE;
		struct operation *op = &operations[i];
		if (op->id && op->candidate && !hidden_online(op) && due(now, op->changed + 200)
			&& (!op->progress_valid || due(now, op->progress_sent + 500))) {
			op_rr = (i + 1) % OP_SIZE;
			return progress(op, now);
		}
		/* A materialized progress packet rejected by ESB is still pending. */
		if (op->latest.valid && op->latest.event.event == CAL_EVENT_STEP && !op->latest.copies) {
			op_rr = (i + 1) % OP_SIZE;
			return &op->latest;
		}
	}
	for (unsigned n = 0; n < 2; n++) {
		unsigned i = (state_rr + n) % 2;
		struct state_slot *s = &states[i];
		if (!s->candidate && s->published.valid && s->published.copies < s->published.limit) {
			state_rr = (i + 1) % 2;
			return &s->published;
		}
	}
	return NULL;
}
bool tracker_events_select(uint32_t now, uint8_t tracker_id, struct tracker_event_tx *out)
{
	if (!out || tracker_id >= 16) {
		return false;
	}
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	maintain(now);
	struct record *r = nonce && (!send_gate || due(now, next_send)) ? choose(now) : NULL;
	bool result = false;
	if (r) {
		struct tracker_event event = r->event;
		event.tracker_id = tracker_id;
		result = tracker_event_encode(out->packet, &event);
		out->token = r->token;
		selected = *r;
	}
	k_spin_unlock(&event_lock, key);
	return result;
}
static void receipt(struct record *r, uint32_t token, uint32_t now)
{
	if (r->valid && r->token == token) {
		if (r->copies < UINT8_MAX) {
			r->copies++;
		}
		r->sent = now;
	}
}
void tracker_events_complete(uint32_t token, bool success, uint32_t now)
{
	if (!token) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	if (!selected.valid || selected.token != token || selected.event.nonce != nonce) {
		k_spin_unlock(&event_lock, key);
		return;
	}
	send_gate = true;
	next_send = now + (success ? TRACKER_EVENT_SPACING_MS : 10);
	if (success) {
		bool startup_admitted = startup.token[0] == token || startup.token[1] == token;
		for (unsigned i = 0; i < 2; i++) {
			if (startup.token[i] == token) {
				startup.pending[i] = false;
			}
		}
		struct operation *sent_op = lookup(selected.event.operation_id);
		if (sent_op) {
			sent_op->heartbeat = now + 2000;
			if (selected.event.event == CAL_EVENT_STEP) {
				sent_op->progress_sent = now;
				sent_op->progress_valid = true;
			}
		}
		if (selected.event.event == CAL_EVENT_STATE && !selected.copies) {
			unsigned i = selected.event.kind - TRACKER_EVENT_KIND_TRACKER_REST;
			if (i < 2) {
				states[i].last_publish = now;
				states[i].publish_valid = true;
			}
		}
		for (unsigned i = 0; i < QUEUE_SIZE; i++) {
			struct record *r = &queue[i];
			if (r->valid && r->token == token && !r->copies && startup_admitted) {
				r->born = now;
			}
			receipt(r, token, now);
			if (r->valid && r->token == token && r->copies >= r->limit) {
				r->valid = false;
			}
		}
		for (unsigned i = 0; i < OP_SIZE; i++) {
			struct operation *op = &operations[i];
			if (op->latest.valid && op->latest.token == token) {
				receipt(&op->latest, token, now);
				op->heartbeat = now + 2000;
				if (op->latest.event.event == CAL_EVENT_STEP) {
					op->progress_sent = now;
					op->progress_valid = true;
				}
			}
		}
		for (unsigned i = 0; i < 2; i++) {
			struct state_slot *s = &states[i];
			if (s->published.valid && s->published.token == token) {
				if (!s->published.copies) {
					s->last_publish = now;
					s->publish_valid = true;
				}
				receipt(&s->published, token, now);
			}
		}
	}
	selected.valid = false;
	k_spin_unlock(&event_lock, key);
}
static void deadline_add(uint32_t *best, uint32_t value, uint32_t now)
{
	if (due(now, value)) {
		value = now;
	}
	/* UINT32_MAX is no work. Use the next timestamp (zero after rollover),
	 * not now + 1: signed-delta consumers sleep until the boundary, without
	 * polling each millisecond while a future sentinel-valued deadline waits. */
	if (value == UINT32_MAX) {
		value = 0;
	}
	if (*best == UINT32_MAX || (int32_t)(value - *best) < 0) {
		*best = value;
	}
}
uint32_t tracker_events_deadline(uint32_t now)
{
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	uint32_t best = UINT32_MAX;
	if (nonce) {
		maintain(now);
		for (unsigned i = 0; i < 2; i++) {
			if (startup.pending[i] && !startup.token[i]) {
				deadline_add(&best, startup.at, now);
			}
		}
		for (unsigned i = 0; i < QUEUE_SIZE; i++) {
			if (queue[i].valid) {
				deadline_add(&best, now, now);
			}
		}
		for (unsigned i = 0; i < OP_SIZE; i++) {
			struct operation *op = &operations[i];
			if (!op->id) {
				continue;
			}
			if (op->active && hidden_online(op)) {
				deadline_add(&best, op->born + 1000, now);
			} else {
				if (op->active && op->visible && (op->latest.valid || op->candidate)) {
					deadline_add(
						&best,
						op->candidate && op->progress_valid ? later(op->heartbeat, op->progress_sent + 500)
															: op->heartbeat,
						now
					);
				}
				if (op->candidate) {
					deadline_add(
						&best,
						op->progress_valid ? later(op->changed + 200, op->progress_sent + 500) : op->changed + 200,
						now
					);
				}
				if (op->latest.valid && op->latest.event.event == CAL_EVENT_STEP && !op->latest.copies) {
					deadline_add(&best, now, now);
				}
			}
		}
		for (unsigned i = 0; i < 2; i++) {
			struct state_slot *s = &states[i];
			if (s->observed) {
				deadline_add(&best, s->last_observed + 1000, now);
			}
			if (s->candidate) {
				deadline_add(&best, s->publish_valid ? s->last_publish + 500 : now, now);
			} else if (s->published.valid) {
				if (s->published.copies < s->published.limit) {
					deadline_add(&best, now, now);
				} else if (s->observed) {
					deadline_add(&best, s->published.sent + 5000, now);
				}
			}
		}
		if (best != UINT32_MAX && send_gate) {
			best = later(best, next_send);
			/* The send gate can also land on the sentinel; defer that timestamp
			 * by one millisecond across rollover, just as deadline_add does. */
			if (best == UINT32_MAX) {
				best = 0;
			}
		}
	}
	bool report = due(now, diagnostics_at) && (drops || coalesces || suppressions);
	uint32_t d = drops, c = coalesces, s = suppressions;
	if (report) {
		diagnostics_at = now + 5000;
		drops = coalesces = suppressions = 0;
	}
	k_spin_unlock(&event_lock, key);
	if (report) {
		LOG_DBG("Event telemetry drop=%u coalesce=%u suppress=%u", d, c, s);
	}
	return best;
}
void tracker_events_notify(void)
{
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	bool wake = notify_pending;
	notify_pending = false;
	k_spin_unlock(&event_lock, key);
	if (wake) {
		connection_tracker_event_wake();
	}
}
void tracker_events_session_changed(void)
{
	uint32_t fresh = 0;
	int error = 0;
	/* Disable publication before entropy acquisition: old identity cannot leak. */
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	if (entropy_failed) {
		k_spin_unlock(&event_lock, key);
		return;
	}
	nonce = 0;
	k_spin_unlock(&event_lock, key);
	for (unsigned i = 0; i < 4; i++) {
		error = sys_csrand_get(&fresh, sizeof(fresh));
		if (error || fresh) {
			break;
		}
	}
	key = k_spin_lock(&event_lock);
	memset(queue, 0, sizeof(queue));
	memset(operations, 0, sizeof(operations));
	memset(states, 0, sizeof(states));
	sequence = 0;
	send_gate = false;
	/* Re-materialize only observations never admitted in the old identity. */
	startup.token[0] = startup.token[1] = 0;
	notify_pending = startup.pending[0] || startup.pending[1];
	selected.valid = false;
	if (!++epoch) {
		++epoch;
	}
	nonce = error ? 0 : fresh;
	entropy_failed = !nonce;
	k_spin_unlock(&event_lock, key);
	if (error) {
		LOG_ERR("Event session disabled: cryptographic entropy API failed (%d)", error);
	} else if (!fresh) {
		LOG_ERR("Event session disabled: cryptographic entropy returned zero on all 4 attempts");
	}
}
static int tracker_events_init(void)
{
	tracker_events_session_changed();
	return 0;
}
SYS_INIT(tracker_events_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
