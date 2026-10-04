#include "led_policy.h"
#include <limits.h>
#include <string.h>

/* Every interval fits signed uptime32 arithmetic, including a complete button
 * count train. Network raw32 discontinuity never enters these deadlines. */
static bool due(uint32_t now_ms, uint32_t deadline_ms)
{
	return (int32_t)(now_ms - deadline_ms) >= 0;
}
static uint32_t remaining(uint32_t now_ms, uint32_t deadline_ms)
{
	return due(now_ms, deadline_ms) ? 0 : deadline_ms - now_ms;
}
static uint32_t identity(struct led_engine *e)
{
	if (++e->identity_counter == 0) {
		++e->identity_counter;
	}
	return e->identity_counter;
}
static bool terminal(enum led_semantic semantic)
{
	return semantic == LED_SUCCESS || semantic == LED_FAILED || semantic == LED_PARTIAL
		|| semantic == LED_APPLIED_NOT_SAVED || semantic == LED_CANCELLED;
}
static bool non_success(enum led_semantic semantic)
{
	return semantic == LED_FAILED || semantic == LED_PARTIAL || semantic == LED_APPLIED_NOT_SAVED
		|| semantic == LED_REJECTED;
}
static uint32_t black_gap(const struct led_engine *e, uint32_t now_ms, uint32_t minimum_ms)
{
	if (!e->black_known) {
		return minimum_ms;
	}
	uint32_t elapsed = now_ms - e->black_since_ms;
	return elapsed >= minimum_ms ? 0 : minimum_ms - elapsed;
}
static bool manual_exit_active(const struct led_engine *e)
{
	return e->owners[LED_OWNER_SYSTEM].semantic == LED_MANUAL_EXIT;
}
static void ready_truth(struct led_engine *e)
{
	bool blocked = false;
	for (enum led_owner owner = LED_OWNER_IMU; owner < LED_OWNER_COUNT; ++owner) {
		const struct led_owner_state *state = &e->owners[owner];
		blocked |= state->fault != LED_FAULT_NONE || state->maintenance || state->ota_active
				|| state->semantic == LED_OTA_ACTIVE || state->semantic == LED_TEST_ACTIVE;
	}
	e->ready = e->connection.radio_required && e->connection.healthy && e->connection.output_ready && !blocked;
}
void led_engine_init(struct led_engine *e, uint32_t now_ms)
{
	memset(e, 0, sizeof(*e));
	e->winner.priority = LED_PRIORITY_BACKGROUND;
#ifdef CONFIG_LED_DEBUG
	e->real_winner.priority = LED_PRIORITY_BACKGROUND;
#endif
	e->black_since_ms = now_ms;
}
struct led_token led_engine_begin(struct led_engine *e, enum led_owner owner, uint32_t request_id)
{
	struct led_token token = {.owner = owner, .request_id = request_id};
	if (e->quiesced || !led_owner_valid(owner) || !request_id) {
		return token;
	}
	struct led_owner_state *state = &e->owners[owner];
	if (state->session && state->request_id == request_id) {
		token.session = state->session;
		return token;
	}
	if (++state->session == 0) {
		++state->session;
	}
	e->ack_seen[owner] = false;
	state->request_id = request_id;
	state->revision = 0;
	state->semantic = LED_NONE;
	state->terminal = LED_NONE;
	state->identity = identity(e);
	struct led_pending_event *pending = &e->events[owner];
	if (!(pending->present && pending->semantic == LED_INPUT_ACK && pending->request_id == request_id)) {
		pending->present = false;
	}
	if (e->active_event.present && !e->active_event.button_count && e->active_owner == owner
		&& !(e->active_event.semantic == LED_INPUT_ACK && e->active_event.request_id == request_id)) {
		e->active_event.present = false;
	}
	token.session = state->session;
	return token;
}
enum led_admission led_engine_state(
	struct led_engine *e,
	struct led_token token,
	uint32_t revision,
	enum led_semantic semantic,
	uint32_t now_ms
)
{
	if (e->quiesced) {
		return LED_SHUTDOWN;
	}
	const struct led_behavior *behavior = led_behavior_get(semantic);
	if (!led_owner_valid(token.owner) || !token.session || !revision
		|| (semantic != LED_NONE && (!behavior || behavior->priority != LED_PRIORITY_TASK))
		|| (semantic == LED_MANUAL_EXIT && token.owner != LED_OWNER_SYSTEM)) {
		return LED_INVALID;
	}
	struct led_owner_state *state = &e->owners[token.owner];
	if (token.session != state->session || token.request_id != state->request_id || state->terminal != LED_NONE) {
		return LED_STALE;
	}
	if ((int32_t)(revision - state->revision) <= 0) {
		return LED_DUPLICATE;
	}
	state->revision = revision;
	if (semantic != state->semantic) {
		state->semantic = semantic;
		state->origin_ms = now_ms;
		state->identity = identity(e);
		ready_truth(e);
	}
	if (semantic == LED_MANUAL_EXIT) {
		/* A terminal gesture gets one end cue, not old counts/results followed
		 * by a fade. Business facts and terminal bookkeeping remain intact. */
		for (enum led_owner owner = LED_OWNER_IMU; owner < LED_OWNER_COUNT; ++owner) {
			e->events[owner].present = false;
		}
		if (!e->active_event.button_count) {
			e->active_event.present = false;
		}
	}
	return LED_ADMITTED;
}

static enum led_admission enqueue(
	struct led_engine *e,
	struct led_token token,
	uint32_t event_id,
	enum led_semantic semantic,
	uint32_t now_ms
)
{
	/* New ordinary result facts may still finish their owner's bookkeeping,
	 * but cannot queue a second visual behind an explicit manual exit. */
	if (manual_exit_active(e)) {
		return LED_ADMITTED;
	}
	const struct led_behavior *behavior = led_behavior_get(semantic);
	struct led_pending_event *slot = &e->events[token.owner];
	if (terminal(semantic) && e->active_event.present && e->active_owner == token.owner
		&& e->active_event.request_id == token.request_id) {
		e->active_event.present = false;
	}
	if (slot->present) {
		const struct led_behavior *old = led_behavior_get(slot->semantic);
		bool same = slot->request_id == token.request_id && slot->session == token.session;
		if (same && non_success(slot->semantic) && semantic == LED_SUCCESS) {
			return LED_STALE;
		}
		if (!same && old->result_rank < behavior->result_rank) {
			return LED_ADMITTED;
		}
	}
	*slot = (struct led_pending_event){
		.present = true,
		.session = token.session,
		.request_id = token.request_id,
		.event_id = event_id,
		.semantic = semantic,
		.occurred_ms = now_ms,
		.expires_ms = now_ms + behavior->ttl_ms,
		.identity = identity(e)
	};
	return LED_ADMITTED;
}
static int ack_identity_class(enum led_semantic semantic)
{
	switch (semantic) {
	case LED_INPUT_ACK:
		return 0;
	case LED_ACCEPTED:
		return 1;
	case LED_STAGE_ACK:
		return 2;
	default:
		return -1;
	}
}
enum led_admission led_engine_event(
	struct led_engine *e,
	struct led_token token,
	uint32_t event_id,
	enum led_semantic semantic,
	uint32_t now_ms,
	bool request_only
)
{
	if (e->quiesced) {
		return LED_SHUTDOWN;
	}
	const struct led_behavior *behavior = led_behavior_get(semantic);
	if (!led_owner_valid(token.owner) || !token.request_id || !event_id || !behavior
		|| behavior->priority != LED_PRIORITY_EVENT) {
		return LED_INVALID;
	}
	struct led_owner_state *state = &e->owners[token.owner];
	if (!request_only && (!token.session || token.session != state->session || token.request_id != state->request_id)) {
		return LED_STALE;
	}
	/* Terminal identity is deduplicated by operation/request below, not a
	 * global event high-water mark: its ID may have been reserved before
	 * unrelated same-owner input facts occurred. */
	int ack_class = ack_identity_class(semantic);
	if (ack_class >= 0) {
		uint32_t previous_id = e->last_ack_event_id[token.owner][ack_class];
		if (previous_id && (int32_t)(event_id - previous_id) <= 0) {
			return LED_DUPLICATE;
		}
	}
	if (request_only) {
		token.session = 0;
	}
	bool is_terminal = terminal(semantic) || (request_only && semantic == LED_REJECTED);
	if (request_only && e->terminal_request_id[token.owner]
		&& (int32_t)(token.request_id - e->terminal_request_id[token.owner]) < 0) {
		return LED_STALE;
	}
	enum led_semantic previous = request_only && e->terminal_request_id[token.owner] == token.request_id
								   ? e->terminal_request[token.owner]
								   : (!request_only ? state->terminal : LED_NONE);
	bool conflict = previous != LED_NONE && previous != semantic && is_terminal;
	if (previous != LED_NONE) {
		if (conflict) {
			/* Only a success may be conservatively replaced by failure. */
			if (previous != LED_SUCCESS || !non_success(semantic)) {
				return LED_CONFLICT;
			}
		} else {
			return LED_STALE;
		}
	}
	if (ack_class >= 0) {
		e->last_ack_event_id[token.owner][ack_class] = event_id;
	}
	if (is_terminal) {
		if (request_only) {
			e->terminal_request_id[token.owner] = token.request_id;
			e->terminal_request[token.owner] = semantic;
		} else {
			state->terminal = semantic;
			state->semantic = LED_NONE;
		}
	}
	if (semantic == LED_INPUT_ACK) {
		e->last_input_ms[token.owner] = now_ms;
		e->last_input_request_id[token.owner] = token.request_id;
		e->input_seen[token.owner] = true;
	}
	if (semantic == LED_ACCEPTED && e->input_seen[token.owner]
		&& e->last_input_request_id[token.owner] == token.request_id && now_ms - e->last_input_ms[token.owner] <= 300) {
		return LED_DUPLICATE;
	}
	if (semantic == LED_STAGE_ACK && e->ack_seen[token.owner] && now_ms - e->last_ack_ms[token.owner] < 500) {
		return LED_DUPLICATE;
	}
	if (semantic == LED_STAGE_ACK) {
		e->ack_seen[token.owner] = true;
		e->last_ack_ms[token.owner] = now_ms;
	}
	enum led_admission result = enqueue(e, token, event_id, semantic, now_ms);
	return conflict ? LED_CONFLICT : result;
}
uint32_t led_engine_button_input(struct led_engine *e)
{
	if (++e->button_generation == 0) {
		++e->button_generation;
	}
	e->button_submitted_generation = 0;
	e->button_event.present = false;
	e->button_hold_active = false;
	e->button_hold_retired_generation = 0;
	if (e->active_event.button_count) {
		e->active_event.present = false;
	}
	return e->button_generation;
}
enum led_admission
led_engine_button_hold(
	struct led_engine *e, uint32_t generation, bool active, uint32_t started_ms, uint32_t now_ms
)
{
	if (e->quiesced) {
		return LED_SHUTDOWN;
	}
	if (!generation) {
		return LED_INVALID;
	}
	if (generation != e->button_generation) {
		return LED_STALE;
	}
	if (active && e->button_hold_retired_generation == generation) {
		return LED_STALE;
	}
	if (!active) {
		e->button_hold_retired_generation = generation;
	}
	if (active == e->button_hold_active) {
		return LED_DUPLICATE;
	}
	e->button_hold_active = active;
	if (active) {
		(void)now_ms;
		e->button_hold_origin_ms = started_ms;
		e->button_hold_identity = identity(e);
	}
	return LED_ADMITTED;
}
enum led_admission led_engine_button_exit(
	struct led_engine *e, struct led_token token, uint32_t revision, uint32_t generation, uint32_t now_ms
)
{
	if (e->quiesced) {
		return LED_SHUTDOWN;
	}
	if (!generation) {
		return LED_INVALID;
	}
	if (generation != e->button_generation) {
		return LED_STALE;
	}
	bool inherit = e->button_hold_active && now_ms - e->button_hold_origin_ms >= LED_BUTTON_HOLD_MS;
	uint32_t visual_origin = e->button_hold_origin_ms + LED_BUTTON_HOLD_MS;
	enum led_admission result = led_engine_state(e, token, revision, LED_MANUAL_EXIT, now_ms);
	if (result == LED_ADMITTED) {
		/* Keep the physical threshold's visual age while the business wait
		 * still starts at release. Retirement cannot erase a newer press. */
		if (inherit) {
			e->owners[token.owner].origin_ms = visual_origin;
		}
		(void)led_engine_button_input(e);
	}
	return result;
}
enum led_admission
led_engine_button_group(struct led_engine *e, uint32_t generation, uint32_t count, uint32_t now_ms)
{
	if (e->quiesced) {
		return LED_SHUTDOWN;
	}
	if (manual_exit_active(e)) {
		return LED_STALE;
	}
	if (!generation || !count || count > LED_BUTTON_GROUP_MAX) {
		return LED_INVALID;
	}
	if (generation != e->button_generation) {
		return LED_STALE;
	}
	if (generation == e->button_submitted_generation) {
		return LED_DUPLICATE;
	}
	e->button_submitted_generation = generation;
	e->button_event = (struct led_pending_event){
		.present = true,
		.event_id = generation,
		.semantic = LED_INPUT_ACK,
		.button_count = count,
		.occurred_ms = now_ms,
		.expires_ms = now_ms + count * LED_BUTTON_CYCLE_MS + LED_BUTTON_START_TTL_MS,
		.identity = identity(e)
	};
	return LED_ADMITTED;
}
void led_engine_connection(struct led_engine *e, const struct led_connection_facts *facts)
{
	e->connection = *facts;
	ready_truth(e);
}
void led_engine_maintenance(struct led_engine *e, enum led_owner owner, bool active)
{
	e->owners[owner].maintenance = active;
	ready_truth(e);
}
void led_engine_operation(
	struct led_engine *e,
	enum led_owner owner,
	bool ota_active,
	bool heated_active
)
{
	e->owners[owner].ota_active = ota_active;
	e->owners[owner].heated_active = heated_active;
	ready_truth(e);
}
void led_engine_power(struct led_engine *e, enum led_power_state power, bool low, uint32_t now_ms)
{
	bool effective = low && power != LED_POWER_CHARGING;
	if (effective && !e->low) {
		e->low_due_ms = now_ms;
	}
	if (!effective) {
		e->low_active = false;
	}
	e->low = effective;
	e->power = power;
}
void led_engine_fault(
	struct led_engine *e,
	enum led_owner owner,
	enum led_fault_kind fault,
	uint32_t protection_id,
	uint32_t now_ms
)
{
	if (!led_owner_valid(owner) || fault < LED_FAULT_NONE || fault > LED_FAULT_SAFETY) {
		return;
	}
	struct led_owner_state *state = &e->owners[owner];
	state->fault = fault;
	if (protection_id && protection_id != state->protection_id) {
		state->protection_id = protection_id;
		state->protection_record = true;
		state->protection_expires_ms = now_ms + 6000;
	}
	ready_truth(e);
}
void led_engine_identify(struct led_engine *e, uint32_t now_ms)
{
	if (e->quiesced) {
		return;
	}
	if (!e->identify_active || due(now_ms, e->identify_origin_ms + 6000)) {
		e->identify_active = true;
		e->identify_origin_ms = now_ms;
	}
}
static struct led_selection
selection(enum led_semantic semantic, enum led_owner owner, uint32_t origin, uint32_t id, bool finite, bool network)
{
	const struct led_behavior *behavior = led_behavior_get(semantic);
	return (struct led_selection){
		.semantic = semantic,
		.owner = owner,
		.priority = behavior ? behavior->priority : LED_PRIORITY_BACKGROUND,
		.origin_ms = origin,
		.identity = id,
		.finite = finite,
		.network = network,
		.next_ms = UINT32_MAX
	};
}
static void cap_deadline(struct led_selection *selection, uint32_t now_ms, uint32_t deadline_ms)
{
	uint32_t delta = remaining(now_ms, deadline_ms);
	if (delta < selection->next_ms) {
		selection->next_ms = delta;
	}
}
#ifdef CONFIG_LED_DEBUG
static enum led_semantic lifecycle_blocker(const struct led_engine *e)
{
	if (manual_exit_active(e)) {
		return LED_MANUAL_EXIT;
	}
	if (e->button_hold_active) {
		return LED_BUTTON_HOLD;
	}
	for (enum led_owner owner = LED_OWNER_IMU; owner < LED_OWNER_COUNT; ++owner) {
		if (e->owners[owner].ota_active || e->owners[owner].semantic == LED_OTA_ACTIVE) {
			return LED_OTA_ACTIVE;
		}
	}
	for (enum led_owner owner = LED_OWNER_IMU; owner < LED_OWNER_COUNT; ++owner) {
		if (e->owners[owner].heated_active || e->owners[owner].semantic == LED_HEATED_ACTIVE) {
			return LED_HEATED_ACTIVE;
		}
	}
	return LED_NONE;
}
#endif
static uint32_t event_duration(const struct led_pending_event *event)
{
	return event->button_count ? event->button_count * LED_BUTTON_CYCLE_MS
							  : led_behavior_get(event->semantic)->duration_ms;
}
static uint32_t event_start_gap(const struct led_engine *e, uint32_t now_ms, bool button_count)
{
	return black_gap(e, now_ms, button_count ? 120U : LED_FEEDBACK_START_GAP_MS);
}
static bool event_fits(
	const struct led_engine *e, struct led_pending_event *event, uint32_t now_ms, uint32_t gap
)
{
	uint32_t needed = gap + event_duration(event);
	if (remaining(now_ms, event->expires_ms) < needed) {
		event->present = false;
		return false;
	}
	/* LOW only defers pending events; an unlit active event is retired by its
	 * caller. A lit envelope keeps its original completion/preemption rules. */
	return !e->low || remaining(now_ms, e->low_due_ms) >= needed;
}
static bool event_expired(const struct led_pending_event *event, uint32_t now_ms)
{
	if (!event->present) {
		return true;
	}
	return due(now_ms, event->expires_ms) || (event->playing && due(now_ms, event->origin_ms + event_duration(event)));
}
/* -2 means the active envelope; -1 means none; LED_OWNER_COUNT is button count. */
static int select_event(struct led_engine *e, uint32_t now_ms)
{
	struct led_pending_event *active = &e->active_event;
	if (event_expired(active, now_ms)) {
		active->present = false;
	}
	if (active->present && !active->illuminated) {
		uint32_t gap = event_start_gap(e, now_ms, active->button_count != 0);
		if (!event_fits(e, active, now_ms, gap)) {
			active->present = false;
		} else if (gap) {
			active->origin_ms = now_ms + gap;
		}
	}
	int best = active->present ? -2 : -1;
	for (unsigned slot = 0; slot <= LED_OWNER_COUNT; ++slot) {
		struct led_pending_event *event = slot == LED_OWNER_COUNT ? &e->button_event : &e->events[slot];
		if (event_expired(event, now_ms)) {
			event->present = false;
			continue;
		}
		const struct led_behavior *behavior = led_behavior_get(event->semantic);
		uint32_t gap = event_start_gap(e, now_ms, event->button_count != 0);
		if (!event_fits(e, event, now_ms, gap)) {
			continue;
		}
		if (best == -1) {
			best = (int)slot;
			continue;
		}
		const struct led_pending_event *old
			= best == -2 ? active : best == LED_OWNER_COUNT ? &e->button_event : &e->events[best];
		const struct led_behavior *old_behavior = led_behavior_get(old->semantic);
		if (behavior->result_rank < old_behavior->result_rank
			|| (best != -2 && behavior->result_rank == old_behavior->result_rank
				&& (int32_t)(event->occurred_ms - old->occurred_ms) < 0)) {
			best = (int)slot;
		}
	}
	return best;
}
static void drop_interrupted(struct led_engine *e, const struct led_selection *chosen)
{
	if (e->active_event.present && e->active_event.identity != chosen->identity) {
		e->active_event.present = false;
	}
}
static struct led_selection foreground(struct led_engine *e, uint32_t now_ms)
{
	struct led_selection out = selection(LED_NONE, LED_OWNER_SYSTEM, 0, 0, false, false);
	uint8_t best_rank = UINT8_MAX;
	for (enum led_owner owner = LED_OWNER_IMU; owner < LED_OWNER_COUNT; ++owner) {
		struct led_owner_state *state = &e->owners[owner];
		const struct led_behavior *behavior = led_behavior_get(state->semantic);
		if (behavior && behavior->priority == LED_PRIORITY_TASK) {
			if (state->semantic == LED_INITIALIZING && now_ms - state->origin_ms < 300) {
				continue;
			}
			if (behavior->task_rank < best_rank) {
				best_rank = behavior->task_rank;
				out = selection(
					state->semantic,
					owner,
					state->origin_ms,
					state->identity,
					state->semantic == LED_EXIT_PENDING,
					false
				);
			}
		}
		/* OTA's actual lock includes reboot-pending/error boundaries after its
		 * result event. This fact preserves process visibility until release. */
		if (state->ota_active && 1 < best_rank) {
			best_rank = 1;
			out = selection(LED_OTA_ACTIVE, owner, 0, 0x40000010U + (uint32_t)owner, false, false);
		}
		if (state->maintenance && 5 < best_rank) {
			best_rank = 5;
			out = selection(LED_TEST_ACTIVE, owner, 0, 0x40000000U + (uint32_t)owner, false, false);
		}
	}
	return out;
}
static struct led_selection background(const struct led_engine *e, uint32_t now_ms)
{
	if (e->power != LED_POWER_BATTERY) {
		enum led_semantic power = e->power == LED_POWER_CHARGING ? LED_CHARGING
								: e->power == LED_POWER_CHARGED  ? LED_CHARGED
																 : LED_EXTERNAL_POWER_UNKNOWN;
		return selection(power, LED_OWNER_POWER, 0, 0x50000000U + (uint32_t)power, false, power == LED_CHARGING);
	}
	if (e->connection.pairing) {
		return selection(LED_PAIRING, LED_OWNER_RADIO, 0, 0x70000001, false, false);
	}
	if (e->connection.radio_required && !e->connection.healthy) {
		return selection(
			e->connection.paired ? LED_RECONNECTING : LED_UNPAIRED_IDLE,
			LED_OWNER_RADIO,
			0,
			0x50000002,
			false,
			false
		);
	}
	if (e->ready) {
		return selection(LED_READY, LED_OWNER_SYSTEM, 0, 0x60000000, false, true);
	}
	for (enum led_owner owner = LED_OWNER_IMU; owner < LED_OWNER_COUNT; ++owner) {
		if (e->owners[owner].semantic == LED_INITIALIZING && now_ms - e->owners[owner].origin_ms < 300) {
			return selection(LED_NONE, LED_OWNER_SENSOR, 0, 0, false, false);
		}
	}
	return selection(
		e->connection.radio_required ? LED_PROCESSING : LED_NONE,
		LED_OWNER_SYSTEM,
		0,
		0x50000003,
		false,
		false
	);
}

struct led_selection led_engine_select(struct led_engine *e, uint32_t now_ms)
{
	ready_truth(e);
	struct led_selection out = selection(LED_NONE, LED_OWNER_SYSTEM, 0, 0, false, false);
	if (e->quiesced) {
		out.priority = LED_PRIORITY_SHUTDOWN;
		goto chosen;
	}
	for (enum led_owner owner = LED_OWNER_IMU; owner < LED_OWNER_COUNT; ++owner) {
		struct led_owner_state *state = &e->owners[owner];
		if (state->protection_record && due(now_ms, state->protection_expires_ms)) {
			state->protection_record = false;
		}
		if (state->fault == LED_FAULT_SAFETY || state->protection_record) {
			out = selection(LED_SAFETY_FAULT, owner, 0, 0x10000000U + (uint32_t)owner, false, false);
			goto chosen;
		}
	}
	for (enum led_owner owner = LED_OWNER_IMU; owner < LED_OWNER_COUNT; ++owner) {
		enum led_fault_kind fault = e->owners[owner].fault;
		if (fault != LED_FAULT_NONE && fault != LED_FAULT_SAFETY) {
			enum led_semantic semantic = fault == LED_FAULT_SENSOR_MISSING ? LED_SENSOR_MISSING
										: fault == LED_FAULT_SENSOR ? LED_SENSOR_FAULT : LED_BLOCKING_FAULT;
			out = selection(semantic, owner, 0, 0x20000000U + 8U * (uint32_t)owner + fault, false, false);
			goto chosen;
		}
	}
	if (e->low_active && due(now_ms, e->low_origin_ms + 2000)) {
		e->low_active = false;
	}
	if (e->low && !e->low_active && due(now_ms, e->low_due_ms)) {
		e->low_active = true;
		e->low_origin_ms = now_ms;
		e->low_due_ms = now_ms + 10000;
	}
	if (e->low_active) {
		out = selection(
			LED_LOW_BATTERY,
			LED_OWNER_POWER,
			e->low_origin_ms,
			0x30000000U + e->low_origin_ms,
			true,
			false
		);
		cap_deadline(&out, now_ms, e->low_origin_ms + 2000);
		goto chosen;
	}
	/* Manual handoff is exclusive only below fault, LOW and shutdown. Its original
	 * wall-clock budget never restarts after a safety preemption. */
	if (manual_exit_active(e)) {
		const struct led_owner_state *state = &e->owners[LED_OWNER_SYSTEM];
		out = selection(LED_MANUAL_EXIT, LED_OWNER_SYSTEM, state->origin_ms, state->identity, true, false);
		goto chosen;
	}
	/* Only physical press feedback has this exception. Ordinary task state,
	 * result admission/coalescing and preview policy remain unchanged. */
	if (e->button_hold_active) {
		out = selection(
			LED_BUTTON_HOLD, LED_OWNER_SYSTEM, e->button_hold_origin_ms, e->button_hold_identity, false, false
		);
		out.button_generation = e->button_generation;
		goto chosen;
	}
	int event_owner = select_event(e, now_ms);
	if (event_owner != -1) {
		if (event_owner >= 0) {
			struct led_pending_event *pending
				= event_owner == LED_OWNER_COUNT ? &e->button_event : &e->events[event_owner];
			e->active_event = *pending;
			pending->present = false;
			e->active_owner = event_owner == LED_OWNER_COUNT ? LED_OWNER_SYSTEM : (enum led_owner)event_owner;
			e->active_event.playing = true;
			e->active_event.origin_ms = now_ms
				+ event_start_gap(e, now_ms, e->active_event.button_count != 0);
		}
		struct led_pending_event *event = &e->active_event;
		out = selection(event->semantic, e->active_owner, event->origin_ms, event->identity, true, false);
		out.button_count = event->button_count;
		if ((int32_t)(now_ms - event->origin_ms) < 0) {
			cap_deadline(&out, now_ms, event->origin_ms);
		}
		goto chosen;
	}
	if (e->identify_active && due(now_ms, e->identify_origin_ms + 6000)) {
		e->identify_active = false;
	}
	if (e->identify_active) {
		out = selection(
			LED_IDENTIFY,
			LED_OWNER_RADIO,
			e->identify_origin_ms,
			0x70000000U + e->identify_origin_ms,
			true,
			false
		);
		cap_deadline(&out, now_ms, e->identify_origin_ms + 6000);
		goto chosen;
	}
#ifdef CONFIG_LED_DEBUG
	if (e->preview.status == LED_PREVIEW_ACTIVE) {
		if (lifecycle_blocker(e) != LED_NONE) {
			e->preview.status = LED_PREVIEW_PREEMPTED;
			e->preview.blocker = lifecycle_blocker(e);
		} else if (due(now_ms, e->preview.expires_ms)) {
			e->preview.status = LED_PREVIEW_EXPIRED;
		} else {
			const struct led_behavior *behavior = led_behavior_get(e->preview.semantic);
			bool finite = behavior && behavior->extent == LED_FINITE;
			if (!e->preview_started) {
				uint32_t gap = finite && behavior->priority == LED_PRIORITY_EVENT
					? event_start_gap(e, now_ms, false) : 0;
				if (finite && remaining(now_ms, e->preview.expires_ms) < gap + behavior->duration_ms) {
					e->preview.status = LED_PREVIEW_EXPIRED;
				} else {
					e->preview_started = true;
					e->preview_play_origin_ms = now_ms + gap;
				}
			}
			if (e->preview.status == LED_PREVIEW_ACTIVE && finite && (int32_t)(now_ms - e->preview_play_origin_ms) >= 0
				&& due(now_ms, e->preview_play_origin_ms + behavior->duration_ms)) {
				e->preview.status = LED_PREVIEW_COMPLETED;
			}
			if (e->preview.status == LED_PREVIEW_ACTIVE) {
				out = selection(
					e->preview.semantic,
					LED_OWNER_SYSTEM,
					e->preview_play_origin_ms,
					0x80000000U + e->preview.generation,
					finite,
					false
				);
				out.priority = LED_PRIORITY_IDENTIFY; /* Synthetic source follows real IDENTIFY. */
				e->real_winner = foreground(e, now_ms);
				if (e->real_winner.semantic == LED_NONE) {
					e->real_winner = background(e, now_ms);
				}
				cap_deadline(&out, now_ms, e->preview.expires_ms);
				goto chosen;
			}
		}
	}
#endif
	out = foreground(e, now_ms);
	if (out.semantic != LED_NONE) {
		goto chosen;
	}
	out = background(e, now_ms);
chosen:
	if (out.priority <= LED_PRIORITY_FAULT && e->low) {
		e->low_active = false;
		e->low_due_ms = now_ms;
	}
	drop_interrupted(e, &out);
#ifdef CONFIG_LED_DEBUG
	if (e->preview.status == LED_PREVIEW_ACTIVE
		&& (out.priority <= LED_PRIORITY_IDENTIFY || out.button_generation || out.semantic == LED_MANUAL_EXIT)
		&& out.identity != 0x80000000U + e->preview.generation) {
		e->preview.status = LED_PREVIEW_PREEMPTED;
		e->preview.blocker = out.semantic;
	}
#endif
	if (out.identity != e->visible_identity) {
		e->visible_identity = out.identity;
		e->visible_origin_ms = now_ms;
	}
	/* Wall-clock sources retain their original phase; other continuous states
	 * restart only on visibility changes, never on repeated revisions. */
	bool event = out.priority == LED_PRIORITY_EVENT;
	bool wall = out.semantic == LED_IDENTIFY || out.semantic == LED_EXIT_PENDING || out.semantic == LED_MANUAL_EXIT
			 || out.semantic == LED_BUTTON_HOLD || out.semantic == LED_LOW_BATTERY || out.network;
#ifdef CONFIG_LED_DEBUG
	wall |= e->preview.status == LED_PREVIEW_ACTIVE && out.identity == 0x80000000U + e->preview.generation;
#endif
	if (!event && !wall && out.semantic != LED_NONE) {
		out.origin_ms = e->visible_origin_ms;
	}
	if (e->low && !e->low_active && out.priority > LED_PRIORITY_LOW) {
		cap_deadline(&out, now_ms, e->low_due_ms);
	}
	for (enum led_owner owner = LED_OWNER_IMU; owner < LED_OWNER_COUNT; ++owner) {
		const struct led_owner_state *state = &e->owners[owner];
		if (state->protection_record) {
			cap_deadline(&out, now_ms, state->protection_expires_ms);
		}
		if (state->semantic == LED_INITIALIZING && now_ms - state->origin_ms < 300) {
			cap_deadline(&out, now_ms, state->origin_ms + 300);
		}
	}
#ifdef CONFIG_LED_DEBUG
	if (!(e->preview.status == LED_PREVIEW_ACTIVE && out.identity == 0x80000000U + e->preview.generation)) {
		e->real_winner = out;
	}
#endif
	e->winner = out;
	return out;
}
void led_engine_black(struct led_engine *e, uint32_t now_ms, bool black, bool success)
{
	if (!success) {
		e->black_known = false;
		return;
	}
	if (black) {
		if (!e->black_known) {
			e->black_since_ms = now_ms;
		}
		e->black_known = true;
	} else {
		e->black_known = false;
		if (e->winner.priority == LED_PRIORITY_EVENT && led_owner_valid(e->winner.owner)) {
			struct led_pending_event *event = &e->active_event;
			if (event->present && event->identity == e->winner.identity) {
				event->illuminated = true;
			}
		}
	}
}
#ifdef CONFIG_LED_DEBUG
void led_engine_view(const struct led_engine *e, uint32_t now_ms, struct led_policy_view *view)
{
	*view = (struct led_policy_view){
		.winner = e->real_winner,
		.low_due_ms = e->low_due_ms,
		.quiesced = e->quiesced,
		.ready = e->ready,
		.low = e->low,
		.low_active = e->low_active,
		.black_since_ms = e->black_since_ms
	};
	for (enum led_owner owner = LED_OWNER_IMU; owner < LED_OWNER_COUNT; ++owner) {
		view->event_remaining_ms[owner] = e->events[owner].present ? remaining(now_ms, e->events[owner].expires_ms) : 0;
		if (e->active_event.present && e->active_owner == owner) {
			uint32_t active_remaining = remaining(now_ms, e->active_event.expires_ms);
			if (active_remaining > view->event_remaining_ms[owner]) {
				view->event_remaining_ms[owner] = active_remaining;
			}
		}
		view->ota_active |= e->owners[owner].ota_active || e->owners[owner].semantic == LED_OTA_ACTIVE;
		view->heated_active |= e->owners[owner].heated_active || e->owners[owner].semantic == LED_HEATED_ACTIVE;
	}
	if (e->button_event.present) {
		uint32_t button_remaining = remaining(now_ms, e->button_event.expires_ms);
		if (button_remaining > view->event_remaining_ms[LED_OWNER_SYSTEM]) {
			view->event_remaining_ms[LED_OWNER_SYSTEM] = button_remaining;
		}
	}
}
enum led_preview_result led_engine_preview(
	struct led_engine *e,
	uint32_t console_session,
	enum led_semantic semantic,
	int color_override,
	uint32_t duration_ms,
	uint32_t now_ms,
	const struct led_hardware_info *hardware,
	bool color_supported,
	struct led_preview_record *record
)
{
	const struct led_behavior *behavior = led_behavior_get(semantic);
	enum led_preview_result result = LED_PREVIEW_ADMITTED;
	if (e->quiesced) {
		result = LED_PREVIEW_SHUTDOWN;
	} else if (
		!console_session || console_session != e->console_session
		|| (color_override < 0 ? !behavior : color_override > LED_PHYSICAL_WHITE)
	) {
		result = LED_PREVIEW_INVALID;
	} else if (hardware->capability == LED_CAP_NO_LED || (color_override >= 0 && !color_supported)) {
		result = LED_PREVIEW_UNSUPPORTED;
	} else if (lifecycle_blocker(e) != LED_NONE || (e->low && (e->low_active || due(now_ms, e->low_due_ms)))) {
		result = LED_PREVIEW_BUSY;
	} else {
		for (enum led_owner owner = LED_OWNER_IMU; owner < LED_OWNER_COUNT; ++owner) {
			if (e->owners[owner].fault != LED_FAULT_NONE
				|| (e->owners[owner].protection_record && !due(now_ms, e->owners[owner].protection_expires_ms))
				|| (e->events[owner].present && !event_expired(&e->events[owner], now_ms))) {
				result = LED_PREVIEW_BUSY;
				break;
			}
		}
		if (e->identify_active && !due(now_ms, e->identify_origin_ms + 6000)) {
			result = LED_PREVIEW_BUSY;
		}
		if (e->active_event.present && !event_expired(&e->active_event, now_ms)) {
			result = LED_PREVIEW_BUSY;
		}
	}
	bool finite = color_override < 0 && behavior && behavior->extent == LED_FINITE;
	if (result == LED_PREVIEW_ADMITTED
		&& ((finite && duration_ms != 0) || (!finite && (duration_ms < 250 || duration_ms > 30000)))) {
		result = LED_PREVIEW_INVALID;
	}
	if (result == LED_PREVIEW_ADMITTED) {
		if (++e->preview_generation == 0) {
			++e->preview_generation;
		}
		e->preview = (struct led_preview_record){
			.generation = e->preview_generation,
			.console_session = console_session,
			.semantic = semantic,
			.color_override = color_override,
			.origin_ms = now_ms,
			.expires_ms = now_ms + (finite ? behavior->duration_ms + LED_FEEDBACK_START_GAP_MS + 500U : duration_ms),
			.status = LED_PREVIEW_ACTIVE,
			.result = result,
			.muted = hardware->global_limit_pptt == 0
		};
		e->preview_started = false;
		*record = e->preview;
	} else {
		*record = (struct led_preview_record){
			.semantic = semantic,
			.color_override = color_override,
			.result = result,
			.blocker = lifecycle_blocker(e) != LED_NONE ? lifecycle_blocker(e) : e->real_winner.semantic,
			.muted = hardware->global_limit_pptt == 0
		};
	}
	return result;
}
#endif
