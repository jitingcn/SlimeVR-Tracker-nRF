#ifndef SLIMENRF_LED_POLICY_H
#define SLIMENRF_LED_POLICY_H
#include "led_internal.h"

struct led_owner_state {
	uint32_t session;
	uint32_t request_id;
	uint32_t revision;
	uint32_t origin_ms;
	uint32_t identity;
	enum led_semantic semantic;
	enum led_semantic terminal;
	enum led_fault_kind fault;
	uint32_t protection_id;
	uint32_t protection_expires_ms;
	bool maintenance;
	bool ota_active;
	bool heated_active;
	bool protection_record;
};
struct led_pending_event {
	uint32_t session;
	uint32_t request_id;
	uint32_t event_id;
	uint32_t occurred_ms;
	uint32_t expires_ms;
	uint32_t origin_ms;
	uint32_t identity;
	enum led_semantic semantic;
	uint32_t button_count; /* zero for ordinary command/result events */
	bool present;
	bool playing;
	bool illuminated;
};
struct led_engine {
	struct led_owner_state owners[LED_OWNER_COUNT];
	struct led_pending_event events[LED_OWNER_COUNT];
	/* One displayed result is separate from one pending slot per owner. Equal
	 * class arrivals cannot destroy a currently visible finite envelope. */
	struct led_pending_event active_event;
	enum led_owner active_owner;
	/* One whole count train, not one queued event per physical click. */
	struct led_pending_event button_event;
	uint32_t button_generation;
	uint32_t button_submitted_generation;
	uint32_t button_hold_retired_generation;
	bool button_hold_active;
	uint32_t button_hold_origin_ms;
	uint32_t button_hold_identity;
	uint32_t last_ack_event_id[LED_OWNER_COUNT][3];
	uint32_t last_ack_ms[LED_OWNER_COUNT];
	uint32_t last_input_ms[LED_OWNER_COUNT];
	uint32_t last_input_request_id[LED_OWNER_COUNT];
	bool ack_seen[LED_OWNER_COUNT];
	bool input_seen[LED_OWNER_COUNT];
	uint32_t terminal_request_id[LED_OWNER_COUNT];
	enum led_semantic terminal_request[LED_OWNER_COUNT];
	struct led_connection_facts connection;
	enum led_power_state power;
	bool low;
	bool low_active;
	bool quiesced;
	bool ready;
	bool identify_active;
	bool black_known;
	uint32_t low_due_ms;
	uint32_t low_origin_ms;
	uint32_t identify_origin_ms;
	uint32_t black_since_ms;
	uint32_t identity_counter;
	uint32_t visible_identity;
	uint32_t visible_origin_ms;
	struct led_selection winner;
#ifdef CONFIG_LED_DEBUG
	struct led_selection real_winner;
	struct led_preview_record preview;
	uint32_t preview_generation;
	uint32_t console_session;
	bool preview_started;
	uint32_t preview_play_origin_ms;
#endif
};
void led_engine_init(struct led_engine *engine, uint32_t now_ms);
struct led_token led_engine_begin(struct led_engine *engine, enum led_owner owner, uint32_t request_id);
enum led_admission led_engine_state(
	struct led_engine *engine,
	struct led_token token,
	uint32_t revision,
	enum led_semantic semantic,
	uint32_t now_ms
);
enum led_admission led_engine_event(
	struct led_engine *engine,
	struct led_token token,
	uint32_t event_id,
	enum led_semantic semantic,
	uint32_t now_ms,
	bool request_only
);
uint32_t led_engine_button_input(struct led_engine *engine);
enum led_admission
led_engine_button_group(struct led_engine *engine, uint32_t generation, uint32_t count, uint32_t now_ms);
enum led_admission
led_engine_button_hold(
	struct led_engine *engine, uint32_t generation, bool active, uint32_t started_ms, uint32_t now_ms
);
enum led_admission led_engine_button_exit(
	struct led_engine *engine, struct led_token token, uint32_t revision, uint32_t generation, uint32_t now_ms
);
void led_engine_connection(struct led_engine *engine, const struct led_connection_facts *facts);
void led_engine_maintenance(struct led_engine *engine, enum led_owner owner, bool active);
void led_engine_operation(
	struct led_engine *engine,
	enum led_owner owner,
	bool ota_active,
	bool heated_active
);
void led_engine_power(struct led_engine *engine, enum led_power_state power, bool low, uint32_t now_ms);
void led_engine_fault(
	struct led_engine *engine,
	enum led_owner owner,
	enum led_fault_kind fault,
	uint32_t protection_id,
	uint32_t now_ms
);
void led_engine_identify(struct led_engine *engine, uint32_t now_ms);
/* Advance only from worker. Queries read winner/deadlines without consuming. */
struct led_selection led_engine_select(struct led_engine *engine, uint32_t now_ms);
void led_engine_black(struct led_engine *engine, uint32_t now_ms, bool black, bool success);
#ifdef CONFIG_LED_DEBUG
void led_engine_view(const struct led_engine *engine, uint32_t now_ms, struct led_policy_view *view);
enum led_preview_result led_engine_preview(
	struct led_engine *engine,
	uint32_t console_session,
	enum led_semantic semantic,
	int color_override,
	uint32_t duration_ms,
	uint32_t now_ms,
	const struct led_hardware_info *hardware,
	bool color_supported,
	struct led_preview_record *record
);
#endif
#endif
