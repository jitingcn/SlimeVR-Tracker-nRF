#ifndef SLIMENRF_SYSTEM_LED
#define SLIMENRF_SYSTEM_LED

#include <stdbool.h>
#include <stdint.h>

/* Fixed display ownership; order is the deterministic same-class tie break. */
enum led_owner {
	LED_OWNER_IMU,
	LED_OWNER_ACC,
	LED_OWNER_MAG,
	LED_OWNER_SENS,
	LED_OWNER_TCAL,
	LED_OWNER_SENSOR,
	LED_OWNER_RADIO,
	LED_OWNER_SYSTEM,
	LED_OWNER_POWER,
	LED_OWNER_COUNT
};

enum led_semantic {
	LED_NONE,
	LED_INPUT_ACK,
	LED_ACCEPTED,
	LED_STAGE_ACK,
	LED_CANCELLED,
	LED_SUCCESS,
	LED_REJECTED,
	LED_FAILED,
	LED_PARTIAL,
	LED_APPLIED_NOT_SAVED,
	LED_PAIRING,
	LED_RECONNECTING,
	LED_UNPAIRED_IDLE,
	LED_READY,
	LED_WAIT_STILL,
	LED_COLLECT_STILL,
	LED_WAIT_MOVE,
	LED_COLLECT_MOVE,
	LED_PROCESSING,
	LED_OTA_ACTIVE,
	LED_HEATED_ACTIVE,
	LED_MAINTENANCE,
	LED_TEST_ACTIVE,
	LED_INITIALIZING,
	LED_CHARGING,
	LED_CHARGED,
	LED_EXTERNAL_POWER_UNKNOWN,
	LED_LOW_BATTERY,
	LED_SENSOR_MISSING,
	LED_SENSOR_FAULT,
	LED_BLOCKING_FAULT,
	LED_SAFETY_FAULT,
	LED_IDENTIFY,
	LED_EXIT_PENDING,
	LED_BUTTON_HOLD,
	LED_MANUAL_EXIT,
	LED_SEMANTIC_COUNT
};

/* Admission is local bookkeeping, never proof of display or business success. */
enum led_admission { LED_ADMITTED, LED_DUPLICATE, LED_STALE, LED_INVALID, LED_SHUTDOWN, LED_CONFLICT };
struct led_token {
	enum led_owner owner;
	uint32_t session;
	uint32_t request_id;
};

/* Milliseconds are local monotonic uptime. Allocate request/event identities
 * with these helpers and retain them across retries; counters skip zero and
 * comparisons use signed modular differences. No submission calls a driver. */
uint32_t led_request_id(void);
uint32_t led_event_id(void);
/* Call only AFTER business admission. A new session invalidates that owner's old
 * state/results. Carry this token through asynchronous apply/persist callbacks. */
struct led_token led_begin(enum led_owner owner, uint32_t request_id);
/* revision must increase within a session. LED_NONE releases only this token's
 * state. Repeated snapshots do not restart an animation. */
enum led_admission led_state(struct led_token token, uint32_t revision, enum led_semantic semantic);
/* Fact timestamps and TTL are assigned at submission; retained event_id
 * deduplicates retries. A terminal releases matching state and occurs once.
 * Contradictory terminals return LED_CONFLICT and preserve non-success. */
enum led_admission led_result(struct led_token token, uint32_t event_id, enum led_semantic semantic);
/* Rejection before admission preserves the active session. INPUT_ACK is also a
 * request-only event; do not begin a session for a button candidate. */
enum led_admission
led_request_event(enum led_owner owner, uint32_t request_id, uint32_t event_id, enum led_semantic semantic);
/* Button count feedback is separate from command/result events. A validated
 * physical candidate cancels the previous train and returns its generation.
 * Publish the completed group only after the existing quiet window and while
 * no new press is held. Stale generations cannot resurrect cancelled counts.
 * Every click owns 80ms on + 420ms black (2Hz), including the final black. The only
 * count limit is signed uptime32 deadline arithmetic; larger counts are
 * rejected, never truncated. Results and P0/safety/fault/LOW may interrupt. */
#define LED_BUTTON_CYCLE_MS 500U
#define LED_BUTTON_START_TTL_MS 500U
#define LED_BUTTON_GROUP_MAX ((UINT32_C(2147483647) - LED_BUTTON_START_TTL_MS) / LED_BUTTON_CYCLE_MS)
uint32_t led_button_input(void);
enum led_admission led_button_group(uint32_t generation, uint32_t count);
/* Physical-press feedback uses the actual local uptime32 press stamp, not the
 * delayed producer publication time. Repeated active generation never restarts
 * the full-on/1s marker/linear fade/held blink sequence. Inactive ignores
 * started_ms. Stale generations cannot resurrect cancelled feedback. */
#define LED_BUTTON_HOLD_MS 1000U
enum led_admission led_button_hold(uint32_t generation, bool active, uint32_t started_ms);
/* Button-only generation-safe handoff. A qualified active physical press keeps
 * its threshold-origin marker/fade phase; no release replay. Admitted handoff
 * retires that generation atomically. Nonbutton exits use admission origin. */
enum led_admission led_button_exit(struct led_token token, uint32_t revision, uint32_t generation);
/* Configured possibility of light only (capability and nonzero global cap),
 * not driver readiness, physical delivery, or a reservation of output. */
bool led_output_enabled(void);
/* Reversible business wait remains 1800ms from release/admission, independent
 * of the 1500ms marker/linear-fade/black visual and any inherited visual age.
 * Automatic/protective/OTA transitions never wait for this envelope. */
#define LED_MANUAL_EXIT_MS 1800U
void led_identify(void); /* merge into an existing six-second wall-clock window */

/* Complete owner facts, not inversions of suppressed error flags. No LED policy
 * changes thresholds, sleep/radio decisions, or charger interpretation. */
struct led_connection_facts {
	bool healthy;
	bool output_ready;
	bool radio_required;
	bool paired;
	bool pairing;
};
void led_connection_publish(const struct led_connection_facts *facts);
enum led_power_state { LED_POWER_BATTERY, LED_POWER_CHARGING, LED_POWER_CHARGED, LED_POWER_EXTERNAL_UNKNOWN };
void led_power_publish(enum led_power_state state, bool low);
enum led_fault_kind {
	LED_FAULT_NONE,
	LED_FAULT_SENSOR_MISSING,
	LED_FAULT_SENSOR,
	LED_FAULT_SYSTEM,
	LED_FAULT_SAFETY
};
/* Typed local cause changes no protocol bits. A new nonzero protection_id
 * records six seconds from actual protection; repeats never extend it. Clear
 * current safety independently; the record keeps its own deadline. */
void led_fault_publish(enum led_owner owner, enum led_fault_kind fault, uint32_t protection_id);
/* Persistent maintenance facts coexist with a same-owner finite command/session.
 * Call on actual test/raw/batch/capture application, not transport admission. */
void led_maintenance_publish(enum led_owner owner, bool active);
/* Explicit lifecycle facts survive reservation/finalizing/reboot-pending even
 * when another display state wins. OTA also preserves its P6 process while
 * reboot-pending after a terminal; heated reservation alone is not RUNNING. */
void led_operation_publish(enum led_owner owner, bool ota_active, bool heated_active);
/* Irreversible P0 gate. Submission returns immediately, never waits for an
 * animation. led_shutdown is the bounded worker black/power-gate handshake used
 * by existing shutdown paths; black-write errors cannot prevent power gating. */
void led_quiesce(void);
void led_shutdown(void);

#endif
