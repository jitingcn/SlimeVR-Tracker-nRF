#include "globals.h"
#include <zephyr/kernel.h>
#include <limits.h>
#include "led_policy.h"
#include "led_sync.h"
#if CONFIG_LED_NETWORK_SYNC
#include "connection/esb.h"
#endif

LOG_MODULE_REGISTER(led, LOG_LEVEL_INF);

/* Existing worker is the only driver/rail owner. Business calls copy bounded
 * scalar facts under this lock; no driver, wait or formatting occurs inside. */
static struct led_engine engine;
static struct k_spinlock led_lock;
static bool initialized;
static bool hardware_quiesced;
static uint32_t request_counter, event_counter;
#ifdef CONFIG_LED_DEBUG
static enum led_time_source clock_source;
#endif
K_SEM_DEFINE(led_changed, 0, 1);
K_SEM_DEFINE(led_quiesced, 0, 1);
static void led_thread(void);
K_THREAD_DEFINE(led_thread_id, CONFIG_LED_THREAD_STACK_SIZE, led_thread, NULL, NULL, NULL, LED_THREAD_PRIORITY, 0, 0);

static uint32_t local_ms(void)
{
	return (uint32_t)k_uptime_get();
}
static void init_locked(uint32_t now_ms)
{
	if (!initialized) {
		led_engine_init(&engine, now_ms);
		initialized = true;
	}
}
static uint32_t next_id(uint32_t *counter)
{
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	if (++*counter == 0) {
		++*counter;
	}
	uint32_t value = *counter;
	k_spin_unlock(&led_lock, key);
	return value;
}
uint32_t led_request_id(void)
{
	return next_id(&request_counter);
}
uint32_t led_event_id(void)
{
	return next_id(&event_counter);
}
struct led_token led_begin(enum led_owner owner, uint32_t request_id)
{
	uint32_t now_ms = local_ms();
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(now_ms);
	uint32_t previous = led_owner_valid(owner) ? engine.owners[owner].session : 0;
	struct led_token token = led_engine_begin(&engine, owner, request_id);
	k_spin_unlock(&led_lock, key);
	if (token.session && token.session != previous) {
		k_sem_give(&led_changed);
	}
	return token;
}
enum led_admission led_state(struct led_token token, uint32_t revision, enum led_semantic semantic)
{
	uint32_t now_ms = local_ms();
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(now_ms);
	enum led_admission result = led_engine_state(&engine, token, revision, semantic, now_ms);
	k_spin_unlock(&led_lock, key);
	if (result == LED_ADMITTED) {
		k_sem_give(&led_changed);
	}
	return result;
}
static enum led_admission
result_submit(struct led_token token, uint32_t event_id, enum led_semantic semantic, bool request_only)
{
	uint32_t now_ms = local_ms();
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(now_ms);
	enum led_admission result = led_engine_event(&engine, token, event_id, semantic, now_ms, request_only);
	k_spin_unlock(&led_lock, key);
	if (result == LED_CONFLICT) {
		LOG_ERR("LED owner %u request %u has contradictory terminal %u", token.owner, token.request_id, semantic);
	}
	if (result == LED_ADMITTED || result == LED_CONFLICT) {
		k_sem_give(&led_changed);
	}
	return result;
}
enum led_admission led_result(struct led_token token, uint32_t event_id, enum led_semantic semantic)
{
	return result_submit(token, event_id, semantic, false);
}
enum led_admission
led_request_event(enum led_owner owner, uint32_t request_id, uint32_t event_id, enum led_semantic semantic)
{
	struct led_token token = {.owner = owner, .request_id = request_id};
	return result_submit(token, event_id, semantic, true);
}
uint32_t led_button_input(void)
{
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(local_ms());
	bool changed = engine.button_hold_active || engine.button_event.present
				 || (engine.active_event.present && engine.active_event.button_count);
	uint32_t generation = led_engine_button_input(&engine);
	k_spin_unlock(&led_lock, key);
	if (changed) {
		k_sem_give(&led_changed);
	}
	return generation;
}
enum led_admission led_button_group(uint32_t generation, uint32_t count)
{
	uint32_t now_ms = local_ms();
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(now_ms);
	enum led_admission result = led_engine_button_group(&engine, generation, count, now_ms);
	k_spin_unlock(&led_lock, key);
	if (result == LED_ADMITTED) {
		k_sem_give(&led_changed);
	}
	return result;
}
enum led_admission led_button_hold(uint32_t generation, bool active, uint32_t started_ms)
{
	uint32_t now_ms = local_ms();
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(now_ms);
	enum led_admission result = led_engine_button_hold(&engine, generation, active, started_ms, now_ms);
	k_spin_unlock(&led_lock, key);
	if (result == LED_ADMITTED) {
		k_sem_give(&led_changed);
	}
	return result;
}
enum led_admission led_button_exit(struct led_token token, uint32_t revision, uint32_t generation)
{
	uint32_t now_ms = local_ms();
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(now_ms);
	enum led_admission result = led_engine_button_exit(&engine, token, revision, generation, now_ms);
	k_spin_unlock(&led_lock, key);
	if (result == LED_ADMITTED) {
		k_sem_give(&led_changed);
	}
	return result;
}
void led_identify(void)
{
	uint32_t now_ms = local_ms();
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(now_ms);
	bool changed
		= !engine.quiesced && (!engine.identify_active || (int32_t)(now_ms - (engine.identify_origin_ms + 6000)) >= 0);
	led_engine_identify(&engine, now_ms);
	k_spin_unlock(&led_lock, key);
	if (changed) {
		k_sem_give(&led_changed);
	}
}
void led_connection_publish(const struct led_connection_facts *facts)
{
	if (!facts) {
		return;
	}
	uint32_t now_ms = local_ms();
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(now_ms);
	const struct led_connection_facts *old = &engine.connection;
	bool changed = !engine.quiesced
				&& (facts->healthy != old->healthy || facts->output_ready != old->output_ready
					|| facts->radio_required != old->radio_required || facts->paired != old->paired
					|| facts->pairing != old->pairing);
	if (changed) {
		led_engine_connection(&engine, facts);
	}
	k_spin_unlock(&led_lock, key);
	if (changed) {
		k_sem_give(&led_changed);
	}
}
void led_power_publish(enum led_power_state state, bool low)
{
	if (state < LED_POWER_BATTERY || state > LED_POWER_EXTERNAL_UNKNOWN) {
		return;
	}
	uint32_t now_ms = local_ms();
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(now_ms);
	bool changed = !engine.quiesced && (engine.power != state || engine.low != (low && state != LED_POWER_CHARGING));
	if (changed) {
		led_engine_power(&engine, state, low, now_ms);
	}
	k_spin_unlock(&led_lock, key);
	if (changed) {
		k_sem_give(&led_changed);
	}
}
void led_fault_publish(enum led_owner owner, enum led_fault_kind fault, uint32_t protection_id)
{
	/* Reject before computing changes: an invalid fault must not wake the
	 * worker and consume or expire otherwise untouched pending feedback. */
	if (!led_owner_valid(owner) || fault < LED_FAULT_NONE || fault > LED_FAULT_SAFETY) {
		return;
	}
	uint32_t now_ms = local_ms();
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(now_ms);
	const struct led_owner_state *state = &engine.owners[owner];
	bool changed = !engine.quiesced
				&& (state->fault != fault || (protection_id && protection_id != state->protection_id));
	if (changed) {
		led_engine_fault(&engine, owner, fault, protection_id, now_ms);
	}
	k_spin_unlock(&led_lock, key);
	if (changed) {
		k_sem_give(&led_changed);
	}
}
void led_maintenance_publish(enum led_owner owner, bool active)
{
	if (!led_owner_valid(owner)) {
		return;
	}
	uint32_t now_ms = local_ms();
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(now_ms);
	bool changed = !engine.quiesced && engine.owners[owner].maintenance != active;
	if (changed) {
		led_engine_maintenance(&engine, owner, active);
	}
	k_spin_unlock(&led_lock, key);
	if (changed) {
		k_sem_give(&led_changed);
	}
}
void led_operation_publish(enum led_owner owner, bool ota_active, bool heated_active)
{
	if (!led_owner_valid(owner)) {
		return;
	}
	uint32_t now_ms = local_ms();
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(now_ms);
	bool changed
		= !engine.quiesced
	   && (engine.owners[owner].ota_active != ota_active || engine.owners[owner].heated_active != heated_active);
	if (changed) {
		led_engine_operation(&engine, owner, ota_active, heated_active);
	}
	k_spin_unlock(&led_lock, key);
	if (changed) {
		k_sem_give(&led_changed);
	}
}
void led_quiesce(void)
{
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(local_ms());
	bool changed = !engine.quiesced;
	engine.quiesced = true;
	engine.button_hold_active = false;
	k_spin_unlock(&led_lock, key);
	if (changed) {
		k_sem_give(&led_changed);
	}
}
bool led_output_enabled(void)
{
	struct led_hardware_info hardware;
	led_hw_info(&hardware);
	return hardware.capability != LED_CAP_NO_LED && hardware.global_limit_pptt != 0;
}
void led_shutdown(void)
{
	led_quiesce();
	if (!led_output_enabled()) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	bool done = hardware_quiesced;
	k_spin_unlock(&led_lock, key);
	/* Bounded existing black/gate handshake, not an animation drain. A platform
	 * driver stuck beyond its proven bound cannot delay the device shutdown. */
	if (!done) {
		(void)k_sem_take(&led_quiesced, K_MSEC(50));
	}
}
#ifdef CONFIG_LED_DEBUG
void led_policy_snapshot(struct led_policy_view *view)
{
	if (!view) {
		return;
	}
	uint32_t now_ms = local_ms();
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	/* Strictly readonly: status cannot admit/expire events or change phases. */
	led_engine_view(&engine, now_ms, view);
	view->time_source = clock_source;
	k_spin_unlock(&led_lock, key);
}
enum led_preview_result led_preview_start(
	uint32_t console_session,
	enum led_semantic semantic,
	int color_override,
	uint32_t duration_ms,
	struct led_preview_record *record
)
{
	if (!record) {
		return LED_PREVIEW_INVALID;
	}
	struct led_hardware_info hardware;
	led_hw_info(&hardware);
	bool color_supported = color_override < 0 || led_hw_color_supported(color_override);
	uint32_t now_ms = local_ms();
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(now_ms);
	enum led_preview_result result = led_engine_preview(
		&engine,
		console_session,
		semantic,
		color_override,
		duration_ms,
		now_ms,
		&hardware,
		color_supported,
		record
	);
	k_spin_unlock(&led_lock, key);
	if (result == LED_PREVIEW_ADMITTED) {
		k_sem_give(&led_changed);
	}
	return result;
}
void led_preview_stop(void)
{
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	if (engine.preview.status == LED_PREVIEW_ACTIVE) {
		engine.preview.status = LED_PREVIEW_STOPPED;
	}
	k_spin_unlock(&led_lock, key);
	k_sem_give(&led_changed);
}
void led_preview_session_start(uint32_t session)
{
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(local_ms());
	if (session != engine.console_session && engine.preview.status == LED_PREVIEW_ACTIVE) {
		engine.preview.status = LED_PREVIEW_STOPPED;
	}
	engine.console_session = session;
	k_spin_unlock(&led_lock, key);
	k_sem_give(&led_changed);
}
void led_preview_disconnect(uint32_t session)
{
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	if (engine.console_session == session) {
		engine.console_session = 0;
	}
	if (engine.preview.status == LED_PREVIEW_ACTIVE && engine.preview.console_session == session) {
		engine.preview.status = LED_PREVIEW_STOPPED;
	}
	k_spin_unlock(&led_lock, key);
	k_sem_give(&led_changed);
}
void led_preview_snapshot(struct led_preview_record *record)
{
	if (!record) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	*record = engine.preview;
	k_spin_unlock(&led_lock, key);
}
#endif

static uint32_t led_clock_raw(void)
{
	uint32_t local = led_sync_kernel_ticks(k_uptime_ticks(), CONFIG_SYS_CLOCK_TICKS_PER_SEC);
#if CONFIG_LED_NETWORK_SYNC
	static bool have_offset;
	static uint32_t held_offset;
	uint32_t network;
	bool fresh = esb_get_status_clock(&local, &network);
	if (fresh) {
		held_offset = network - local;
		have_offset = true;
	}
#ifdef CONFIG_LED_DEBUG
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	clock_source = fresh ? LED_TIME_NETWORK_FRESH : have_offset ? LED_TIME_NETWORK_HELD : LED_TIME_LOCAL;
	k_spin_unlock(&led_lock, key);
#endif
	return have_offset ? local + held_offset : local;
#else
	return local;
#endif
}
/* A single bounded coordinator iteration is also the host smoke surface. */
static uint32_t led_worker_step(void)
{
	uint64_t uptime_ms = (uint64_t)k_uptime_get();
	uint32_t now_ms = (uint32_t)uptime_ms;
	k_spinlock_key_t key = k_spin_lock(&led_lock);
	init_locked(now_ms);
#ifdef CONFIG_LED_DEBUG
	clock_source = LED_TIME_LOCAL;
#endif
	struct led_selection chosen = led_engine_select(&engine, now_ms);
	k_spin_unlock(&led_lock, key);
	if (chosen.priority == LED_PRIORITY_SHUTDOWN) {
		led_hw_off();
		key = k_spin_lock(&led_lock);
		hardware_quiesced = true;
		led_engine_black(&engine, local_ms(), true, true);
		k_spin_unlock(&led_lock, key);
		k_sem_give(&led_quiesced);
		return UINT32_MAX;
	}
	struct led_hardware_info hardware;
	led_hw_info(&hardware);
	if (hardware.capability == LED_CAP_NO_LED || hardware.global_limit_pptt == 0) {
		led_hw_off();
		/* Invisible animations neither hold power nor schedule frame edges. A
		 * muted preview still has a single hard-deadline bookkeeping wake. */
#ifdef CONFIG_LED_DEBUG
		key = k_spin_lock(&led_lock);
		uint32_t muted_wait = UINT32_MAX;
		if (engine.preview.status == LED_PREVIEW_ACTIVE) {
			const struct led_behavior *preview_behavior = led_behavior_get(engine.preview.semantic);
			if (engine.preview.color_override < 0 && preview_behavior && preview_behavior->extent == LED_FINITE) {
				engine.preview.status = LED_PREVIEW_COMPLETED;
			} else {
				muted_wait = engine.preview.expires_ms - now_ms;
			}
		}
		k_spin_unlock(&led_lock, key);
		return muted_wait;
#else
		return UINT32_MAX;
#endif
	}
	const struct led_behavior *behavior = led_behavior_get(chosen.semantic);
	int color_override = -1;
#ifdef CONFIG_LED_DEBUG
	key = k_spin_lock(&led_lock);
	if (engine.preview.status == LED_PREVIEW_ACTIVE && chosen.identity == 0x80000000U + engine.preview.generation) {
		color_override = engine.preview.color_override;
	}
	k_spin_unlock(&led_lock, key);
#endif
	uint32_t wait_ms = chosen.next_ms;
	struct led_envelope envelope = {.next_ms = UINT32_MAX};
	enum led_role role = behavior ? behavior->role : LED_ROLE_NEUTRAL;
	uint16_t level = behavior ? behavior->level : LED_LEVEL_NOTICE;
	bool before_origin = !chosen.network && chosen.finite && (int32_t)(now_ms - chosen.origin_ms) < 0;
	if (color_override >= 0) {
		envelope.value_pptt = 10000;
	} else if (behavior && !before_origin) {
		uint32_t elapsed = now_ms - chosen.origin_ms;
		if (chosen.network) {
			uint32_t raw = led_clock_raw();
			elapsed = led_sync_phase_ms(raw, behavior->duration_ms);
			uint32_t wrap = led_sync_wrap_ms(raw);
			if (wrap < wait_ms) {
				wait_ms = wrap;
			}
#if CONFIG_LED_NETWORK_SYNC
			if (wait_ms > 250) {
				wait_ms = 250;
			}
#endif
		}
		envelope = chosen.button_count ? led_timeline_button_count(chosen.button_count, elapsed)
									 : led_timeline_behavior(behavior, elapsed, hardware.gpio, chosen.finite);
	}
	if (before_origin && chosen.origin_ms - now_ms < wait_ms) {
		wait_ms = chosen.origin_ms - now_ms;
	}
	if (envelope.next_ms < wait_ms) {
		wait_ms = envelope.next_ms;
	}
	if (envelope.fading) {
		/* Producer wakes must not shift the next absolute frame-grid deadline;
		 * pixel BREATHE carry is spent once per successful visible slot. */
		uint32_t frame_wait = LED_FRAME_MS - uptime_ms % LED_FRAME_MS;
		if (frame_wait < wait_ms) {
			wait_ms = frame_wait;
		}
	}
	/* Recheck irreversible shutdown and cancelled button selections before the
	 * driver call. Hardware remains worker-only and outside the facts lock. */
	key = k_spin_lock(&led_lock);
	bool shutdown = engine.quiesced;
	bool cancelled = chosen.button_count
		&& (!engine.active_event.present || engine.active_event.identity != chosen.identity
			|| engine.active_event.event_id != engine.button_generation);
	cancelled |= chosen.button_generation
		&& (!engine.button_hold_active || chosen.button_generation != engine.button_generation
			|| chosen.identity != engine.button_hold_identity);
	k_spin_unlock(&led_lock, key);
	if (shutdown || cancelled) {
		return 0;
	}
	struct led_fade_sample fade;
	const struct led_fade_sample *fade_sample = NULL;
	if (hardware.capability == LED_CAP_RGB_PIXEL && behavior && envelope.fading && color_override < 0
		&& (behavior->style == LED_STYLE_BREATHE || behavior->style == LED_STYLE_BUTTON_HOLD
			|| behavior->style == LED_STYLE_MANUAL_EXIT)) {
		fade.source = ((uint64_t)chosen.owner << 48) | ((uint64_t)chosen.semantic << 32) | chosen.identity;
		fade.origin_ms = chosen.origin_ms;
		fade.slot = (uint32_t)(uptime_ms / LED_FRAME_MS);
		fade_sample = &fade;
	}
	bool success = led_hw_write(role, level, envelope.value_pptt, color_override, fade_sample);
	key = k_spin_lock(&led_lock);
	led_engine_black(&engine, local_ms(), envelope.value_pptt == 0, success);
	k_spin_unlock(&led_lock, key);
	/* At most two bounded retries per unchanged failed frame. New semantic
	 * identity resets budget; a static broken driver never spins forever. */
	static uint32_t failed_identity;
	static uint8_t failures;
	if (success) {
		failures = 0;
	} else {
		if (failed_identity != chosen.identity) {
			failed_identity = chosen.identity;
			failures = 0;
		}
		if (++failures < 3 && wait_ms > 20) {
			wait_ms = 20;
		}
	}
	return wait_ms;
}
static void led_thread(void)
{
	led_hw_init();
	for (;;) {
		uint32_t start = local_ms();
		uint32_t wait_ms = led_worker_step();
		/* The segment deadline includes driver elapsed time, never delay+cost. */
		if (wait_ms != UINT32_MAX) {
			uint32_t spent = local_ms() - start;
			wait_ms = spent >= wait_ms ? 0 : wait_ms - spent;
		}
		(void)k_sem_take(&led_changed, wait_ms == UINT32_MAX ? K_FOREVER : K_MSEC(wait_ms));
	}
}
