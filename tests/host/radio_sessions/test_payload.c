#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include "../harness/include/atomic.h"

#define K_FOREVER 0
#define CONFIG_CLOCK_CONTROL_NRF 1
#define NRF_CLOCK_HAS_PLL 0
#define CLOCK_CONTROL_NRF_SUBSYS_HF 0
#define SYS_INIT(...)
#define ARG_UNUSED(value) ((void)(value))
#define K_MUTEX_DEFINE(name) int name
#define K_SEM_DEFINE(name, initial, maximum) struct k_sem name = {initial, maximum}
struct k_sem { unsigned count, limit; };
static inline void k_sem_give(struct k_sem *sem)
{
	assert(sem->count < sem->limit);
	++sem->count;
}
static inline int k_sem_take(struct k_sem *sem, int timeout)
{
	assert(sem->count > 0);
	--sem->count;
	return 0;
}
static int esb_radio_lock;
static void k_mutex_lock(int *lock, int timeout) { ++*lock; }
static void k_mutex_unlock(int *lock) { assert(*lock > 0); --*lock; }
static bool channel_wait_normal, channel_search;
static atomic_t channel_redirect_pending, channel_found;
static bool channel_confirmed = true, channel_legacy_peer, ping_channel_confirm_sent;
static bool esb_ota_is_active(void) { return false; }
static inline int atomic_inc(atomic_t *value) { return __atomic_fetch_add(value, 1, __ATOMIC_SEQ_CST); }
static inline int atomic_dec(atomic_t *value) { return __atomic_fetch_sub(value, 1, __ATOMIC_SEQ_CST); }
#define IS_ENABLED(option) 0
enum tdma_ping_admission { TDMA_PING_UNAVAILABLE, TDMA_PING_DEFERRED, TDMA_PING_ADMITTED };
static void clocks_stop(void);
static void host_tx_success(void);

#define CONFIG_CONNECTION_TDMA 1
#define CONFIG_ESB_MAX_PAYLOAD_LENGTH 64
#define ESB_PING_TYPE 0xF0
static void host_log(const char *format, ...)
{
}
#define LOG_DBG(...) host_log(__VA_ARGS__)
#define LOG_INF(...) host_log(__VA_ARGS__)
#define LOG_WRN(...) host_log(__VA_ARGS__)
#define LOG_ERR(...) host_log(__VA_ARGS__)
#define ESB_CREATE_PAYLOAD(...) {0}
#define ESB_ST_PAIRING 0
#define ENOMEM_ERROR_WINDOW_MS 1000
#define ENOMEM_ERROR_THRESHOLD 10
#define PING_HISTORY_SIZE 8
struct esb_payload {
	uint8_t pipe, noack, length, data[64];
};
static bool esb_initialized = true, server_time_synced, radio_user_disabled;
extern bool clock_status;
static int esb_conn_state = 1;
static uint8_t tracker_id = 2, ping_counter = 7, ping_ctr_sent;
static bool ping_pending, ping_failed;
static struct {
	uint8_t type;
	bool noack;
	size_t length;
	int64_t timestamp;
} last_tx;
struct ping_history_entry {
	uint8_t counter;
	uint32_t ping_ticks, ping_ticks_kernel;
	bool valid;
};
static struct ping_history_entry ping_history[PING_HISTORY_SIZE];
static unsigned ping_history_idx, consecutive_enomem_errors;
static unsigned esb_write_dropped, esb_write_dup_queued, esb_write_queued;
static int64_t last_enomem_time, last_tx_time, ping_send_time;
static int64_t now_ms = 1000;
static bool batch, tdma_enabled, idle = true;
static uint32_t ping_failures;
static void esb_clear_time_sync_state(void) { server_time_synced = false; }
static void tdma_set_enabled(bool enabled) { tdma_enabled = enabled; }
static void connection_request_ping_resync(void) {}
static int hook_site, queue_calls, queue_failures;
static unsigned irq_depth;
static bool defer_pong_on_queue, pong_waiting, coherent_pong;
static struct esb_payload queued[8];
static unsigned queued_count;
static unsigned tx_success_count, clock_releases;
static bool finish_during_wait, deny_admission, check_tx_clock, check_ping_window;
static int clock_start_error, sync_age = -1;
struct onoff_manager { int unused; };
struct onoff_client;
typedef void (*notify_callback)(struct onoff_manager *, struct onoff_client *, uint32_t, int);
struct sys_notify { int result; notify_callback callback; };
struct onoff_client { struct sys_notify notify; };
static struct onoff_manager mock_clock_manager;
static struct onoff_manager *z_nrf_clock_control_get_onoff(int subsystem)
{
	return &mock_clock_manager;
}
static inline void sys_notify_init_spinwait(struct sys_notify *notify)
{
	notify->result = 0;
	notify->callback = NULL;
}
static inline void sys_notify_init_callback(struct sys_notify *notify, notify_callback callback)
{
	notify->callback = callback;
}
static inline int sys_notify_fetch_result(struct sys_notify *notify, int *result)
{
	*result = notify->result;
	return 0;
}
static int onoff_request(struct onoff_manager *manager, struct onoff_client *client)
{
	if (clock_start_error) {
		return clock_start_error;
	}
	now_ms += 5; /* Cold HFXO startup may exceed a guarded admission window. */
	client->notify.result = 0;
	/* A ready onoff manager may invoke the completion inline. */
	if (client->notify.callback) {
		client->notify.callback(manager, client, 0, 0);
	}
	return 0;
}
static int onoff_release(struct onoff_manager *manager)
{
	clock_releases++;
	return 0;
}
static inline void k_yield(void) {}
int esb_write(uint8_t *data, bool no_ack, size_t length);
static void interleave(int site)
{
	if (hook_site != site) {
		return;
	}
	hook_site = 0;
	uint8_t other[] = {0x70, 0x91, 0x82, 0x73, 0x64};
	assert(esb_write(other, false, sizeof(other)) == 0);
}
/* Clock ownership and admission use the extracted production implementation. */
static bool connection_get_data_collection(void)
{
	return false;
}
static void drop_failed_tx_payload_if_pending(void)
{
}
static bool connection_get_data_collection_batch(void)
{
	return batch;
}
static void esb_write_rate_tick(void)
{
}
struct esb_network_clock {
	uint32_t receiver_ticks;
	uint64_t local_kernel_ticks;
	int64_t sync_age_ms;
};
static bool esb_get_network_clock(struct esb_network_clock *clock)
{
	clock->receiver_ticks = 123;
	clock->local_kernel_ticks = 0;
	clock->sync_age_ms = sync_age;
	return server_time_synced;
}
static uint8_t crc8_ccitt(uint8_t seed, const uint8_t *data, size_t size)
{
	return seed;
}
static int64_t k_uptime_get(void)
{
	return now_ms;
}
static uint64_t k_uptime_ticks(void)
{
	return (uint64_t)now_ms * 32;
}
static uint64_t k_ticks_to_ms_floor64(uint64_t ticks) { return ticks / 32; }
static uint64_t net_ticks_from_kernel64(uint64_t ticks)
{
	return ticks;
}
static bool tdma_is_enabled(void)
{
	return tdma_enabled;
}
static int esb_get_sync_age_ms(void)
{
	return sync_age;
}
static bool esb_is_idle(void)
{
	return idle;
}
static void tdma_note_radio_busy(void)
{
}
static bool tdma_wait_for_slot(uint8_t size)
{
	if (finish_during_wait) {
		finish_during_wait = false;
		host_tx_success();
	}
	interleave(1);
	return !deny_admission;
}
static enum tdma_ping_admission tdma_wait_for_ping_window(void)
{
	if (deny_admission) {
		return TDMA_PING_DEFERRED;
	}
	/* Model a 1ms guarded window every 10ms, without delaying to rescue
	 * transmissions after admission. Only the admission owner may wait. */
	now_ms += (10 - now_ms % 10) % 10;
	return TDMA_PING_ADMITTED;
}
static bool esb_ready(void)
{
	return true;
}
static uint32_t k_cycle_get_32(void)
{
	return 500;
}
static void k_usleep(uint32_t us)
{
	interleave(2);
}
static void k_msleep(int ms)
{
	now_ms += ms;
	interleave(3);
}
static int esb_write_payload(const struct esb_payload *payload)
{
	queue_calls++;
	if (queue_failures) {
		queue_failures--;
		return -ENOMEM;
	}
	assert(queued_count < 8);
	queued[queued_count++] = *payload;
	if (defer_pong_on_queue && payload->data[0] == ESB_PING_TYPE) {
		assert(irq_depth != 0);
		defer_pong_on_queue = false;
		pong_waiting = true;
	}
	return 0;
}
static void esb_start_queued_tx(void)
{
	if (check_tx_clock) {
		assert(clock_status);
	}
	if (check_ping_window) {
		assert(now_ms % 10 == 0);
	}
	if (check_tx_clock) {
		idle = false;
	}
}
static int esb_flush_tx(void)
{
	return 0;
}
static int esb_suspend(void)
{
	return 0;
}
static void esb_deinitialize(void)
{
}
static int esb_initialize(bool enabled)
{
	return 0;
}
static unsigned irq_lock(void)
{
	return irq_depth++;
}
static void irq_unlock(unsigned key)
{
	assert(irq_depth == key + 1);
	irq_depth = key;
	if (irq_depth == 0 && pong_waiting) {
		const struct esb_payload *payload = &queued[queued_count - 1];
		unsigned index = (ping_history_idx + PING_HISTORY_SIZE - 1) % PING_HISTORY_SIZE;
		assert(ping_pending && ping_ctr_sent == payload->data[2]);
		assert(ping_send_time == now_ms && ping_history[index].valid);
		assert(ping_history[index].counter == ping_ctr_sent);
		assert(ping_history[index].ping_ticks == (uint32_t)now_ms * 32);
		assert(ping_history[index].ping_ticks_kernel == (uint32_t)now_ms * 32);
		pong_waiting = false;
		ping_pending = false;
		coherent_pong = true;
	}
}
#include "payload.inc"

static void reset(void)
{
	queued_count = 0;
	queue_calls = 0;
	queue_failures = 0;
	defer_pong_on_queue = pong_waiting = coherent_pong = false;
	assert(irq_depth == 0);
	batch = false;
	tdma_enabled = false;
	idle = true;
	clock_start_error = 0;
	clocks_stop();
	assert(!clock_status);
	assert(clocks_init() == 0);
	assert(clocks_start() == 0);
	assert(clock_status);
	clock_releases = 0;
	now_ms = 1000;
	finish_during_wait = false;
	deny_admission = false;
	check_tx_clock = false;
	check_ping_window = false;
	sync_age = -1;
	esb_initialized = true;
	radio_user_disabled = false;
	esb_conn_state = 1;
	channel_wait_normal = channel_search = false; atomic_set(&channel_redirect_pending, false);
	channel_confirmed = true; channel_legacy_peer = ping_channel_confirm_sent = false;
	memset(&wom_command, 0, sizeof(wom_command));
}
static void cold_clock(void)
{
	clocks_stop();
	assert(!clock_status);
}
static void assert_original(unsigned index, const uint8_t *data, size_t length, bool noack)
{
	assert(queued[index].length == length);
	assert(queued[index].pipe == 3);
	assert(queued[index].noack == noack);
	assert(memcmp(queued[index].data, data, length) == 0);
}
static void clock_lifetime(void)
{
	uint8_t mag[17] = {4, 2};
	reset();
	tdma_enabled = true;
	finish_during_wait = true;
	check_tx_clock = true;

	int err = esb_write(mag, true, sizeof(mag));

	assert(err == 0);
	assert(queued_count == 1);
	assert(clock_status);

	idle = true;
	host_tx_success();

	assert(!clock_status);
	assert(clock_releases == 1);
	puts("clock lifetime: prior TX completion cannot disable a waiting successor");
}

static void cold_ping(void)
{
	uint8_t ping[13] = {0xF0};
	reset();
	cold_clock();
	tdma_enabled = true;
	sync_age = 0;
	check_tx_clock = true;
	check_ping_window = true;

	int err = host_send_ping(ping, false);

	assert(err == 0);
	assert(queued_count == 1);
	assert(clock_status);
	puts("cold PING: HFXO startup precedes guarded admission");
}

static void deferred_data_releases_clock(void)
{
	uint8_t mag[17] = {4, 2};
	reset();
	tdma_enabled = true;
	deny_admission = true;

	int err = esb_write(mag, true, sizeof(mag));

	assert(err == -EAGAIN);
	assert(!clock_status);
	assert(queued_count == 0);
}

static void deferred_ping_keeps_clock_warm(void)
{
	uint8_t ping[13] = {0xF0};
	reset();
	cold_clock();
	deny_admission = true;

	int err = host_send_ping(ping, false);

	assert(err == -EAGAIN);
	assert(clock_status);
	assert(queued_count == 0);

	err = host_send_ping(ping, false);

	assert(err == -EAGAIN);
	assert(clock_status);
	assert(queued_count == 0);

	deny_admission = false;
	check_tx_clock = true;
	check_ping_window = true;
	err = host_send_ping(ping, false);

	assert(err == 0);
	assert(queued_count == 1);
	assert(clock_status);

	idle = true;
	host_tx_success();

	assert(!clock_status);
}

static void deferred_ping_allows_explicit_clock_stop(void)
{
	uint8_t ping[13] = {0xF0};
	reset();
	cold_clock();
	deny_admission = true;

	int err = host_send_ping(ping, false);
	clocks_stop();

	assert(err == -EAGAIN);
	assert(!clock_status);
	assert(queued_count == 0);
}

static void clock_start_failure_does_not_queue(void)
{
	uint8_t mag[17] = {4, 2};
	reset();
	cold_clock();
	clock_start_error = -EIO;

	int err = esb_write(mag, true, sizeof(mag));

	assert(err == -EIO);
	assert(!clock_status);
	assert(queued_count == 0);
}

static void inactive_radio_does_not_start_clock(void)
{
	uint8_t ping[13] = {0xF0};
	reset();
	cold_clock();
	esb_initialized = false;

	int err = host_send_ping(ping, false);

	assert(err == -EACCES);
	assert(!clock_status);
	assert(queued_count == 0);
}

static void forced_resync_bypasses_admission(void)
{
	uint8_t ping[13] = {0xF0};
	reset();
	cold_clock();
	deny_admission = true;
	check_tx_clock = true;

	int err = host_send_ping(ping, true);

	assert(err == 0);
	assert(queued_count == 1);
	assert(clock_status);

	idle = true;
	host_tx_success();

	assert(!clock_status);
}

static void clock_errors(void)
{
	deferred_data_releases_clock();
	deferred_ping_keeps_clock_warm();
	deferred_ping_allows_explicit_clock_stop();
	clock_start_failure_does_not_queue();
	inactive_radio_does_not_start_clock();
	forced_resync_bypasses_admission();
	puts("clock errors: deferred, startup failure and inactive paths release ownership; resync transmits");
}

static void channel_proof_ping_preserves_arguments(void)
{
	const uint8_t commands[] = {ESB_PONG_FLAG_TEST_MODE_ON,
		ESB_PONG_FLAG_SET_CHANNEL, ESB_PONG_FLAG_DATA_COLLECT_METADATA};
	for (unsigned command = 0; command < sizeof(commands); ++command) {
		for (unsigned proof = 0; proof < 5; ++proof) {
			reset();
			channel_wait_normal = proof != 0;
			channel_search = proof == 2;
			atomic_set(&channel_redirect_pending, proof == 3);
			channel_confirmed = proof == 0 || proof == 4;
			channel_legacy_peer = proof == 1;
			uint8_t ping[13] = {ESB_PING_TYPE};
			ping[7] = commands[command];
			const uint8_t arguments[] = {0x12, 0x34, 0x56, 0x78};
			memcpy(&ping[8], arguments, sizeof(arguments));
			assert(host_send_ping(ping, true) == 0);
			assert(queued_count == 1 && queued[0].length == 13);
			bool requested = proof == 2 || proof == 3;
			assert(queued[0].data[7] == (requested ? ESB_PING_FLAG_CHANNEL_CONFIRM : commands[command]));
			assert(ping_channel_confirm_sent == requested);
			assert(memcmp(&queued[0].data[8], arguments, sizeof(arguments)) == 0);
			assert(queued[0].data[12] == crc8_ccitt(7, queued[0].data, 12));
		}
	}
}

static void fw_info_before_normal(void)
{
	uint8_t info[13] = {ESB_OTA_FW_INFO_TYPE};
	uint8_t pose[17] = {1};
	for (unsigned legacy = 0; legacy < 2; ++legacy) {
		reset();
		channel_wait_normal = true;
		channel_confirmed = !legacy;
		channel_legacy_peer = legacy;
		assert(esb_write(pose, true, sizeof(pose)) == -EAGAIN);
		assert(queue_calls == 0);
		assert(esb_write(info, false, sizeof(info)) == 0);
		assert_original(0, info, sizeof(info), false);
		assert(channel_wait_normal);
	}
	reset();
	channel_confirmed = false;
	channel_wait_normal = true;
	uint8_t ping[13] = {ESB_PING_TYPE};
	queue_failures = 2;
	ping_channel_confirm_sent = false;
	assert(esb_write_ping(ping, true) != 0);
	assert(!ping_channel_confirm_sent); /* Failed admission is not a request witness. */
}

static void restart_blocks_ordinary_writer(void)
{
	reset();
	tdma_enabled = true;
	server_time_synced = true;
	channel_confirmed = true; atomic_set(&channel_found, true);
	assert(host_receiver_restart() == 1);
	assert(channel_wait_normal && !atomic_get(&channel_found) && !channel_confirmed);
	assert(!tdma_enabled && !server_time_synced);
	uint8_t pose[17] = {1};
	assert(esb_write(pose, true, sizeof(pose)) == -EAGAIN);
	assert(queue_calls == 0 && queued_count == 0);
	uint8_t ping[13] = {ESB_PING_TYPE};
	assert(esb_write_ping(ping, true) == 0);
	assert(queued_count == 1 && queued[0].data[7] == ESB_PING_FLAG_CHANNEL_CONFIRM);
}

static void wom_response_payload(void)
{
	for (unsigned proof = 0; proof < 2; ++proof) {
		for (unsigned outcome = 0; outcome < 4; ++outcome) {
			reset();
			channel_confirmed = !proof;
			wom_command.cached = wom_command.response_pending = true;
			wom_command.cached_token = 0x4567;
			wom_command.result = outcome == 0 ? 35791 : outcome == 1 ? EIO
				: outcome == 2 ? 0 : ESB_WOM_ENTER;
			wom_command.result_flag = outcome == 1 ? ESB_PING_FLAG_WOM_ERROR
				: ESB_PING_FLAG_WOM_RESULT;
			uint8_t ping[13] = {ESB_PING_TYPE};
			ping[7] = ESB_PING_FLAG_WOM_RESULT;
			/* Deliberately stale caller bytes: the real writer must snapshot
			 * completed flag/result/token together before its real CRC. */
			memset(&ping[8], 0xab, 4);
			assert(host_send_ping(ping, true) == 0);
			assert(queued_count == 1 && queued[0].length == 13);
			assert(queued[0].data[12] == crc8_ccitt(7, queued[0].data, 12));
			if (proof) {
				assert(queued[0].data[7] == ESB_PING_FLAG_CHANNEL_CONFIRM);
				assert(wom_command.sent_token == 0 && wom_command.response_pending);
			} else {
				assert(queued[0].data[7] == wom_command.result_flag);
				assert(queued[0].data[8] == (wom_command.result >> 8));
				assert(queued[0].data[9] == (uint8_t)wom_command.result);
				assert(queued[0].data[10] == 0x45 && queued[0].data[11] == 0x67);
				assert(wom_command.sent_token == 0x4567);
				assert(wom_command.sent_flag == wom_command.result_flag);
			}
		}
	}
}

static void ping_publication(void)
{
	uint8_t ping[13] = {ESB_PING_TYPE};
	reset();
	hook_site = 0;
	idle = false;
	defer_pong_on_queue = true;
	uint8_t counter = ping_counter;
	assert(esb_write(ping, false, sizeof(ping)) == 0);
	assert(coherent_pong && !ping_pending && ping_ctr_sent == counter);

	reset();
	hook_site = 0;
	ping_pending = true;
	ping_failed = false;
	ping_ctr_sent = 41;
	ping_send_time = now_ms - 200;
	counter = ping_counter;
	unsigned index = ping_history_idx;
	queue_failures = 2;
	assert(esb_write(ping, false, sizeof(ping)) == -ENOMEM);
	assert(ping_pending && !ping_failed && ping_ctr_sent == 41);
	assert(ping_send_time == now_ms - 200 && ping_counter == counter && ping_history_idx == index);
	puts("payload: PONG sees one complete PING and failed replacement preserves the pending PING");
}

int main(void)
{
	const char *scenario = getenv("RADIO_SCENARIO");
	if (scenario) {
		if (strcmp(scenario, "clock") == 0) {
			clock_lifetime();
		} else if (strcmp(scenario, "cold-ping") == 0) {
			cold_ping();
		} else if (strcmp(scenario, "clock-errors") == 0) {
			clock_errors();
		} else {
			return 2;
		}
		return 0;
	}
	uint8_t ordinary[] = {1, 2, 3, 4, 5, 6};
	for (int site = 1; site <= 2; site++) {
		reset();
		hook_site = site;
		tdma_enabled = site == 1;
		assert(esb_write(ordinary, true, sizeof(ordinary)) == 0);
		assert(queued_count == 2);
		assert_original(1, ordinary, sizeof(ordinary), true);
	}
	/* Reliable raw retry sleeps with FIFO full; its duplicate must retain bytes. */
	uint8_t raw[] = {0x10, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
	reset();
	hook_site = 3;
	idle = false;
	queue_failures = 1;
	assert(esb_write(raw, false, sizeof(raw)) == 0);
	assert(queued_count == 3 && queue_calls == 4);
	assert_original(1, raw, sizeof(raw), false);
	assert_original(2, raw, sizeof(raw), true);
	/* Post-wait PING history and timestamp decisions must use this call's bytes. */
	uint8_t ping[13] = {0xF0};
	reset();
	hook_site = 3;
	idle = false;
	queue_failures = 1;
	unsigned index = ping_history_idx;
	assert(esb_write(ping, false, sizeof(ping)) == 0);
	assert_original(1, ping, sizeof(ping), false);
	assert(ping_history[index].counter == 7 && ping_ctr_sent == 7);
	assert(ping_history[index].ping_ticks != 0 && ping_history_idx == index + 1);
	puts("payload: TDMA, jitter, FIFO retry, duplicate and PING history remain call-local");
	clock_lifetime();
	cold_ping();
	clock_errors();
	reset();
	channel_wait_normal = true;
	assert(esb_write(ordinary, true, sizeof(ordinary)) == -EAGAIN);
	assert(queue_calls == 0);
	assert(esb_write_ping(ping, true) == 0);
	channel_wait_normal = false;
	reset();
	assert(esb_write(ordinary, true, sizeof(ordinary)) == 0);
	assert_original(0, ordinary, sizeof(ordinary), true);
	channel_proof_ping_preserves_arguments();
	fw_info_before_normal();
	restart_blocks_ordinary_writer();
	wom_response_payload();
	ping_publication();
	return 0;
}
