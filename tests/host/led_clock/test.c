#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#if KERNEL_HZ == 31250
#define CONFIG_SOC_SERIES_NRF54L 1
#endif
#define SYS_STATUS_CONNECTION_ERROR 0x20
#define ESB_ST_PAIRED 1
static uint64_t now_ticks;
static bool locked, change_on_unlock;
static bool server_time_synced;
static int esb_conn_state, status_bits;
static int64_t g_last_sync_timestamp;
static uint32_t g_server_ticks_offset, g_last_sync_local_ticks;
static int32_t g_clock_skew_ppb;
static uint32_t tdma_runtime_enabled, tdma_cfg_pack;
static uint32_t atomic_get(const uint32_t *value) { return *value; }
static unsigned irq_lock(void) { assert(!locked); locked = true; return 0; }
static void irq_unlock(unsigned key)
{
	(void)key;
	assert(locked);
	locked = false;
	if (change_on_unlock) {
		server_time_synced = false;
		g_server_ticks_offset = 99;
		g_last_sync_local_ticks = 0;
		g_clock_skew_ppb = 0;
	}
}
static uint64_t k_uptime_ticks(void) { assert(locked); return now_ticks; }
static int get_status(int mask) { assert(locked); return status_bits & mask; }
static uint64_t k_ticks_to_us_near64(uint64_t ticks)
{
	assert(!locked);
	return (ticks * 1000000ULL + KERNEL_HZ / 2) / KERNEL_HZ;
}
static uint64_t k_ticks_to_ms_floor64(uint64_t ticks)
{
	assert(!locked);
	return ticks * 1000ULL / KERNEL_HZ;
}
#include "production.inc"

static void reset(void)
{
	now_ticks = 100ULL * KERNEL_HZ;
	server_time_synced = true;
	esb_conn_state = ESB_ST_PAIRED;
	status_bits = 0;
	g_last_sync_timestamp = 100000;
	g_server_ticks_offset = 1000;
	g_last_sync_local_ticks = 100U * 32768;
	g_clock_skew_ppb = 0;
	tdma_runtime_enabled = 1;
	tdma_cfg_pack = 1U | (22U << 8) | (220U << 16);
	change_on_unlock = false;
}
static void unavailable(void)
{
	uint32_t local = 0, network = 0;
	assert(!esb_get_status_clock(&local, &network));
	assert(local == (uint32_t)net_ticks_from_kernel64(now_ticks));
	assert(network == local);
}
int main(void)
{
	reset();
#if CONFIG_CONNECTION_TDMA
	uint32_t local, network;
	assert(esb_get_status_clock(&local, &network));
	assert(local == 3276800 && network == local + 1000);
	g_server_ticks_offset = 0U - local;
	assert(esb_get_status_clock(&local, &network) && network == 0);
	reset(); status_bits = SYS_STATUS_CONNECTION_ERROR; unavailable();
	reset(); esb_conn_state = 2; unavailable();
	reset(); server_time_synced = false; unavailable();
	reset(); tdma_runtime_enabled = 0; unavailable();
	reset(); tdma_cfg_pack = 0; unavailable();
	reset(); tdma_cfg_pack = 10U | (22U << 8) | (220U << 16); unavailable();
	reset(); tdma_cfg_pack = (15U << 8) | (150U << 16); unavailable();
	reset(); g_last_sync_timestamp = 100001; unavailable();
	reset(); g_last_sync_timestamp -= TDMA_SYNC_STALE_MS;
	assert(esb_get_status_clock(&local, &network));
	--g_last_sync_timestamp; unavailable();
	/* Accepted sync at uptime zero is not confused with no sync. */
	reset(); now_ticks = 0; g_last_sync_timestamp = 0;
	assert(esb_get_status_clock(&local, &network) && local == 0);
	/* Wrap-safe elapsed time with both signs of drift, and network wrap. */
	reset(); g_last_sync_local_ticks -= 65536; g_clock_skew_ppb = 1000000;
	assert(esb_get_status_clock(&local, &network) && network == local + 1065);
	g_clock_skew_ppb = -1000000;
	assert(esb_get_status_clock(&local, &network) && network == local + 935);
	reset(); now_ticks = (1ULL << 32) + 2ULL * KERNEL_HZ;
	g_last_sync_timestamp = now_ticks * 1000ULL / KERNEL_HZ;
	uint32_t expected_local = (uint32_t)net_ticks_from_kernel64(now_ticks);
	g_last_sync_local_ticks = expected_local - 65536;
	g_clock_skew_ppb = 1000000;
	g_server_ticks_offset = UINT32_MAX - expected_local;
	assert(esb_get_status_clock(&local, &network));
	assert(local == expected_local && network == 64);
	/* Radio updates immediately after unlock cannot mix estimator epochs. */
	reset(); g_last_sync_local_ticks -= 65536; g_clock_skew_ppb = 1000000;
	change_on_unlock = true;
	assert(esb_get_status_clock(&local, &network) && network == local + 1065);
#else
	unavailable();
#endif
	printf("status clock: kernel=%d TDMA=%d passed\n", KERNEL_HZ, CONFIG_CONNECTION_TDMA);
}
