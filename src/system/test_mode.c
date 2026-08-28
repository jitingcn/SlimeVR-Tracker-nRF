#include "test_mode.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#if defined(CONFIG_CONNECTION_TDMA)
#include "../connection/tdma.h"
#endif

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(test_mode, LOG_LEVEL_INF);

static atomic_t test_mode_active;
/* 0 = built-in default (TEST_MODE_MIN_SEND_INTERVAL_MS); else 1..max_tps */
static atomic_t test_mode_target_tps;

bool test_mode_get(void)
{
	return atomic_get(&test_mode_active) != 0;
}

void test_mode_set(bool enable)
{
	bool was_enabled = atomic_set(&test_mode_active, enable ? 1 : 0) != 0;
	if (was_enabled != enable) {
		LOG_INF("Test mode %s", enable ? "ENABLED" : "DISABLED");
	}
}

void test_mode_set_target_tps(uint16_t tps)
{
	atomic_set(&test_mode_target_tps, tps); /* 0 = reset to default; clamped at read */
	LOG_INF("Test mode target TPS set to %u", tps);
}

uint16_t test_mode_get_target_tps(void)
{
	return (uint16_t)atomic_get(&test_mode_target_tps);
}

#define TEST_MODE_RATE_QUANTUM_TPS 10U

uint16_t test_mode_effective_tps(void)
{
	if (!test_mode_get()) {
		return 0;
	}
	uint16_t tps = test_mode_get_target_tps();
	if (tps == 0) {
		tps = 1000U / TEST_MODE_MIN_SEND_INTERVAL_MS;
	}
	uint16_t max_tps = 1000;
#if defined(CONFIG_CONNECTION_TDMA)
	if (tdma_is_enabled()) {
		uint16_t frame_ticks = tdma_frame_ticks_get();
		if (frame_ticks > 0) {
			uint16_t capacity_tps = (uint16_t)(32768U / frame_ticks);
			/* Round capacity down to 10 TPS: 8 trackers 256->250,
			 * 10 trackers 204->200. The remainder covers PING guards. */
			max_tps = (capacity_tps / TEST_MODE_RATE_QUANTUM_TPS)
				* TEST_MODE_RATE_QUANTUM_TPS;
			if (max_tps == 0) {
				max_tps = 1;
			}
		}
	}
#endif
	return CLAMP(tps, 1U, max_tps);
}

int test_mode_min_send_interval_ms(void)
{
	uint16_t tps = test_mode_effective_tps();
	if (tps == 0) {
		return 1000; /* normal mode heartbeat */
	}
	/* Sensor-side heartbeat view only. Connection uses exact rational
	 * scheduling, so this rounded value does not set the radio rate. */
	return (int)((1000U + tps / 2U) / tps);
}
