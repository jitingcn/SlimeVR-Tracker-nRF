/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <nrfx_timer.h>

#include "esb.h"
#include "timer.h"

LOG_MODULE_REGISTER(timer, LOG_LEVEL_INF);

// Timer instance index
#define TIMER_INST_IDX 1

// Hardware timer instance (TIMER1 @ 1MHz)
static nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(TIMER_INST_IDX);
static bool timer_initialized = false;

// TDMA state
static bool tdma_enabled = false;
static uint8_t tracker_id = 0;
static int32_t converge_count = 0;

// TX pending flag - set by esb_write, cleared after esb_start_tx
static volatile bool tx_pending_flag = false;

// Thresholds for TDMA enable/disable (asymmetric to favor keeping TDMA active)
#define SKEW_CONVERGE_THRESHOLD 30 // To enable: error must be <= 30 (~1ms)
#define SKEW_DIVERGE_THRESHOLD 150 // To disable: error must be > 150 (~5ms)
#define CONVERGE_COUNT_REQUIRED 3  // Consecutive stable updates to enable
#define DIVERGE_COUNT_REQUIRED 5   // Consecutive unstable updates to disable
#define UNSYNC_TX_INTERVAL_US 800  // Fallback TX interval when not synced

// Forward declaration
static void schedule_next_slot(void);

// Timer handler - called at TDMA slot time
static void timer_handler(nrf_timer_event_t event_type, void *p_context)
{
	ARG_UNUSED(p_context);

	if (event_type == NRF_TIMER_EVENT_COMPARE0) {
		if (tx_pending_flag) {
			// send all packets until esb_start_tx returns nodata
			while (esb_start_tx() == 0) {
				// do nothing
			}
			tx_pending_flag = false;
		}
		// Schedule next slot
		schedule_next_slot();
	}
}

static void schedule_next_slot(void)
{
	uint32_t wait_us;

	if (tdma_enabled) {
		uint64_t server_us = esb_get_server_time_us_64();
		if (server_us > 0) {
			// Calculate next slot time
			uint32_t frame_pos = server_us % TDMA_PACKET_INTERVAL_US;
			uint32_t slot_start = (tracker_id % TDMA_NUM_TRACKERS) * TDMA_SLOT_DURATION_US;
			uint32_t target = slot_start + TDMA_GUARD_TIME_US;

			if (frame_pos < target) {
				wait_us = target - frame_pos;
			} else {
				// Missed current slot, wait for next frame
				wait_us = (TDMA_PACKET_INTERVAL_US - frame_pos) + target;
			}

			// Clamp minimum wait to avoid too frequent interrupts
			if (wait_us < 100) {
				wait_us = TDMA_PACKET_INTERVAL_US + wait_us;
			}
		} else {
			// Server time not available, use fallback interval
			wait_us = UNSYNC_TX_INTERVAL_US;
		}
	} else {
		// Not in TDMA mode, use fallback interval
		wait_us = UNSYNC_TX_INTERVAL_US;
	}

	uint32_t ticks = nrfx_timer_us_to_ticks(&m_timer, wait_us);
	nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, true);
	nrfx_timer_clear(&m_timer);
}

void timer_init(void)
{
	if (timer_initialized) {
		return;
	}

	nrfx_err_t err;
	nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(1000000); // 1MHz
	timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
	// timer_cfg.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY;

	// Connect IRQ using nrfx macros (as per nrfx samples)
	IRQ_CONNECT(
		NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(TIMER_INST_IDX)),
		IRQ_PRIO_LOWEST,
		NRFX_TIMER_INST_HANDLER_GET(TIMER_INST_IDX),
		0,
		0
	);

	err = nrfx_timer_init(&m_timer, &timer_cfg, timer_handler);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("Timer init failed: %d", err);
		return;
	}

	irq_enable(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(TIMER_INST_IDX)));

	// Start with a default interval (will be adjusted when TDMA enables)
	uint32_t initial_ticks = nrfx_timer_us_to_ticks(&m_timer, UNSYNC_TX_INTERVAL_US);
	nrfx_timer_extended_compare(
		&m_timer,
		NRF_TIMER_CC_CHANNEL0,
		initial_ticks,
		NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
		true
	);
	nrfx_timer_enable(&m_timer);

	timer_initialized = true;
	LOG_INF("TDMA timer initialized (tracker_id=%u)", tracker_id);
}

void timer_set_tracker_id(uint8_t id)
{
	tracker_id = id;
	LOG_INF("Timer tracker_id set to %u, slot_start=%u us", id, (id % TDMA_NUM_TRACKERS) * TDMA_SLOT_DURATION_US);
}

static int diverge_count = 0;

void timer_on_skew_update(int32_t clock_diff)
{
	clock_diff = (clock_diff < 0) ? -clock_diff : clock_diff;

	if (clock_diff <= SKEW_CONVERGE_THRESHOLD) {
		// Good: error is small
		converge_count++;
		diverge_count = 0; // Reset diverge counter
		if (converge_count >= CONVERGE_COUNT_REQUIRED && !tdma_enabled) {
			tdma_enabled = true;
			LOG_INF("TDMA enabled (error=%d, count=%d)", clock_diff, converge_count);
		}
	} else if (clock_diff > SKEW_DIVERGE_THRESHOLD) {
		// Bad: error is very large
		diverge_count++;
		converge_count = 0; // Reset converge counter
		if (diverge_count >= DIVERGE_COUNT_REQUIRED && tdma_enabled) {
			// Don't disable TDMA on divergence!
			// It's better to keep running with the last known good offset (flywheel mode)
			// than to revert to random transmission which causes collisions.
			// tdma_enabled = false;
			// diverge_count = 0; // Keep counting or reset? Reset to avoid log spam
			if (diverge_count % 10 == 0) {
				LOG_WRN("TDMA divergence detected (error=%d, count=%d), maintaining sync", clock_diff, diverge_count);
			}
		}
	} else {
		// Middle zone: don't change state, just reset counters
		// This prevents oscillation
		converge_count = 0;
		diverge_count = 0;
	}
}

bool timer_is_tdma_active(void)
{
	return tdma_enabled && timer_initialized;
}

void timer_signal_tx_pending(void)
{
	tx_pending_flag = true;

	// In TDMA mode, timer interrupt will handle TX
	// In non-TDMA mode, trigger immediately
	if (!timer_is_tdma_active()) {
		while (esb_start_tx() == 0) {
			// do nothing
		}
		tx_pending_flag = false;
	}
}
