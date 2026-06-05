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
#include "globals.h"
#include "sensor/calibration.h"
#include "sensor/sensor.h"
#include "system/system.h"
#include "system/test_mode.h"
#include "system/watchdog.h"
#include "system/esb_ota.h"
#include "connection.h"
#include "zephyr/sys/byteorder.h"
#include "zephyr/sys/time_units.h"

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */
#include <hal/nrf_timer.h>
#include <nrfx_timer.h>
#include <zephyr/sys/crc.h>
#include <zephyr/kernel.h>

#include <stdlib.h>
#include "esb.h"
#include "tdma.h"
#include "console.h"
#include "system/clock_control.h"

uint8_t last_reset = 0;
// const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);
bool esb_state = false;
bool timer_state = false;
uint16_t led_clock = 0;
uint32_t led_clock_offset = 0;

int64_t connection_error_start_time = 0;
static bool shutdown_requested = false;
static bool pair_ack_pending = false; // True once step 1 is sent and we expect a receiver response

static struct esb_payload rx_payload;
// Normal data payload (16+1 bytes when used), length set per write
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0, 0, 0, 0, 0, 0, 0, 0, 0);

static uint8_t paired_addr[8] = {0};

static bool esb_initialized = false;
static bool esb_paired = false;

#define TX_ERROR_THRESHOLD 300
#define RADIO_RETRANSMIT_DELAY CONFIG_RADIO_RETRANSMIT_DELAY
#define RADIO_RF_CHANNEL CONFIG_RADIO_RF_CHANNEL

#if defined(CONFIG_CONNECTION_ENABLE_ACK)
#define CONNECTION_ENABLE_ACK true
#else
#define CONNECTION_ENABLE_ACK false
#endif

// Require N consecutive successful ACK probes before clearing connection error
#ifndef PING_RECOVERY_THRESHOLD
#define PING_RECOVERY_THRESHOLD 1
#endif

#define PING_BACKOFF_LVL1_THRESHOLD 2
#define PING_BACKOFF_LVL2_THRESHOLD 5
#define PING_BACKOFF_LVL3_THRESHOLD 10
#define PING_BACKOFF_LVL4_THRESHOLD 20
#define PING_BACKOFF_LVL1_MS        500
#define PING_BACKOFF_LVL2_MS        1500
#define PING_BACKOFF_LVL3_MS        4000
#define PING_BACKOFF_LVL4_MS        9000

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 1024, esb_thread, NULL, NULL, NULL, 6, 0, 0);
static int64_t last_tx_time = 0;

static uint32_t ping_success_streak = 0; // consecutive success counter
static bool ping_pending = false;
static bool ping_failed = false;

static uint32_t ping_failures = 0;
static uint32_t ping_ctr_sent = 0;
static uint8_t ping_counter = 0;
static int64_t ping_send_time = 0;


// Track send cycles for recent PINGs (circular buffer)
#define PING_HISTORY_SIZE 10
struct ping_history_entry {
	uint8_t counter;
	// ticks at ping send time
	uint32_t ping_ticks;
};
static struct ping_history_entry ping_history[PING_HISTORY_SIZE] = {0};
static uint8_t ping_history_idx = 0;

static uint8_t received_remote_command = ESB_PONG_FLAG_NORMAL;
static uint8_t acked_remote_command = ESB_PONG_FLAG_NORMAL;
static int64_t remote_command_receive_time = 0;
static uint32_t received_channel_value = 0; // Store channel value from PONG data[8-11]
static float received_sens_data[3] = {0};   // Store sensitivity data
static uint8_t received_sens_auto_axis = 0;
static uint16_t received_sens_auto_revolutions = 0;
#define REMOTE_COMMAND_DELAY_MS 1500

/* ── OTA packet queue (ISR → thread) ─────────────────────────────
 * OTA packets received in ESB ISR are queued here and processed
 * in the connection thread where flash/logging is safe. */
#define OTA_RX_QUEUE_SIZE 16
static struct {
	uint8_t data[CONFIG_ESB_MAX_PAYLOAD_LENGTH];
	uint8_t length;
} ota_rx_queue[OTA_RX_QUEUE_SIZE];
static volatile uint8_t ota_rx_head;
static volatile uint8_t ota_rx_tail;

// Server time synchronization for TDMA scheduling (using ticks)
static bool server_time_synced = false;

// Server time synchronization
static uint32_t g_server_ticks_offset = 0;
static uint32_t g_last_rx_raw_ticks = 0;
static uint32_t g_last_sync_local_ticks = 0;
static bool g_time_initialized = false;
static int64_t g_last_sync_timestamp = 0;
#define TIME_SYNC_TIMEOUT_MS 15000

// Clock skew compensation (tracker vs receiver crystal frequency difference)
static int32_t g_clock_skew_ppb = 0;         // Estimated clock skew in parts per billion
static int32_t g_skew_ref_offset = 0;        // Offset at skew reference point
static uint32_t g_skew_ref_local_ticks = 0;  // Local ticks at skew reference point (updated infrequently)
#define SKEW_REF_REFRESH_TICKS (60 * 32768)  // Refresh skew reference every ~60s

// Minimum RTT tracking for PONG acceptance threshold and diagnostics.
// Init to ~305µs (10 ticks): conservative estimate for clean 2Mbps ESB RTT.
static uint32_t g_min_rtt_ticks = 10;
static uint32_t g_min_rtt_age = 0; // PONGs since last min_rtt update
#define MIN_RTT_AGE_LIMIT 120      // Age out after ~120 PONGs (~2 min at 1/s)
#define MIN_RTT_CEILING   20       // Never age beyond this (conservative upper bound)

// Warm-up counter: first few PONGs use faster EMA for quick convergence
static uint32_t g_sync_update_count = 0;
#define SYNC_WARM_UP_COUNT 5

// Track last sent packet for TX_FAILED diagnostics
struct last_tx_info {
	uint8_t type;        // First byte of payload (packet type)
	bool noack;          // noack flag
	uint8_t length;      // Packet length
	int64_t timestamp;   // When it was sent
};
static struct last_tx_info last_tx = {0};

// Meow arrays for remote meow command
static const char *meows[] = {
	"Mew", "Meww", "Meow", "Meow meow", "Mrrrp", "Mrrf", "Mreow", "Mrrrow", "Mrrr", "Purr",
	"mew", "meww", "meow", "meow meow", "mrrrp", "mrrf", "mreow", "mrrrow", "mrrr", "purr",
};
static const char *meow_punctuations[] = {".", "?", "!", "-", "~", ""};
static const char *meow_suffixes[]
	= {" :3", " :3c", " ;3", " ;3c", " x3", " x3c", " X3", " X3c", " >:3", " >:3c", " >;3", " >;3c", ""};

static void remote_print_meow(void)
{
	int64_t ticks = k_uptime_ticks();
	ticks %= ARRAY_SIZE(meows) * ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes);
	uint8_t meow = ticks / (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	ticks %= (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	uint8_t punctuation = ticks / ARRAY_SIZE(meow_suffixes);
	uint8_t suffix = ticks % ARRAY_SIZE(meow_suffixes);
	LOG_INF("%s%s%s", meows[meow], meow_punctuations[punctuation], meow_suffixes[suffix]);
}

static uint8_t tracker_id = 0;
static void set_tracker_id(uint8_t id)
{
	tracker_id = id;
}

// --- esb_write() rate logging ---
static uint32_t esb_write_calls = 0;
static uint32_t esb_write_queued = 0;
static uint32_t esb_write_dup_queued = 0;
static int64_t esb_rate_last_ts = 0;

void esb_write_rate_tick(void)
{
	int64_t now = k_uptime_get();
	if (esb_rate_last_ts == 0) {
		esb_rate_last_ts = now;
	}
	esb_write_calls++;
	if (now - esb_rate_last_ts >= 5000) {
		LOG_INF("esb_write rate: calls=%u/s queued=%u/s dup=%u/s",
			esb_write_calls / 5, esb_write_queued / 5, esb_write_dup_queued / 5);
		esb_write_calls = 0;
		esb_write_queued = 0;
		esb_write_dup_queued = 0;
		esb_rate_last_ts = now;
	}
}

// ESB recovery mechanism for persistent ENOMEM errors
static uint32_t consecutive_enomem_errors = 0;
static int64_t last_enomem_time = 0;
#define ENOMEM_ERROR_THRESHOLD 3    // Force recovery after N consecutive errors
#define ENOMEM_ERROR_WINDOW_MS 1000 // Reset counter if no error for this duration

static void esb_clear_time_sync_state(void)
{
	server_time_synced = false;
	g_time_initialized = false;
	g_last_sync_timestamp = 0;
	g_last_rx_raw_ticks = 0;
	g_last_sync_local_ticks = 0;
	g_server_ticks_offset = 0;
	g_clock_skew_ppb = 0;
	g_skew_ref_offset = 0;
	g_skew_ref_local_ticks = 0;
	g_min_rtt_ticks = 10;
	g_sync_update_count = 0;
}



uint32_t esb_get_ping_backoff_ms(void)
{
	if (ping_failures >= PING_BACKOFF_LVL4_THRESHOLD) {
		return PING_BACKOFF_LVL4_MS;
	}
	if (ping_failures >= PING_BACKOFF_LVL3_THRESHOLD) {
		return PING_BACKOFF_LVL3_MS;
	}
	if (ping_failures >= PING_BACKOFF_LVL2_THRESHOLD) {
		return PING_BACKOFF_LVL2_MS;
	}
	if (ping_failures >= PING_BACKOFF_LVL1_THRESHOLD) {
		return PING_BACKOFF_LVL1_MS;
	}

	return 0;
}

bool clock_status = false;

#if defined(CONFIG_CLOCK_CONTROL_NRF)
static struct onoff_manager *clk_mgr;

static int clocks_init(void)
{
	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENOTSUP;
	}

	return 0;
}

SYS_INIT(clocks_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

int clocks_start(void)
{
	if (clock_status) {
		return 0;
	}
	int err;
	int res;
	struct onoff_client clk_cli;
	int fetch_attempts = 0;

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		k_usleep(100);
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
		if (err && ++fetch_attempts > 10) {
			LOG_WRN_ONCE("Unable to fetch Clock request result: %d", err);
			return err;
		}
	} while (err);

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	clock_status = true;
	return 0;
}

void clocks_stop(void)
{
	if (!clock_status) {
		return;
	}

	/* When using LF synthesizer, HFXO must remain active as it's the source
	 * for the LF clock. Don't stop HFXO in this case. */
	if (IS_ENABLED(CONFIG_CLOCK_USE_LF_SYNTH)) {
		LOG_DBG("HF clock kept running for LF_SYNTH");
		return;
	}

	clock_status = false;

	onoff_release(clk_mgr);

	LOG_DBG("HF clock stop request");
}

#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif

static struct k_thread clocks_thread_id;
static K_THREAD_STACK_DEFINE(clocks_thread_id_stack, 512);

// Wrapper function with correct signature for k_thread_entry_t
static void clocks_start_wrapper(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	(void)clocks_start();
}

void clocks_request_start(uint32_t delay_us)
{
	k_thread_create(
		&clocks_thread_id,
		clocks_thread_id_stack,
		K_THREAD_STACK_SIZEOF(clocks_thread_id_stack),
		clocks_start_wrapper,
		NULL,
		NULL,
		NULL,
		5,
		0,
		K_USEC(delay_us)
	);
}

static struct k_thread clocks_stop_thread_id;
static K_THREAD_STACK_DEFINE(clocks_stop_thread_id_stack, 512);

// Wrapper function with correct signature for k_thread_entry_t
static void clocks_stop_wrapper(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	clocks_stop();
}

void clocks_request_stop(uint32_t delay_us)
{
	k_thread_create(
		&clocks_stop_thread_id,
		clocks_stop_thread_id_stack,
		K_THREAD_STACK_SIZEOF(clocks_stop_thread_id_stack),
		clocks_stop_wrapper,
		NULL,
		NULL,
		NULL,
		5,
		0,
		K_USEC(delay_us)
	);
}

void event_handler(struct esb_evt const *event)
{
	static uint32_t tx_success_count = 0;
	static uint32_t tx_failed_count = 0;
	static uint32_t last_log_time = 0;

	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		tx_success_count++;
		// Reset ENOMEM error counter on successful transmission
		consecutive_enomem_errors = 0;
		if (esb_paired && !connection_get_data_collection() && esb_is_idle()) {
			clocks_stop();
		}
		break;
	case ESB_EVENT_TX_FAILED:
		tx_failed_count++;

		// Detailed packet type diagnostics for TX_FAILED
		const char *pkt_desc = "UNKNOWN";
		if (last_tx.type == 0x00) {
			pkt_desc = "device info";
		} else if (last_tx.type == 0x01) {
			pkt_desc = "packet 1";
		} else if (last_tx.type == 0x02) {
			pkt_desc = "packet 2";
		} else if (last_tx.type == 0x03) {
			pkt_desc = "status";
		} else if (last_tx.type == 0x04) {
			pkt_desc = "packet 4";
		} else if (last_tx.type == ESB_PING_TYPE) {
			pkt_desc = "PING";
		} else {
			pkt_desc = "OTHER";
		}

		LOG_DBG(
			"TX FAILED: type=%s(0x%02X) len=%u noack=%d age=%lldms attempts=%u",
			pkt_desc,
			last_tx.type,
			last_tx.length,
			last_tx.noack,
			k_uptime_get() - last_tx.timestamp,
			event->tx_attempts
		);

		// Log TX statistics every 100 failures for debugging
		uint32_t now = k_uptime_get_32();
		if (tx_failed_count % 100 == 0 || (now - last_log_time > 5000)) {
			last_log_time = now;
			uint32_t total = tx_success_count + tx_failed_count;
			uint32_t fail_rate = total > 0 ? (tx_failed_count * 100 / total) : 0;
			LOG_INF("TX Stats: success=%u failed=%u rate=%u%%", tx_success_count, tx_failed_count, fail_rate);
		}

		// Only count ping failures for connection timeout
		if (ping_pending && k_uptime_get() - ping_send_time > (get_ping_interval_ms() - 100)) {
			ping_failed = true;
			ping_pending = false;    // Clear the pending flag
			ping_success_streak = 0; // Reset recovery streak on any failure
			ping_failures++;
		}

		if (ping_failures > 0 && ping_failures % 10 == 0 && last_tx.type == ESB_PING_TYPE) // Log every 10 failures
		{
			LOG_WRN("Ping failed, total failures: %d", ping_failures);
		}
		if (ping_failures == TX_ERROR_THRESHOLD) // consecutive ping failures
		{
			connection_error_start_time = k_uptime_get(); // Mark when connection errors started
			LOG_WRN(
				"Ping failure threshold reached (%d failures), starting "
				"timeout timer",
				TX_ERROR_THRESHOLD
			);
		}

		if (esb_paired && !connection_get_data_collection()) {
			clocks_stop();
		}
		break;
	case ESB_EVENT_RX_RECEIVED: {
		uint32_t current_rx_ticks = sys_clock_tick_get_32();
		int err = 0;
		err = esb_read_rx_payload(&rx_payload);
		if (err == -ENODATA) {
			return;
		} else if (err) {
			LOG_ERR("Error while reading rx packet: %d", err);
			return;
		}
		if (!paired_addr[0]) // zero, not paired
		{
			LOG_DBG("tx: %16llX rx: %16llX", *(uint64_t *)tx_payload_pair.data, *(uint64_t *)rx_payload.data);
			if (rx_payload.length == 8) {
				if (!pair_ack_pending) {
					LOG_DBG("Ignoring unsolicited pairing response");
					break;
				}
				if (rx_payload.data[0] != tx_payload_pair.data[0]) {
					LOG_DBG(
						"Ignoring pairing response with mismatched checksum "
						"%02X",
						rx_payload.data[0]
					);
					pair_ack_pending = false;
					break;
				}
				uint64_t responder_addr = 0;
				memcpy(&responder_addr, &rx_payload.data[2], 6);
				responder_addr &= 0xFFFFFFFFFFFFULL;
				uint64_t local_addr = (*(uint64_t *)NRF_FICR->DEVICEADDR) & 0xFFFFFFFFFFFFULL;
				if (responder_addr == local_addr) {
					LOG_WRN(
						"Ignoring pairing response sourced from local device "
						"address"
					);
					pair_ack_pending = false;
					break;
				}
				memcpy(paired_addr, rx_payload.data, sizeof(paired_addr));
				pair_ack_pending = false;
			}
		} else {
			switch (rx_payload.length) {
			case ESB_PONG_LEN: {
				if (rx_payload.data[0] == ESB_PONG_TYPE) {
					// check CRC first
					uint8_t crc_calc = crc8_ccitt(0x07, rx_payload.data, ESB_PONG_LEN - 1);
					if (rx_payload.data[ESB_PONG_LEN - 1] != crc_calc) {
						LOG_WRN("PONG CRC mismatch");
						break;
					}
					uint8_t rx_id = rx_payload.data[1];
					if (rx_id != tracker_id) {
						// When using >7 trackers, multiple trackers share the same pipe
						// This causes PONG responses to have mismatched IDs until TDMA is implemented
						// For now, accept these responses as valid to maintain connectivity
						LOG_WRN("Received PONG for tracker ID %u (local ID %u)", rx_id, tracker_id);
						// set ping valid
						ping_pending = false;
						ping_failed = false;
						ping_failures = 0;
						if (get_status(SYS_STATUS_CONNECTION_ERROR) == true) {
							set_status(SYS_STATUS_CONNECTION_ERROR, false);
							connection_error_start_time = 0;
							shutdown_requested = false;
							ping_success_streak = 0;
						}
						break;
					}
					uint8_t rx_ctr = rx_payload.data[2];
					int counter_diff = (int)ping_counter - (int)rx_ctr;
					if (counter_diff < 0) {
						counter_diff += 256; // Handle wrap-around
					}

					// set ping valid first
					ping_pending = false;
					ping_failed = false;
					ping_failures = 0;
					if (get_status(SYS_STATUS_CONNECTION_ERROR) == true) {
						ping_success_streak++;
						if (ping_success_streak >= PING_RECOVERY_THRESHOLD) {
							set_status(SYS_STATUS_CONNECTION_ERROR, false);
							connection_error_start_time = 0;
							shutdown_requested = false;
							ping_success_streak = 0;
						}
					} else {
						ping_success_streak = 0;
					}

					bool match_ctr = (counter_diff >= 0 && counter_diff <= 5);
					if (!match_ctr) {
						LOG_WRN("unsynced counter %u (expected ~%u, diff=%d)", rx_ctr, ping_counter, counter_diff);
						// Don't break - still process the PONG to maintain connection
						// Just log the warning for debugging
					}

					uint32_t ping_rx_ticks = ((uint32_t)rx_payload.data[3] << 24) | ((uint32_t)rx_payload.data[4] << 16)
										   | ((uint32_t)rx_payload.data[5] << 8) | ((uint32_t)rx_payload.data[6]);

					// Find send ticks for this PONG's counter in history
					uint32_t ping_ticks_for_this_ctr = 0;
					for (int i = 0; i < PING_HISTORY_SIZE; i++) {
						if (ping_history[i].counter == rx_ctr && ping_history[i].ping_ticks != 0) {
							ping_ticks_for_this_ctr = ping_history[i].ping_ticks;
							break;
						}
					}

					// Check flags field (byte 7)
					uint8_t pong_flags = rx_payload.data[7];
					uint32_t rtt_us = 0;
					float pong_sens_data[3] = {0.0f, 0.0f, 0.0f};
					uint8_t pong_sens_auto_axis = 0;
					uint16_t pong_sens_auto_revolutions = 0;

					if (pong_flags == ESB_PONG_FLAG_SENS_SET) {
						// Special case: SENS_SET command repurposes time sync bytes for data
						// Skip time sync update
						int16_t x_int = (int16_t)((rx_payload.data[3] << 8) | rx_payload.data[4]);
						int16_t y_int = (int16_t)((rx_payload.data[5] << 8) | rx_payload.data[6]);
						int16_t z_int = (int16_t)((rx_payload.data[8] << 8) | rx_payload.data[9]);

						pong_sens_data[0] = (float)x_int / 100.0f;
						pong_sens_data[1] = (float)y_int / 100.0f;
						pong_sens_data[2] = (float)z_int / 100.0f;

						LOG_INF(
							"Received SENS_SET data: %.2f, %.2f, %.2f",
							(double)pong_sens_data[0],
							(double)pong_sens_data[1],
							(double)pong_sens_data[2]
						);
					} else if (pong_flags == ESB_PONG_FLAG_SENS_AUTO) {
						pong_sens_auto_axis = rx_payload.data[3];
						pong_sens_auto_revolutions
							= ((uint16_t)rx_payload.data[4] << 8) | (uint16_t)rx_payload.data[5];
						if (pong_sens_auto_revolutions == 0) {
							LOG_INF("Received SENS_AUTO data: axis=%u, revolutions=default", pong_sens_auto_axis);
						} else {
							LOG_INF(
								"Received SENS_AUTO data: axis=%u, revolutions=%u",
								pong_sens_auto_axis,
								pong_sens_auto_revolutions
							);
						}
					} else if (ping_ticks_for_this_ctr != 0) {
						// ====================================================================
						// RTT and Server Time Offset Calculation (Reference-Point Model)
						// ====================================================================
						// In ESB, the return path (ACK) has FIXED delay regardless of
						// retransmissions. All retransmission time is on the forward path.
						//
						// No retransmit:    T1 --[air]--> T2,  T4 <--[ACK]-- T3≈T2
						// With retransmit:  T1 --[fail]--[fail]--[air]--> T2
						//                   T4 <--[ACK]-- T3≈T2
						//
						// offset = T2 - T4 (constant one-way bias cancels for TDMA)
						// ====================================================================

						/* Use ISR-accurate T4 timestamp captured in the RADIO
						 * ISR (esb_last_ack_rx_ticks) instead of EVENT_IRQ
						 * current_rx_ticks.  This eliminates 10-25 ticks of
						 * kernel scheduling jitter from the offset estimate,
						 * reducing server_time noise from ±15 to ±2 ticks. */
						uint32_t t4_ticks = esb_last_ack_rx_ticks;

						// Calculate full RTT: from PING send (T1) to PONG receive (T4)
						uint32_t rtt_ticks = t4_ticks - ping_ticks_for_this_ctr;
						rtt_us = k_ticks_to_us_floor32(rtt_ticks);

						// Track minimum RTT (no-retransmission baseline)
						// with aging: if min hasn't been refreshed in
						// MIN_RTT_AGE_LIMIT PONGs, nudge it upward by 1 tick
						// to recover from anomalously low measurements.
						if (rtt_ticks > 0 && rtt_ticks <= g_min_rtt_ticks) {
							g_min_rtt_ticks = rtt_ticks;
							g_min_rtt_age = 0;
						} else {
							g_min_rtt_age++;
							if (g_min_rtt_age >= MIN_RTT_AGE_LIMIT &&
							    g_min_rtt_ticks < MIN_RTT_CEILING) {
								g_min_rtt_ticks++;
								g_min_rtt_age = 0;
								LOG_DBG("min_rtt aged up to %u ticks", g_min_rtt_ticks);
							}
						}

						// log ping and rtt
						if (rtt_us > 1000) {
							LOG_DBG(
								"PONG ok, ack rtt=%u.%03u ms (ctr=%u)",
								(unsigned)(rtt_us / 1000),
								(unsigned)(rtt_us % 1000),
								rx_ctr
							);
						} else if (rtt_us < 1000) {
							LOG_DBG("PONG ok, ack rtt=%u us (ctr=%u)", (unsigned)rtt_us, rx_ctr);
						}

						/*
						 * Adaptive RTT acceptance threshold.
						 * Accept PONGs with RTT up to 4× min_rtt (handles minor
						 * retransmissions) or 1000µs absolute floor (during min_rtt
						 * warm-up when min is unreliable).
						 */
						uint32_t rtt_threshold_us = k_ticks_to_us_floor32(g_min_rtt_ticks * 4);
						if (rtt_threshold_us < 1000) {
							rtt_threshold_us = 1000;
						}
						if (rtt_us < rtt_threshold_us) {
							// Reference-point offset: T2 - T4
							// The constant one-way delay bias is the same for
							// all trackers and cancels out in TDMA slot alignment.
							// Decoupling from min_rtt avoids noise injection when
							// min_rtt ages/updates.
							int32_t server_offset_ticks
								= (int32_t)(ping_rx_ticks - t4_ticks);

							g_last_rx_raw_ticks = ping_rx_ticks;
							g_last_sync_local_ticks = ping_ticks_for_this_ctr;
							g_last_sync_timestamp = k_uptime_get();

							if (!g_time_initialized) {
								g_server_ticks_offset = server_offset_ticks;
								g_skew_ref_offset = server_offset_ticks;
								g_skew_ref_local_ticks = ping_ticks_for_this_ctr;
								g_sync_update_count = 0;
								g_time_initialized = true;
								server_time_synced = true;
								LOG_DBG("Server offset initialized: %d ticks", server_offset_ticks);
							} else {
								// Long-baseline skew estimation:
								// Keep skew reference point fixed, compute total drift
								// over growing baseline to average out RTT noise
								uint32_t delta_from_ref = ping_ticks_for_this_ctr - g_skew_ref_local_ticks;

								// Compute innovation (prediction residual) for diagnostics
								int32_t predicted_drift = (int32_t)((int64_t)g_clock_skew_ppb * (int64_t)delta_from_ref / 1000000000LL);
								int32_t predicted_offset = g_skew_ref_offset + predicted_drift;
								int32_t innovation = server_offset_ticks - predicted_offset;

								// Large jump detection (> 1 second ≈ 32000 ticks)
								if (abs(innovation) > 32000) {
									LOG_WRN(
										"Large offset jump detected (%d ticks), resetting "
										"(old=%d new=%d)",
										innovation,
										g_server_ticks_offset,
										server_offset_ticks
									);
									g_server_ticks_offset = server_offset_ticks;
									g_skew_ref_offset = server_offset_ticks;
									g_skew_ref_local_ticks = ping_ticks_for_this_ctr;
									g_clock_skew_ppb = 0;
								} else {
									// Update skew from long-baseline total drift
									// As baseline grows, RTT noise effect shrinks
									if (delta_from_ref >= 32768) {
										int64_t total_drift = (int64_t)(server_offset_ticks - g_skew_ref_offset);
										int32_t raw_skew_ppb = (int32_t)(total_drift * 1000000000LL / (int64_t)delta_from_ref);
										// Gentle EMA: long baseline already provides stability
										g_clock_skew_ppb = g_clock_skew_ppb + (raw_skew_ppb - g_clock_skew_ppb) / 4;
									}

									/*
								 * Predict-Update EMA offset filter.
								 *
								 * Use skew prediction as the expected offset,
								 * so EMA innovation contains only measurement
								 * noise (not deterministic drift).  This allows
								 * a smaller alpha for better noise rejection
								 * while skew handles drift tracking.
								 *
								 * Warm-up (first 5 PONGs): alpha=3/4 for fast
								 * initial convergence before skew is estimated.
								 */
								g_sync_update_count++;
								/* Predict: where we expect the offset to be based on skew */
								uint32_t delta_since_sync = ping_ticks_for_this_ctr - g_last_sync_local_ticks;
								int32_t predicted_current = (int32_t)g_server_ticks_offset
									+ (int32_t)((int64_t)g_clock_skew_ppb * delta_since_sync / 1000000000LL);
								int32_t offset_innovation = server_offset_ticks - predicted_current;
								if (g_sync_update_count <= SYNC_WARM_UP_COUNT) {
									/* Warm-up: alpha=3/4 for fast convergence */
									g_server_ticks_offset = predicted_current + (offset_innovation * 3 + 2) / 4;
								} else {
									/* Steady state: alpha=1/4 (skew handles drift) */
									g_server_ticks_offset = predicted_current + (offset_innovation + 2) / 4;
								}
								// Refresh skew reference periodically to avoid uint32 wrap
								if (delta_from_ref > SKEW_REF_REFRESH_TICKS) {
										g_skew_ref_offset = server_offset_ticks;
										g_skew_ref_local_ticks = ping_ticks_for_this_ctr;
									}

									LOG_DBG("Offset update: innovation=%d offset=%d skew=%d ppb (rtt=%u min=%u)",
										innovation, g_server_ticks_offset, g_clock_skew_ppb,
										rtt_ticks, g_min_rtt_ticks);
								}
							}

							server_time_synced = true;

							// Display skew-compensated estimated server time
							uint32_t local_now = sys_clock_tick_get_32();
							uint32_t elapsed_since_meas = local_now - ping_ticks_for_this_ctr;
							int32_t skew_corr = (int32_t)((int64_t)g_clock_skew_ppb * elapsed_since_meas / 1000000000LL);
							uint32_t est_ticks = (uint32_t)((int32_t)g_server_ticks_offset + (int32_t)local_now + skew_corr);
							uint32_t server_time_ms = k_ticks_to_ms_near32(est_ticks);
							uint32_t server_ms = server_time_ms % 1000;
							uint32_t server_s = (server_time_ms / 1000) % 60;
							uint32_t server_m = (server_time_ms / 60000) % 60;
							uint32_t server_h = (server_time_ms / 3600000) % 24;
							LOG_DBG(
								"estimated server time: %02u:%02u:%02u.%03u (ticks=%u)",
								server_h,
								server_m,
								server_s,
								server_ms,
								est_ticks
							);
						}
					} else {
						// No history found - likely too old or buffer wrapped
					}

					/* Parse dynamic TDMA config from NORMAL PONG bytes 8-11.
					 * Only valid when pong_flags == NORMAL (other commands
					 * use bytes 8-11 for command-specific data). */
					if (pong_flags == ESB_PONG_FLAG_NORMAL) {
						uint8_t tdma_slot   = rx_payload.data[8];
						uint8_t tdma_total  = rx_payload.data[9];
						uint8_t tdma_sticks = rx_payload.data[10];
						uint8_t tdma_epoch  = rx_payload.data[11];

						if (tdma_slot != 0xFF && tdma_total > 0 && tdma_sticks > 0 &&
						    tdma_epoch != tdma_get_config_epoch()) {
							tdma_update_config(tdma_slot, tdma_total, tdma_sticks, tdma_epoch);
						}
					}

					// handle remote commands and delayed execution
					if (pong_flags != ESB_PONG_FLAG_NORMAL) {
						if (received_remote_command == ESB_PONG_FLAG_NORMAL ||
						    (received_remote_command == acked_remote_command &&
						     pong_flags != received_remote_command)) {
							// new command received, or override already-executed command
							// whose confirmation was superseded by the receiver
							// (but skip re-accepting the same command repeatedly)
							received_remote_command = pong_flags;
							remote_command_receive_time = k_uptime_get();

							// For SET_CHANNEL command, extract channel value from data[8-11]
							if (pong_flags == ESB_PONG_FLAG_SET_CHANNEL) {
								received_channel_value
									= ((uint32_t)rx_payload.data[8] << 24) | ((uint32_t)rx_payload.data[9] << 16)
									| ((uint32_t)rx_payload.data[10] << 8) | ((uint32_t)rx_payload.data[11]);
							} else if (pong_flags == ESB_PONG_FLAG_SENS_SET) {
								memcpy(received_sens_data, pong_sens_data, sizeof(received_sens_data));
							} else if (pong_flags == ESB_PONG_FLAG_SENS_AUTO) {
								received_sens_auto_axis = pong_sens_auto_axis;
								received_sens_auto_revolutions = pong_sens_auto_revolutions;
							}

							const char *cmd_name = "UNKNOWN";
							switch (pong_flags) {
							case ESB_PONG_FLAG_SHUTDOWN:
								cmd_name = "SHUTDOWN";
								break;
							case ESB_PONG_FLAG_CALIBRATE:
								cmd_name = "CALIBRATE";
								break;
							case ESB_PONG_FLAG_SIX_SIDE_CAL:
								cmd_name = "SIX_SIDE_CAL";
								break;
							case ESB_PONG_FLAG_MEOW:
								cmd_name = "MEOW";
								break;
							case ESB_PONG_FLAG_SCAN:
								cmd_name = "SCAN";
								break;
							case ESB_PONG_FLAG_MAG_CLEAR:
								cmd_name = "MAG_CLEAR";
								break;
							case ESB_PONG_FLAG_MAG_CAL:
								cmd_name = "MAG_CAL";
								break;
							case ESB_PONG_FLAG_MAG_ON:
								cmd_name = "MAG_ON";
								break;
							case ESB_PONG_FLAG_MAG_OFF:
								cmd_name = "MAG_OFF";
								break;
							case ESB_PONG_FLAG_REBOOT:
								cmd_name = "REBOOT";
								break;
							case ESB_PONG_FLAG_CLEAR:
								cmd_name = "CLEAR";
								break;
							case ESB_PONG_FLAG_DFU:
								cmd_name = "DFU";
								break;
							case ESB_PONG_FLAG_DFU_OTA:
								cmd_name = "DFU_OTA";
								break;
							case ESB_PONG_FLAG_SET_CHANNEL:
								cmd_name = "SET_CHANNEL";
								break;
							case ESB_PONG_FLAG_SENS_SET:
								cmd_name = "SENS_SET";
								break;
							case ESB_PONG_FLAG_SENS_RESET:
								cmd_name = "SENS_RESET";
								break;
							case ESB_PONG_FLAG_SENS_AUTO:
								cmd_name = "SENS_AUTO";
								break;
							case ESB_PONG_FLAG_RESET_ZRO:
								cmd_name = "RESET_ZRO";
								break;
							case ESB_PONG_FLAG_RESET_ACC:
								cmd_name = "RESET_ACC";
								break;
							case ESB_PONG_FLAG_RESET_BAT:
								cmd_name = "RESET_BAT";
								break;
							case ESB_PONG_FLAG_RESET_TCAL:
								cmd_name = "RESET_TCAL";
								break;
							case ESB_PONG_FLAG_TCAL_AUTO_ON:
								cmd_name = "TCAL_AUTO_ON";
								break;
							case ESB_PONG_FLAG_TCAL_AUTO_OFF:
								cmd_name = "TCAL_AUTO_OFF";
								break;
							case ESB_PONG_FLAG_PING:
								cmd_name = "PING";
								break;
							case ESB_PONG_FLAG_FUSION_RESET:
								cmd_name = "FUSION_RESET";
								break;
							case ESB_PONG_FLAG_TCAL_BOOT_ON:
								cmd_name = "TCAL_BOOT_ON";
								break;
							case ESB_PONG_FLAG_TCAL_BOOT_OFF:
								cmd_name = "TCAL_BOOT_OFF";
								break;
							case ESB_PONG_FLAG_TCAL_ON:
								cmd_name = "TCAL_ON";
								break;
							case ESB_PONG_FLAG_TCAL_OFF:
								cmd_name = "TCAL_OFF";
								break;
							case ESB_PONG_FLAG_TDMA_ON:
								cmd_name = "TDMA_ON";
								break;
							case ESB_PONG_FLAG_TDMA_OFF:
								cmd_name = "TDMA_OFF";
								break;
							case ESB_PONG_FLAG_OTA_QUERY_INFO:
								cmd_name = "OTA_QUERY_INFO";
								break;
							case ESB_PONG_FLAG_OTA_ABORT:
								cmd_name = "OTA_ABORT";
								break;
							case ESB_PONG_FLAG_OTA_SUPPRESS:
								cmd_name = "OTA_SUPPRESS";
								break;
							case ESB_PONG_FLAG_OTA_UNSUPPRESS:
								cmd_name = "OTA_UNSUPPRESS";
								break;
							}
							if (pong_flags == ESB_PONG_FLAG_SET_CHANNEL) {
								LOG_INF(
									"Remote command %s (0x%02X) received, channel=%u, will execute in %dms",
									cmd_name,
									pong_flags,
									received_channel_value,
									REMOTE_COMMAND_DELAY_MS
								);
							} else {
								bool is_ota = (pong_flags >= ESB_PONG_FLAG_OTA_QUERY_INFO &&
									       pong_flags <= ESB_PONG_FLAG_OTA_UNSUPPRESS);
								LOG_INF(
									"Remote command %s (0x%02X) received, %s",
									cmd_name,
									pong_flags,
									is_ota ? "executing immediately" :
									"will execute in 1500ms"
								);
							}
						}
					} else {
						// received NORMAL flag, indicates the receiver has confirmed our echo
						if (acked_remote_command != ESB_PONG_FLAG_NORMAL) {
							LOG_DBG("Receiver confirmed command 0x%02X, resetting state", acked_remote_command);
							received_remote_command = ESB_PONG_FLAG_NORMAL;
							acked_remote_command = ESB_PONG_FLAG_NORMAL;
							remote_command_receive_time = 0;
						}
					}
				}
			} break;
			case ESB_SENSOR_DATA_LEN: {
				// received other tracker's sensor data, likely due to shared pipe, just ignore
			} break;
			default:
				/* ACK payload from receiver carrying ARQ retransmit requests */
				if (rx_payload.length >= 4 &&
				    rx_payload.data[0] == 0xAA &&
				    connection_get_data_collection()) {
					uint8_t retx_n = rx_payload.data[1];
					uint8_t max_entries = (rx_payload.length - 2) / 2;
					if (retx_n > max_entries) {
						retx_n = max_entries;
					}
					extern volatile uint16_t raw_retx_queue[];
					extern volatile uint8_t  raw_retx_count;
					for (uint8_t i = 0; i < retx_n; i++) {
						uint16_t seq = sys_get_be16(&rx_payload.data[2 + i * 2]);
						/* Deduplicate */
						bool found = false;
						for (uint8_t j = 0; j < raw_retx_count; j++) {
							if (raw_retx_queue[j] == seq) {
								found = true;
								break;
							}
						}
						if (!found && raw_retx_count < 16) {
							raw_retx_queue[raw_retx_count++] = seq;
						}
					}
				}
				/* OTA packets from receiver (in ACK payload) —
				 * queue for deferred processing in thread context
				 * (flash ops and logging not safe in ISR) */
				else if (rx_payload.length >= 2 &&
					 rx_payload.data[0] >= ESB_OTA_DATA_TYPE &&
					 rx_payload.data[0] <= ESB_OTA_ACTIVATE_TYPE) {
					uint8_t next = (ota_rx_head + 1) % OTA_RX_QUEUE_SIZE;
					if (next != ota_rx_tail) {
						memcpy(ota_rx_queue[ota_rx_head].data,
						       rx_payload.data, rx_payload.length);
						ota_rx_queue[ota_rx_head].length = rx_payload.length;
						__DMB();
						ota_rx_head = next;
					}
				}
				else {
					LOG_WRN("Ignoring invalid payload length %u", rx_payload.length);
				}
			} // end of rx_payload length switch
		}
		break;
	} // end of ESB_EVENT_RX_RECEIVED
	} // end of event switch
}

// this was randomly generated
// TODO: I have no idea?
// TODO: see esb information, check CONFIG_ESB_PIPE_COUNT
/*
base_addr_p0: Base address for pipe 0, in big endian.
base_addr_p1: Base address for pipe 1-7, in big endian.
pipe_prefixes: Address prefix for pipe 0 to 7.
*/
static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

int esb_initialize(bool tx)
{
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	if (tx) {
		config.protocol = ESB_PROTOCOL_ESB_DPL;
		// config.mode = ESB_MODE_PTX;
		config.event_handler = event_handler;
		config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = RADIO_RETRANSMIT_DELAY;
		config.retransmit_count = 2;
		config.tx_mode = ESB_TXMODE_MANUAL_START;
		// config.payload_length = 252; // config by CONFIG_ESB_MAX_PAYLOAD_LENGTH
		config.selective_auto_ack = true;
		config.use_fast_ramp_up = true;
	} else {
		config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = RADIO_RETRANSMIT_DELAY;
		// config.retransmit_count = 3;
		// config.tx_mode = ESB_TXMODE_AUTO;
		// config.payload_length = 252; // config by CONFIG_ESB_MAX_PAYLOAD_LENGTH
		config.selective_auto_ack = true;
		config.use_fast_ramp_up = true;
	}

	err = esb_init(&config);

	if (!err) {
		// Read and apply RF channel from retained/NVS
		// 0xFF and 0 both indicate "use default"
		if (retained->rf_channel != 0xFF && retained->rf_channel != 0 && retained->rf_channel <= 100) {
			LOG_INF("Restoring RF channel from NVS: %u", retained->rf_channel);
			esb_set_rf_channel(retained->rf_channel);
		} else {
			LOG_INF("Using default RF channel: %u", RADIO_RF_CHANNEL);
			esb_set_rf_channel(RADIO_RF_CHANNEL);
			// Initialize with 0xFF to indicate default is being used
			if (retained->rf_channel != 0xFF) {
				retained->rf_channel = 0xFF;
				retained_update();
			}
		}
	}

	if (!err) {
		esb_set_base_address_0(base_addr_0);
	}

	if (!err) {
		esb_set_base_address_1(base_addr_1);
	}

	if (!err) {
		esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	}

	if (err) {
		LOG_ERR("ESB initialization failed: %d", err);
		set_status(SYS_STATUS_CONNECTION_ERROR, true);
		return err;
	}
	LOG_INF("ESB initialized, %sX mode", tx ? "T" : "R");
	esb_initialized = true;
	return 0;
}

void esb_deinitialize(void)
{
	if (esb_initialized) {
		esb_initialized = false;
		k_msleep(3); // wait for pending transmissions
		esb_disable();
	}
	esb_initialized = false;
}

inline void esb_set_addr_discovery(void)
{
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

inline void esb_set_addr_paired(void)
{
	// Recreate receiver address
	uint8_t addr_buffer[16] = {0};
	for (int i = 0; i < 4; i++) {
		addr_buffer[i] = paired_addr[i + 2];
		addr_buffer[i + 4] = paired_addr[i + 2] + paired_addr[6];
	}
	for (int i = 0; i < 8; i++) {
		addr_buffer[i + 8] = paired_addr[7] + i;
	}
	for (int i = 0; i < 16; i++) {
		if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55
			|| addr_buffer[i] == 0xAA) { // Avoid invalid addresses (see nrf datasheet)
			addr_buffer[i] += 8;
		}
	}
	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
}

static int esb_send_pair_step(uint8_t step)
{
	tx_payload_pair.data[1] = step;
	int err = esb_write_payload(&tx_payload_pair);
	if (err == -ENOSPC) {
		esb_flush_tx();
		err = esb_write_payload(&tx_payload_pair);
	}
	if (err) {
		LOG_ERR("Failed to queue pairing burst step %u: %d", step, err);
		return err;
	}
	err = esb_start_tx();
	if (err == -EBUSY) {
		LOG_DBG("Pairing burst step %u already pending", step);
		err = 0;
	} else if (err) {
		LOG_ERR("Failed to start pairing burst step %u: %d", step, err);
	}
	return err;
}

void esb_set_pair(uint64_t addr)
{
	// Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	uint64_t *device_addr = (uint64_t *)NRF_FICR->DEVICEADDR;
	uint8_t buf[6] = {0};
	memcpy(buf, device_addr, 6);
	uint8_t checksum = crc8_ccitt(0x07, buf, 6);
	if (checksum == 0) {
		checksum = 8;
	}
	if ((addr & 0xFF) != checksum) {
		LOG_INF("Incorrect checksum");
		return;
	}
	esb_reset_pair();
	memcpy(paired_addr, &addr, sizeof(paired_addr));
	LOG_INF("Paired");
	sys_write(PAIRED_ID, retained->paired_addr, paired_addr,
			  sizeof(paired_addr)); // Write new address and tracker id
}

void esb_pair(void)
{
	// Reset ping state when starting pairing
	ping_failures = 0;
	set_status(SYS_STATUS_CONNECTION_ERROR, false);
	connection_error_start_time = 0;
	shutdown_requested = false;
	ping_failed = false;
	ping_pending = false;
	// Reset time sync state
	esb_clear_time_sync_state();
	if (!paired_addr[0]) // zero, no receiver paired
	{
		LOG_INF("Pairing");
		esb_set_addr_discovery();
		esb_initialize(true);
		//		timer_init(); // TODO: shouldn't be here!!!
		tx_payload_pair.noack = false;
		// Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
		uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR;
		memcpy(&tx_payload_pair.data[2], addr, 6);
		LOG_INF("Device address: %012llX", *addr & 0xFFFFFFFFFFFF);
		uint8_t checksum = crc8_ccitt(0x07, &tx_payload_pair.data[2], 6);
		if (checksum == 0) {
			checksum = 8;
		}
		LOG_INF("Checksum: %02X", checksum);
		tx_payload_pair.data[0] = checksum; // Use checksum to make sure packet is for this device
		set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
		int64_t pair_start_time = k_uptime_get();
		while (paired_addr[0] != checksum && ((*(uint64_t *)&paired_addr[0] >> 16) & 0xFFFFFFFFFFFF) != *addr) {
			if (!esb_initialized) {
				esb_set_addr_discovery();
				esb_initialize(true);
			}
			if (!clock_status) {
				clocks_start();
			}

#if USER_SHUTDOWN_ENABLED
			// During pairing, only use connection timeout to decide shutdown
			if (!shutdown_requested && (k_uptime_get() - pair_start_time) > CONFIG_CONNECTION_TIMEOUT_DELAY) {
				LOG_WRN("Pairing timeout after %dm", CONFIG_CONNECTION_TIMEOUT_DELAY / 60000);
				shutdown_requested = true;
				sys_request_system_off(false);
			}
#endif
			if (paired_addr[0]) {
				LOG_INF("Incorrect checksum: %02X", paired_addr[0]);
				paired_addr[0] = 0; // Packet not for this device
			}
			esb_flush_rx();
			esb_flush_tx();
			pair_ack_pending = false; // Reset before sending

			/* Feed ESB watchdog during pairing to prevent timeout */
			watchdog_feed(WDT_CHANNEL_ESB);

			if (esb_send_pair_step(0)) {
				k_msleep(100);
				continue;
			}
			k_msleep(2);
			pair_ack_pending = true; // Set before step 1 which expects receiver response
			if (esb_send_pair_step(1)) {
				pair_ack_pending = false;
				k_msleep(100);
				continue;
			}
			k_msleep(2);
			esb_send_pair_step(2); // "acknowledge" pairing from receiver
			k_msleep(996);
		}
		set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_CONNECTION);
		LOG_INF("Paired");
		sys_write(
			PAIRED_ID,
			retained->paired_addr,
			paired_addr,
			sizeof(paired_addr)
		); // Write new address and tracker id
		esb_deinitialize();
		k_msleep(1600); // wait for led pattern
	}
	LOG_INF("Tracker ID: %u", paired_addr[1]);
	LOG_INF("Receiver address: %012llX", (*(uint64_t *)&retained->paired_addr[0] >> 16) & 0xFFFFFFFFFFFF);

	connection_set_id(paired_addr[1]);
	set_tracker_id(paired_addr[1]);

	esb_set_addr_paired();
	esb_paired = true;
	clocks_stop();
}

void esb_reset_pair(void)
{
	if (paired_addr[0] || esb_paired) {
		esb_deinitialize(); // make sure esb is off
		esb_paired = false;
		memset(paired_addr, 0, sizeof(paired_addr));
		LOG_INF("Pairing requested");
	}
}

void esb_clear_pair(void)
{
	esb_reset_pair();
	sys_write(PAIRED_ID, &retained->paired_addr, paired_addr,
			  sizeof(paired_addr)); // write zeroes
	LOG_INF("Pairing data reset");
}

void esb_process_ota_rx_queue(void)
{
	while (ota_rx_tail != ota_rx_head) {
		uint8_t idx = ota_rx_tail;
		if (ota_rx_queue[idx].data[0] != ESB_OTA_DATA_TYPE) {
			LOG_WRN("OTA queue: non-data type=0x%02X len=%u",
				ota_rx_queue[idx].data[0], ota_rx_queue[idx].length);
		}
		esb_ota_process_rx_packet(ota_rx_queue[idx].data,
					  ota_rx_queue[idx].length);
		__DMB();
		ota_rx_tail = (idx + 1) % OTA_RX_QUEUE_SIZE;
	}
}

void esb_write(uint8_t *data, bool no_ack, size_t data_length)
{
	if (!esb_initialized || !esb_paired) {
		return;
	}
	if (!clock_status) {
		clocks_start();
	}
	if (data_length < 1) {
		LOG_ERR("Invalid data length %u", data_length);
		return;
	}

	tx_payload.pipe = 1 + (tracker_id % 7);
	tx_payload.noack = no_ack;
	tx_payload.length = data_length;

	// int64_t now = k_uptime_get();
	// Tick rate counter
	esb_write_rate_tick();

	if (data[0] == ESB_PING_TYPE) {
		if (!server_time_synced) {
			LOG_DBG("Sending PING while time not synced - attempting to re-sync");
		}
		ping_pending = true;
		ping_failed = false;
		// Set sequence number
		data[2] = ping_counter;
		if (server_time_synced) {
			// TODO: Set local store server time if synced
			uint32_t server_time_ticks = (uint32_t)esb_get_server_time_ticks_64();
			data[3] = (server_time_ticks >> 24) & 0xFF;
			data[4] = (server_time_ticks >> 16) & 0xFF;
			data[5] = (server_time_ticks >> 8) & 0xFF;
			data[6] = server_time_ticks & 0xFF;
		}
		// Calculate crc8 checksum over first 12 bytes
		uint8_t crc_calc = crc8_ccitt(0x07, data, ESB_PING_LEN - 1);
		data[ESB_PING_LEN - 1] = crc_calc;
		ping_counter++;
	}
	memcpy(tx_payload.data, data, data_length);

	// Record this packet for TX_FAILED diagnostics
	last_tx.type = data[0];
	last_tx.noack = no_ack;
	last_tx.length = data_length;
	last_tx.timestamp = k_uptime_get();

	bool is_raw = (data[0] >= 0x10 && data[0] <= 0x14);

	/*
	 * TDMA slot gating / random backoff for noack sensor-data packets.
	 *
	 * Wait BEFORE queuing: if the packet were queued first and the radio
	 * were still auto-draining a previous TX, the new packet could be
	 * transmitted before the TDMA slot (bypassing the wait entirely).
	 *
	 * PING / ACK packets bypass this (no_ack == false) so time-sync and
	 * connection-health probes are never delayed.
	 * Raw data (0x10-0x14) always bypasses for minimum latency.
	 *
	 * When TDMA is disabled (compile-time or runtime), use random backoff
	 * to reduce collision
	 */
	if (no_ack && !is_raw) {
#if CONFIG_CONNECTION_TDMA
		if (tdma_is_enabled()) {
			tdma_wait_for_slot();
		} else
#endif
		{
			/* Random backoff: 0-1ms jitter using low bits of cycle counter */
			uint32_t jitter_us = (k_cycle_get_32() & 0x3FF) % 1000;
			if (jitter_us > 100) {
				k_usleep(jitter_us);
			}
		}
	}

	// Try to queue the packet (now inside the TDMA slot window)
	int queue_status = esb_write_payload(&tx_payload);
	// only flush if tx full
	if (queue_status == -ENOMEM) {
		esb_flush_tx();
		queue_status = esb_write_payload(&tx_payload);
	}

	// manually repeat raw IMU/mag packets for better reliability
	// Skip duplication for metadata (0x12) and calibration (0x14)
	// which are sent at controlled intervals with guaranteed delivery
	if (is_raw && data[0] != ESB_RAW_META_TYPE && data[0] != ESB_RAW_CAL_TYPE) {
		tx_payload.noack = true;
		queue_status = esb_write_payload(&tx_payload);
		esb_write_dup_queued++;
	}
# if 0
	if (no_ack && !is_raw) {
		// manually repeat packet for noack packets for better reliability
		int dup_ret = esb_write_payload(&tx_payload);
		if (dup_ret != 0) {
			LOG_WRN("Redundant copy queue failed: %d", dup_ret);
		} else {
			esb_write_dup_queued++;
		}
		queue_status = dup_ret;
	}
#endif
	// Record ping history metadata (timing updated after TDMA wait, just before TX)
	if (data[0] == ESB_PING_TYPE && queue_status == 0 && data_length == ESB_PING_LEN) {
		ping_history[ping_history_idx].counter = tx_payload.data[2];
		ping_ctr_sent = tx_payload.data[2];
		LOG_DBG("PING queued (ctr=%u)", (unsigned)tx_payload.data[2]);
	} else if (tx_payload.data[0] == ESB_PING_TYPE && queue_status != 0) {
		// PING failed to queue - this is critical!
		const char *err_str = "unknown";
		if (queue_status == -ENOMEM) {
			err_str = "ENOMEM (ESB not ready)";
		} else if (queue_status == -ENOSPC) {
			err_str = "ENOSPC (FIFO full)";
		} else if (queue_status == -EACCES) {
			err_str = "EACCES (access denied)";
		} else if (queue_status == -ENODATA) {
			err_str = "ENODATA (no data available)";
		}

		// Only log if this is the first failure or every 10th failure
		if (consecutive_enomem_errors == 1 || consecutive_enomem_errors % 10 == 0) {
			LOG_ERR(
				"esb_write: PING failed to queue (ctr=%u, err=%d %s, consecutive=%u)",
				tx_payload.data[2],
				queue_status,
				err_str,
				consecutive_enomem_errors
			);
		}
	}

	// Handle -ENOMEM error (ESB in bad state) with recovery mechanism
	if (queue_status == -ENOMEM || queue_status == -ENOSPC) {
		int64_t now = k_uptime_get();

		// Reset counter if this is the first error in a while
		if (now - last_enomem_time > ENOMEM_ERROR_WINDOW_MS) {
			consecutive_enomem_errors = 0;
		}

		consecutive_enomem_errors++;
		last_enomem_time = now;

		// Try simple flush first (only if ESB is idle)
		int flush_result = esb_flush_tx();

		if (flush_result == 0) {
			// Flush succeeded
			LOG_DBG("TX FIFO flushed successfully, err_count=%u", consecutive_enomem_errors);
			consecutive_enomem_errors = 0; // Reset after successful flush
		} else if (flush_result == -EBUSY) {
			// ESB is busy transmitting, this is normal - just wait
			if (consecutive_enomem_errors >= ENOMEM_ERROR_THRESHOLD) {
				// Only log warning if we've hit threshold
				LOG_WRN("ESB TX FIFO full for %u consecutive attempts (ESB busy)", consecutive_enomem_errors);

				// Only use suspend as last resort after many failures
				if (consecutive_enomem_errors >= ENOMEM_ERROR_THRESHOLD * 2) {
					LOG_ERR("Forcing recovery after %u errors", consecutive_enomem_errors);

					// Suspend and flush
					int suspend_result = esb_suspend();
					if (suspend_result == 0 || suspend_result == -EALREADY) {
						flush_result = esb_flush_tx();
						if (flush_result == 0) {
							LOG_INF("Recovered via suspend+flush");
							consecutive_enomem_errors = 0;
						} else {
							// Complete reinitialization
							LOG_ERR("Reinitializing ESB");
							esb_deinitialize();
							k_msleep(1);
							esb_initialize(true);
							consecutive_enomem_errors = 0;
						}
					}
				}
			}
			// Wait for hardware to finish current transmission
			k_msleep(1);
		}
	} else if (queue_status == 0) {
		// Success - reset error counter
		consecutive_enomem_errors = 0;
	}

	// Log error if queue failed
	if (queue_status != 0 && consecutive_enomem_errors % 10 == 1) {
		// Only log every 10th error to reduce noise
		LOG_ERR("esb_write: failed to queue packet, err=%d (logged every 10 errors)", queue_status);
	}

	// Record last TX time
	if (queue_status == 0) {
		last_tx_time = k_uptime_get();
		esb_write_queued++;
	}

	/*
	 * Record ping send timestamps here — after TDMA wait and queuing —
	 * so that ping_history[].ping_ticks and ping_send_time reflect the
	 * moment the radio actually begins transmitting.
	 */
	if (tx_payload.data[0] == ESB_PING_TYPE && queue_status == 0) {
		ping_history[ping_history_idx].ping_ticks = sys_clock_tick_get_32();
		ping_send_time = k_uptime_get();
		ping_history_idx = (ping_history_idx + 1) % PING_HISTORY_SIZE;
	}
	/*
	 * In MANUAL_START mode the radio auto-drains the FIFO once started.
	 * esb_start_tx() only needs to kick the first packet; if -EBUSY,
	 * the TX chain is already running and our queued packet will be
	 * sent automatically.  No retry or recovery needed.
	 */
	int tx_ret = esb_start_tx();
	if (tx_ret != 0 && tx_ret != -EBUSY) {
		LOG_WRN("esb_start_tx failed: %d", tx_ret);
	}
}

bool esb_ready(void)
{
	return esb_initialized && esb_paired;
}

uint8_t esb_get_ping_ack_flag(void)
{
	if (acked_remote_command != ESB_PONG_FLAG_NORMAL) {
		return acked_remote_command;
	}
	if (received_remote_command != ESB_PONG_FLAG_NORMAL) {
		return received_remote_command;
	}
	return ESB_PONG_FLAG_NORMAL;
}

uint64_t esb_get_server_time_ticks_64(void)
{
	if (!server_time_synced) {
		return 0;
	}

	int64_t now = k_uptime_get();
	if (now - g_last_sync_timestamp > TIME_SYNC_TIMEOUT_MS) {
		LOG_WRN("Time sync timeout: %lld ms since last sync, clearing sync state", now - g_last_sync_timestamp);
		server_time_synced = false;
		return 0;
	}

	uint32_t local_now = sys_clock_tick_get_32();
	// Apply clock skew compensation only for time elapsed since last PONG
	// (g_server_ticks_offset already incorporates all drift up to last measurement)
	uint32_t elapsed = local_now - g_last_sync_local_ticks;
	int32_t skew_correction = (int32_t)((int64_t)g_clock_skew_ppb * elapsed / 1000000000LL);
	return (uint32_t)(g_server_ticks_offset + local_now) + skew_correction;
}

uint64_t esb_get_server_time_us_64(void)
{
	uint64_t ticks = esb_get_server_time_ticks_64();

	return k_ticks_to_us_near64(ticks);
}

uint32_t esb_get_server_time(void)
{
	uint64_t ticks = esb_get_server_time_ticks_64();
	if (ticks == 0) {
		return 0;
	}
	uint64_t time_us = esb_get_server_time_us_64();
	return (uint32_t)(time_us / 1000ULL);
}

int64_t esb_get_sync_age_ms(void)
{
	if (!server_time_synced || g_last_sync_timestamp == 0) {
		return -1;
	}
	return k_uptime_get() - g_last_sync_timestamp;
}

static void esb_thread(void)
{
#if CONFIG_CONNECTION_OVER_HID
	int64_t start_time = k_uptime_get();
#endif

	/* Register ESB thread with watchdog */
	watchdog_register_thread(WDT_CHANNEL_ESB, 0);

	// Read paired address from retained
	memcpy(paired_addr, retained->paired_addr, sizeof(paired_addr));

	clocks_request_start(0);
	clock_init_external_async();

	while (1) {
#if CONFIG_CONNECTION_OVER_HID
		if (!esb_paired && get_status(SYS_STATUS_USB_CONNECTED) == false
			&& k_uptime_get() - 750 > start_time) // only automatically enter pairing while not
												  // potentially communicating by usb
#else
		if (!esb_paired)
#endif
		{
			esb_pair();
			esb_initialize(true);
		}
		// Check for shutdown timeout if connection errors persist
		if (ping_failures >= TX_ERROR_THRESHOLD && !test_mode_get()) {
#if CONFIG_CONNECTION_OVER_HID
			// only raise error while not potentially communicating by usb
			if (get_status(SYS_STATUS_CONNECTION_ERROR) == false && get_status(SYS_STATUS_USB_CONNECTED) == false && get_status(SYS_STATUS_CALIBRATION_RUNNING) == false)
#else
			if (get_status(SYS_STATUS_CONNECTION_ERROR) == false && get_status(SYS_STATUS_CALIBRATION_RUNNING) == false)
#endif
				set_status(SYS_STATUS_CONNECTION_ERROR, true);
#if USER_SHUTDOWN_ENABLED
			if (!shutdown_requested && connection_error_start_time > 0
				&& !connection_get_ota_suppressed()
				&& k_uptime_get() - connection_error_start_time
					   > CONFIG_CONNECTION_TIMEOUT_DELAY && get_status(SYS_STATUS_CALIBRATION_RUNNING) == false) // shutdown if receiver is not detected and not in calibrating
			{
				LOG_WRN("No response from receiver in %dm", CONFIG_CONNECTION_TIMEOUT_DELAY / 60000);
				shutdown_requested = true;
				sys_request_system_off(false);
			}
#endif
		}
		int64_t now_idle = k_uptime_get();

		if (received_remote_command != ESB_PONG_FLAG_NORMAL && received_remote_command != acked_remote_command
			&& remote_command_receive_time > 0) {
			/* OTA commands (0x30-0x33) bypass the safety delay since they are
			 * time-sensitive and not destructive like SHUTDOWN/CALIBRATE. */
			bool is_ota_cmd = (received_remote_command >= ESB_PONG_FLAG_OTA_QUERY_INFO &&
					   received_remote_command <= ESB_PONG_FLAG_OTA_UNSUPPRESS);
			if (is_ota_cmd || now_idle - remote_command_receive_time >= REMOTE_COMMAND_DELAY_MS) {
				switch (received_remote_command) {
				case ESB_PONG_FLAG_SHUTDOWN:
					LOG_WRN("Executing remote command: SHUTDOWN");
					sys_command_shutdown();
					break;

				case ESB_PONG_FLAG_CALIBRATE:
					LOG_INF("Executing remote command: CALIBRATE");
					sensor_request_calibration();
					break;

				case ESB_PONG_FLAG_SIX_SIDE_CAL:
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
					LOG_INF("Executing remote command: SIX_SIDE_CAL");
					sensor_request_calibration_6_side();
#else
					LOG_WRN("Remote command: SIX_SIDE_CAL not supported (disabled in config)");
#endif
					break;

				case ESB_PONG_FLAG_MEOW:
					LOG_INF("Executing remote command: MEOW");
					remote_print_meow();
					break;

				case ESB_PONG_FLAG_SCAN:
					LOG_INF("Executing remote command: SCAN");
					sensor_request_scan(true);
					break;

				case ESB_PONG_FLAG_MAG_CLEAR:
					LOG_INF("Executing remote command: MAG_CLEAR");
					sensor_calibration_clear_mag(NULL, true);
					break;

				case ESB_PONG_FLAG_MAG_CAL:
					LOG_INF("Executing remote command: MAG_CAL");
					sensor_calibration_clear_mag(NULL, true);
					sensor_request_calibration_mag();
					break;

				case ESB_PONG_FLAG_MAG_ON:
					LOG_INF("Executing remote command: MAG_ON");
					sensor_set_mag_enabled(true);
					break;

				case ESB_PONG_FLAG_MAG_OFF:
					LOG_INF("Executing remote command: MAG_OFF");
					sensor_set_mag_enabled(false);
					break;

				case ESB_PONG_FLAG_TCAL_ON:
#if CONFIG_SENSOR_USE_TCAL
					LOG_INF("Executing remote command: TCAL_ON");
					sensor_tcal_set_enabled(true);
#endif
					break;

				case ESB_PONG_FLAG_TCAL_OFF:
#if CONFIG_SENSOR_USE_TCAL
					LOG_INF("Executing remote command: TCAL_OFF");
					sensor_tcal_set_enabled(false);
#endif
					break;

				case ESB_PONG_FLAG_TDMA_ON:
					LOG_INF("Executing remote command: TDMA_ON");
					tdma_set_enabled(true);
					break;

				case ESB_PONG_FLAG_TDMA_OFF:
					LOG_INF("Executing remote command: TDMA_OFF");
					tdma_set_enabled(false);
					break;

				case ESB_PONG_FLAG_TEST_MODE_ON:
					LOG_INF("Executing remote command: TEST_MODE_ON");
					test_mode_set(true);
					break;

				case ESB_PONG_FLAG_TEST_MODE_OFF:
					LOG_INF("Executing remote command: TEST_MODE_OFF");
					test_mode_set(false);
					break;

				case ESB_PONG_FLAG_REBOOT:
					LOG_WRN("Executing remote command: REBOOT");
					sys_request_system_reboot(false);
					break;

				case ESB_PONG_FLAG_CLEAR:
					LOG_WRN("Executing remote command: CLEAR (clear pairing)");
					esb_clear_pair();
					break;

				case ESB_PONG_FLAG_DFU:
#if CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
					LOG_WRN("Executing remote command: DFU (enter bootloader)");
#if CONFIG_BUILD_OUTPUT_UF2
					NRF_POWER->GPREGRET = ADAFRUIT_DFU_MAGIC_UF2_RESET;
					k_msleep(100);
#endif
					sys_request_system_reboot(false);
#else
					LOG_WRN("Remote command: DFU not supported (no bootloader)");
#endif
					break;

				case ESB_PONG_FLAG_DFU_OTA:
#if CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
					LOG_WRN("Executing remote command: DFU_OTA (enter OTA bootloader)");
#if CONFIG_BUILD_OUTPUT_UF2
					NRF_POWER->GPREGRET = ADAFRUIT_DFU_MAGIC_OTA_RESET;
					k_msleep(2);
#endif
					sys_request_system_reboot(false);
#else
					LOG_WRN("Remote command: DFU_OTA not supported (no bootloader)");
#endif
					break;

				case ESB_PONG_FLAG_SET_CHANNEL: {
					// Validate channel value (0-100)
					if (received_channel_value <= 100) {
						LOG_INF("Executing remote command: SET_CHANNEL to %u", received_channel_value);
						// Save to retained memory
						retained->rf_channel = (uint8_t)received_channel_value;
						retained_update();
						// Save to NVS
						sys_write(
							RF_CHANNEL_ID,
							&retained->rf_channel,
							&retained->rf_channel,
							sizeof(retained->rf_channel)
						);
						LOG_INF("RF channel saved to NVS: %u", retained->rf_channel);
						// Reinitialize ESB with new channel
						esb_deinitialize();
						k_msleep(10);
						esb_initialize(true); // Channel will be applied inside esb_initialize
						LOG_INF("ESB reinitialized with channel %u", retained->rf_channel);
					} else {
						LOG_ERR("Invalid channel value: %u (must be 0-100)", received_channel_value);
					}
				} break;

				case ESB_PONG_FLAG_CLEAR_CHANNEL:
					LOG_INF("Executing remote command: CLEAR_CHANNEL (restore default)");
					// Clear saved channel (set to 0xFF = use default)
					retained->rf_channel = 0xFF;
					retained_update();
					sys_write(
						RF_CHANNEL_ID,
						&retained->rf_channel,
						&retained->rf_channel,
						sizeof(retained->rf_channel)
					);
					LOG_INF("RF channel cleared, will use default on next boot");
					// Reinitialize ESB with default channel
					esb_deinitialize();
					k_msleep(10);
					esb_initialize(true); // Will use default channel since rf_channel is 0xFF
					LOG_INF("ESB reinitialized with default channel %u", RADIO_RF_CHANNEL);
					break;

				case ESB_PONG_FLAG_SENS_SET:
					LOG_INF("Executing remote command: SENS_SET");
					cmd_sens_set(received_sens_data[0], received_sens_data[1], received_sens_data[2]);
					break;

				case ESB_PONG_FLAG_SENS_RESET:
					LOG_INF("Executing remote command: SENS_RESET");
					cmd_sens_reset();
					break;

				case ESB_PONG_FLAG_SENS_AUTO:
					LOG_INF(
						"Executing remote command: SENS_AUTO axis=%u revolutions=%u",
						received_sens_auto_axis,
						received_sens_auto_revolutions
					);
					cmd_sens_auto_request(received_sens_auto_axis, received_sens_auto_revolutions);
					break;

				case ESB_PONG_FLAG_RESET_ZRO:
					LOG_INF("Executing remote command: RESET_ZRO");
					cmd_reset_zro();
					break;

				case ESB_PONG_FLAG_RESET_ACC:
					LOG_INF("Executing remote command: RESET_ACC");
					cmd_reset_acc();
					break;

				case ESB_PONG_FLAG_RESET_BAT:
					LOG_INF("Executing remote command: RESET_BAT");
					cmd_reset_bat();
					break;

				case ESB_PONG_FLAG_RESET_TCAL:
					LOG_INF("Executing remote command: RESET_TCAL");
					cmd_reset_tcal();
					break;

				case ESB_PONG_FLAG_TCAL_AUTO_ON:
#if CONFIG_SENSOR_USE_TCAL
					LOG_INF("Executing remote command: TCAL_AUTO_ON");
					sensor_tcal_set_auto_calibration(true);
#else
					LOG_WRN("Remote command: TCAL_AUTO_ON not supported (T-Cal disabled in config)");
#endif
					break;

				case ESB_PONG_FLAG_TCAL_AUTO_OFF:
#if CONFIG_SENSOR_USE_TCAL
					LOG_INF("Executing remote command: TCAL_AUTO_OFF");
					sensor_tcal_set_auto_calibration(false);
#else
					LOG_WRN("Remote command: TCAL_AUTO_OFF not supported (T-Cal disabled in config)");
#endif
					break;

				case ESB_PONG_FLAG_PING:
					LOG_INF("Executing remote command: PING");
					cmd_ping_start();
					break;

				case ESB_PONG_FLAG_FUSION_RESET:
					LOG_INF("Executing remote command: FUSION_RESET");
					cmd_fusion_reset();
					break;

				case ESB_PONG_FLAG_TCAL_BOOT_ON:
#if CONFIG_SENSOR_USE_TCAL
					LOG_INF("Executing remote command: TCAL_BOOT_ON");
					sensor_boot_cal_set_enabled(true);
#else
					LOG_WRN("Remote command: TCAL_BOOT_ON not supported (T-Cal disabled in config)");
#endif
					break;

				case ESB_PONG_FLAG_TCAL_BOOT_OFF:
#if CONFIG_SENSOR_USE_TCAL
					LOG_INF("Executing remote command: TCAL_BOOT_OFF");
					sensor_boot_cal_set_enabled(false);
#else
					LOG_WRN("Remote command: TCAL_BOOT_OFF not supported (T-Cal disabled in config)");
#endif
					break;

				case ESB_PONG_FLAG_DATA_COLLECT_ON:
					LOG_INF("Executing remote command: DATA_COLLECT_ON");
					connection_set_data_collection(true);
					test_mode_set(true);  // Prevent sleep during data collection
					break;

				case ESB_PONG_FLAG_DATA_COLLECT_OFF:
					LOG_INF("Executing remote command: DATA_COLLECT_OFF");
					connection_set_data_collection(false);
					test_mode_set(false);
					break;

				case ESB_PONG_FLAG_OTA_QUERY_INFO:
					LOG_INF("Executing remote command: OTA_QUERY_INFO");
					esb_ota_handle_query_info();
					break;

				case ESB_PONG_FLAG_OTA_ABORT:
					LOG_WRN("Executing remote command: OTA_ABORT");
					esb_ota_handle_abort();
					break;

				case ESB_PONG_FLAG_OTA_SUPPRESS:
					LOG_INF("Executing remote command: OTA_SUPPRESS (reducing poll rate)");
					connection_set_ota_suppressed(true);
					break;

				case ESB_PONG_FLAG_OTA_UNSUPPRESS:
					LOG_INF("Executing remote command: OTA_UNSUPPRESS (resuming normal rate)");
					connection_set_ota_suppressed(false);
					break;

				default:
					LOG_WRN("Unknown remote command: 0x%02X", received_remote_command);
					break;
				}

				acked_remote_command = received_remote_command;

				if (received_remote_command == ESB_PONG_FLAG_SHUTDOWN) {
					return;
				}
			}
		}

		if (ping_pending && (now_idle - ping_send_time) > (get_ping_interval_ms() - 100)) {
			// Consider missing PONG a failure, clear pending
			ping_failed = true;
			ping_pending = false;
			ping_success_streak = 0;
			ping_failures++;
			LOG_WRN("PING timeout, failures=%u", ping_failures);
			if (ping_failures == TX_ERROR_THRESHOLD) {
				connection_error_start_time = now_idle;
				LOG_WRN(
					"Ping failure threshold reached (%d failures), starting "
					"timeout timer",
					TX_ERROR_THRESHOLD
				);
			}
		}

		/* Feed watchdog at end of each loop iteration */
		watchdog_feed(WDT_CHANNEL_ESB);

		k_msleep(100);
	}
}
