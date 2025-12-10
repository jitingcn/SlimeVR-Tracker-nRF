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
#include "connection.h"
#include "zephyr/logging/log.h"
#include "zephyr/sys/time_units.h"

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */
#include <hal/nrf_timer.h>
#include <nrfx_timer.h>
#include <zephyr/sys/crc.h>
#include <zephyr/kernel.h>

#include "util.h"
#include "esb.h"
#include "console.h"
#include "timer.h"

uint8_t last_reset = 0;
// const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);
bool esb_state = false;
bool timer_state = false;
bool send_data = false;
uint16_t led_clock = 0;
uint32_t led_clock_offset = 0;

int64_t connection_error_start_time = 0;
static bool shutdown_requested = false;
static bool pair_ack_pending = false; // True once step 1 is sent and we expect a receiver response

static struct esb_payload rx_payload;
// Normal data payload (16+1 bytes when used), length set per write
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0, 0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload ack_payload = ESB_CREATE_PAYLOAD(0);

static uint8_t paired_addr[8] = {0};

static bool esb_initialized = false;
static bool esb_paired = false;

#define TX_ERROR_THRESHOLD 270
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

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 512, esb_thread, NULL, NULL, NULL, 6, 0, 0);

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
#define REMOTE_COMMAND_DELAY_MS 1500

// Server time synchronization for TDMA scheduling (using ticks)
static bool server_time_synced = false;

// Server time synchronization
static uint32_t g_server_ticks_offset = 0;
static uint32_t g_last_rx_raw_ticks = 0;
static uint32_t g_last_sync_local_ticks = 0;
static bool g_time_initialized = false;
static int64_t g_last_sync_timestamp = 0;
static int32_t g_clock_skew_fixed = 0; // Fixed point skew (20 bit fraction)
static int64_t g_last_skew_update_time = 0;
static uint32_t g_min_rtt_ticks = 0; // Minimum RTT for outlier detection
// 2 minute timeout for time sync
#define TIME_SYNC_TIMEOUT_MS 120000
#define SKEW_SHIFT 20

// Track last sent packet for TX_FAILED diagnostics
struct last_tx_info {
	uint8_t type;        // First byte of payload (packet type)
	bool is_ack_payload; // Is this the small ack_payload after PING
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
static int64_t esb_rate_last_ts = 0;

void esb_write_rate_tick(void)
{
	int64_t now = k_uptime_get();
	if (esb_rate_last_ts == 0) {
		esb_rate_last_ts = now;
	}
	esb_write_calls++;
	if (now - esb_rate_last_ts >= 3000) {
		LOG_INF("esb_write rate: calls=%u/s", esb_write_calls / 3);
		esb_write_calls = 0;
		esb_rate_last_ts = now;
	}
}

// ESB recovery mechanism for persistent ENOMEM errors
static uint32_t consecutive_enomem_errors = 0;
static int64_t last_enomem_time = 0;
#define ENOMEM_ERROR_THRESHOLD 3    // Force recovery after N consecutive errors
#define ENOMEM_ERROR_WINDOW_MS 1000 // Reset counter if no error for this duration

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
	clock_status = false;

	onoff_release(clk_mgr);

	LOG_DBG("HF clock stop request");
}

#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif

static struct k_thread clocks_thread_id;
static K_THREAD_STACK_DEFINE(clocks_thread_id_stack, 128);

void clocks_request_start(uint32_t delay_us)
{
	k_thread_create(
		&clocks_thread_id,
		clocks_thread_id_stack,
		K_THREAD_STACK_SIZEOF(clocks_thread_id_stack),
		(k_thread_entry_t)clocks_start,
		NULL,
		NULL,
		NULL,
		5,
		0,
		K_USEC(delay_us)
	);
}

static struct k_thread clocks_stop_thread_id;
static K_THREAD_STACK_DEFINE(clocks_stop_thread_id_stack, 128);

void clocks_request_stop(uint32_t delay_us)
{
	k_thread_create(
		&clocks_stop_thread_id,
		clocks_stop_thread_id_stack,
		K_THREAD_STACK_SIZEOF(clocks_stop_thread_id),
		(k_thread_entry_t)clocks_stop,
		NULL,
		NULL,
		NULL,
		5,
		0,
		K_USEC(delay_us)
	);
}

int esb_write_ack(uint8_t type)
{
	// This small ACK packet is crucial for receiving the PONG ACK payload
	// ESB requires a transmission to trigger ACK payload reception
	if (!esb_initialized || !esb_paired) {
		return 0;
	}
	// Check ESB TX FIFO before adding packet
	if (esb_tx_full()) {
		LOG_WRN("TX FIFO full when queuing ACK packet, skipping this ACK");
		// Don't try to recover here - let the main error handler deal with it
		// Forcing recovery for every ACK is too aggressive
		return -ENOSPC;
	}

	// small packet with no data, just to get ack result
	ack_payload.pipe = 1 + (tracker_id % 7);
	ack_payload.noack = false;
	ack_payload.length = 1;
	ack_payload.data[0] = 0x00; // Empty payload marker

	// Record ack_payload for diagnostics
	last_tx.type = type; // ack for last sent packet type
	last_tx.is_ack_payload = true;
	last_tx.noack = false;
	last_tx.length = 1;
	last_tx.timestamp = k_uptime_get();

	int ack_status = esb_write_payload(&ack_payload);
	if (ack_status != 0) {
		const char *err_str = "unknown";
		if (ack_status == -ENOMEM) {
			err_str = "ENOMEM (ESB not ready)";
			consecutive_enomem_errors++;
		} else if (ack_status == -ENOSPC) {
			err_str = "ENOSPC (FIFO full)";
		}
		LOG_ERR("esb_write_ack: failed to queue ACK packet (err=%d %s)", ack_status, err_str);
	} else {
		LOG_DBG("ACK packet queued to trigger PONG reception (type=0x%02X)", type);
	}
	return ack_status;
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
		if (esb_paired && last_tx.type != ESB_PING_TYPE) {
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

		// Log TX statistics every 20 failures for debugging
		uint32_t now = k_uptime_get_32();
		if (tx_failed_count % 20 == 0 || (now - last_log_time > get_ping_interval_ms())) {
			last_log_time = now;
			uint32_t total = tx_success_count + tx_failed_count;
			uint32_t fail_rate = total > 0 ? (tx_failed_count * 100 / total) : 0;
			LOG_WRN("TX Stats: success=%u failed=%u rate=%u%%", tx_success_count, tx_failed_count, fail_rate);
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

		if (esb_paired && last_tx.type != ESB_PING_TYPE) {
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
			case 4: {
				// TODO: Device should never receive packets if it is already
				// paired, why is this packet received? This may be part of
				// acknowledge
				//					if (!nrfx_timer_init_check(&m_timer))
				{
					LOG_WRN("Timer not initialized");
					break;
				}
				if (timer_state == false) {
					//						nrfx_timer_resume(&m_timer);
					timer_state = true;
				}
				//					nrfx_timer_clear(&m_timer);
				last_reset = 0;
				led_clock = (rx_payload.data[0] << 8) + rx_payload.data[1]; // sync led flashes :)
				led_clock_offset = 0;
				LOG_DBG("RX, timer reset");
				pair_ack_pending = false;
			} break;
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

					if (pong_flags == ESB_PONG_FLAG_SENS_SET) {
						// Special case: SENS_SET command repurposes time sync bytes for data
						// Skip time sync update
						int16_t x_int = (int16_t)((rx_payload.data[3] << 8) | rx_payload.data[4]);
						int16_t y_int = (int16_t)((rx_payload.data[5] << 8) | rx_payload.data[6]);
						int16_t z_int = (int16_t)((rx_payload.data[8] << 8) | rx_payload.data[9]);

						received_sens_data[0] = (float)x_int / 100.0f;
						received_sens_data[1] = (float)y_int / 100.0f;
						received_sens_data[2] = (float)z_int / 100.0f;

						LOG_INF(
							"Received SENS_SET data: %.2f, %.2f, %.2f",
							(double)received_sens_data[0],
							(double)received_sens_data[1],
							(double)received_sens_data[2]
						);
					} else if (ping_ticks_for_this_ctr != 0) {
						// Calculate RTT: from PING send to PONG receive
						// Note: current_rx_ticks - ping_ticks_for_this_ctr is the full RTT
						uint32_t ticks_diff = current_rx_ticks - ping_ticks_for_this_ctr;
						rtt_us = k_ticks_to_us_floor32(ticks_diff);

						// log ping and rtt
						if (rtt_us > 1000) {
							LOG_INF(
								"PONG ok, ack rtt=%u.%03u ms (ctr=%u)",
								(unsigned)(rtt_us / 1000),
								(unsigned)(rtt_us % 1000),
								rx_ctr
							);
						} else if (rtt_us < 1000) {
							LOG_INF("PONG ok, ack rtt=%u us (ctr=%u)", (unsigned)rtt_us, rx_ctr);
						}

						if (rtt_us < 1200) {
							// Calculate RTT in ticks
							uint32_t rtt_ticks = current_rx_ticks - ping_ticks_for_this_ctr;

							// Track minimum RTT (with slow decay to adapt to changing conditions)
							// Minimum RTT is more reliable than average because delay spikes are common
							if (g_min_rtt_ticks == 0 || rtt_ticks < g_min_rtt_ticks) {
								g_min_rtt_ticks = rtt_ticks;
							} else {
								// Slowly increase min if no new minimum (handles link quality changes)
								// Increase by 1 tick every update (~1 second)
								g_min_rtt_ticks += 1;
							}

							// Skip update if RTT is abnormally high (> 1.5x minimum)
							// Using 1.5x to catch marginal spikes that still cause drift
							// E.g., min=32 ticks (976us) -> threshold=48 ticks (1464us)
							bool rtt_outlier = (rtt_ticks > g_min_rtt_ticks + (g_min_rtt_ticks >> 1));

							// Also mark "marginal" RTT (> 1.25x min) - good for offset update,
							// but skip skew update as the measurement is less reliable
							bool rtt_marginal = !rtt_outlier && (rtt_ticks > g_min_rtt_ticks + (g_min_rtt_ticks >> 2));

							// Parse receiver's observed ticks_diff from PONG data[8-11]
							// ticks_diff = receiver_rx_time - tracker_estimated_server_time
							// If tracker's estimate is perfect, ticks_diff = uplink_delay
							int32_t receiver_ticks_diff = 0;
							if (pong_flags == ESB_PONG_FLAG_NORMAL) {
								receiver_ticks_diff
									= (int32_t)(((uint32_t)rx_payload.data[8] << 24)
												| ((uint32_t)rx_payload.data[9] << 16)
												| ((uint32_t)rx_payload.data[10] << 8) | rx_payload.data[11]);
							}

							// Use ticks_diff directly as error signal.
							// If ticks_diff > 0, tracker estimate is too low, increase offset
							// If ticks_diff < 0, tracker estimate is too high, decrease offset
							// Target is RTT - 650us (safety margin)
							// This ensures we are slightly "late" (safe) rather than "early" (violation)
							int32_t target_diff = (int32_t)(g_min_rtt_ticks - k_us_to_ticks_near32(650));
							if (target_diff < 0) {
								target_diff = 0;
							}
							int32_t sync_error = receiver_ticks_diff - target_diff;

							// Calculate current skew contribution (drift since last sync)
							int32_t current_skew = 0;
							if (g_clock_skew_fixed != 0 && g_last_sync_local_ticks > 0) {
								uint32_t elapsed = ping_ticks_for_this_ctr - g_last_sync_local_ticks;
								current_skew = (int32_t)(((int64_t)elapsed * g_clock_skew_fixed) >> SKEW_SHIFT);
							}

							// Predict offset with skew correction
							int32_t predicted_offset = g_server_ticks_offset + current_skew;

							// Estimate current server time for display
							int32_t estimated_server_ticks = (int32_t)(predicted_offset + sys_clock_tick_get_32());

							if (rtt_outlier) {
								LOG_INF(
									"sync: SKIP rtt=%u (min=%u) rx_diff=%d",
									rtt_ticks,
									g_min_rtt_ticks,
									receiver_ticks_diff
								);
								// DON'T apply skew prediction - keep offset unchanged
								// This prevents offset drift during consecutive skips
								// g_server_ticks_offset stays the same
								// Also skip skew update when RTT is outlier
								// Don't update g_last_sync_local_ticks/g_last_rx_raw_ticks
								// to avoid polluting skew calculation with outlier data
								// Note: Don't notify timer - RTT outlier is normal, keep TDMA running
								g_last_sync_timestamp = k_uptime_get();
								server_time_synced = true;
								// Skip the rest of the sync logic
								break;
							} else {
								LOG_INF(
									"sync: rx_diff=%d rtt=%u (min=%u) err=%d offset=%d skew=%d",
									receiver_ticks_diff,
									rtt_ticks,
									g_min_rtt_ticks,
									sync_error,
									g_server_ticks_offset,
									current_skew
								);
							}
							// g_last_rx_raw_ticks update moved to after skew calculation

							// Calculate skew BEFORE updating g_last_sync_local_ticks
							// Skip skew calculation if RTT is marginal (> 1.25x min) to prevent
							// unreliable measurements from polluting skew estimation
							if (pong_flags == ESB_PONG_FLAG_NORMAL && g_time_initialized && g_last_sync_local_ticks > 0
								&& g_last_rx_raw_ticks > 0 && !rtt_marginal) {

								// Calculate intervals based on raw timestamps (independent of offset)
								uint32_t local_interval = ping_ticks_for_this_ctr - g_last_sync_local_ticks;
								uint32_t remote_interval = ping_rx_ticks - g_last_rx_raw_ticks;

								// Drift = Remote Interval - Local Interval
								// If drift < 0, remote is slower (or local is faster)
								int32_t drift = (int32_t)(remote_interval - local_interval);

								// Calculate expected drift based on current skew compensation
								// This is what we are already compensating for
								int32_t expected_drift
									= (int32_t)(((int64_t)local_interval * g_clock_skew_fixed) >> SKEW_SHIFT);

								// Calculate residual drift (error term)
								// This is the drift that is NOT yet compensated
								int32_t residual_drift = drift - expected_drift;

								// Normalize drift to per-second rate for outlier detection
								// This handles varying interval lengths properly
								int32_t drift_per_sec
									= (local_interval > 0)
										? (int32_t)(((int64_t)ABS(residual_drift) * 32768) / local_interval)
										: 0;

								// Outlier detection: drift > 500 ticks/sec (~15ms/sec) is abnormal
								// Normal clock drift is 10-100 ppm, which is 0.3-3 ticks/sec
								if (drift_per_sec > 500) {
									// Skip this skew update - likely measurement error
									LOG_WRN(
										"Drift outlier: %d ticks in %u interval (rate=%d/sec), skipping",
										residual_drift,
										local_interval,
										drift_per_sec
									);
								} else if (ABS(drift) > 3000) {
									// Reset skew on outlier - likely restart event
									LOG_WRN(
										"Large drift detected (%d ticks), resetting sync "
										"(local_int=%u remote_int=%u)",
										drift,
										local_interval,
										remote_interval
									);
									g_clock_skew_fixed = 0;
									g_last_skew_update_time = 0;
									g_time_initialized = false;
								} else if (local_interval > 3200) { // Only update if interval is sufficient (> 100ms)
									// Calculate skew error in fixed point based on RESIDUAL drift
									int64_t diff_shifted = ((int64_t)residual_drift) << SKEW_SHIFT;
									int32_t skew_error_fixed = (int32_t)(diff_shifted / local_interval);
									int32_t abs_error = ABS(skew_error_fixed);

									// Adaptive gain control (hill-climbing algorithm)
									// Fast descent to local minimum, then fine adjustment
									int gain_shift;
									const char *gain_desc;
									if (abs_error > 1000) {
										// Large error: fast convergence with gain 1/2
										gain_shift = 1;
										gain_desc = "fast";
									} else if (abs_error > 300) {
										// Medium error: normal adjustment with gain 1/4
										gain_shift = 2;
										gain_desc = "normal";
									} else {
										// Small error: fine tuning with gain 1/8
										gain_shift = 3;
										gain_desc = "fine";
									}

									// Accumulate skew error with adaptive gain
									g_clock_skew_fixed += skew_error_fixed >> gain_shift;

									g_last_skew_update_time = k_uptime_get();
									LOG_INF(
										"Skew update (%s): drift=%d interval=%u error_fixed=%d total_skew_fixed=%d",
										gain_desc,
										drift,
										local_interval,
										skew_error_fixed,
										g_clock_skew_fixed
									);

									// Notify timer module for TDMA enable/disable
									// The original code had `timer_on_skew_update(offset_diff);` here.
									// `offset_diff` is no longer calculated in the same way.
									// For now, we'll pass 0 or re-evaluate what `offset_diff` should represent here.
									// Given the new logic, `sync_error` is the primary error term for offset.
									// Let's use `sync_error` if it's available, otherwise 0.
									timer_on_skew_update(sync_error);
								}
							}

							g_last_rx_raw_ticks = ping_rx_ticks;
							g_last_sync_local_ticks = ping_ticks_for_this_ctr;
							g_last_sync_timestamp = k_uptime_get();

							// Update offset based on receiver feedback
							// Only use receiver feedback when flag is NORMAL (contains valid ticks_diff)
							// Skip error-based update if RTT is an outlier
							if (!g_time_initialized) {
								// First sync: use T2-T1-RTT/2 as initial estimate
								int32_t initial_offset
									= (int32_t)(ping_rx_ticks - ping_ticks_for_this_ctr - rtt_ticks / 2);
								g_server_ticks_offset = initial_offset;
								g_time_initialized = true;
								LOG_INF("Server offset initialized: %d ticks (T2-T1-RTT/2)", initial_offset);
							} else if (rtt_outlier) {
								// RTT outlier detected - skip error correction, just apply skew
								// (offset already set to predicted_offset in earlier block)
							} else if (pong_flags == ESB_PONG_FLAG_NORMAL && receiver_ticks_diff != 0) {
								// Use receiver's feedback to correct offset
								// sync_error tells us how much we're off
								int32_t abs_error = (sync_error < 0) ? -sync_error : sync_error;

								// Large error detection (> 1 second)
								if (abs_error > 32000) {
									// Reset using traditional method
									int32_t reset_offset
										= (int32_t)(ping_rx_ticks - ping_ticks_for_this_ctr - rtt_ticks / 2);
									LOG_WRN(
										"Large sync error (%d ticks), resetting offset to %d",
										sync_error,
										reset_offset
									);
									g_server_ticks_offset = reset_offset;
								} else {
									// Apply partial correction with deadband to filter noise
									// - predicted_offset accounts for clock drift (skew)
									// - sync_error corrects remaining offset error
									// Using 50% gain prevents single measurement spikes from
									// causing large offset jumps while still converging
									if (abs_error <= 3) {
										// Within deadband: only apply skew prediction, no error correction
										// This prevents noise from causing continuous drift
										g_server_ticks_offset = predicted_offset;
										LOG_INF(
											"Offset update: pred=%d (err=%d in deadband)",
											predicted_offset,
											sync_error
										);
									} else {
										// Apply 50% of sync_error to smooth out spikes
										int32_t applied_error = sync_error >> 1;
										g_server_ticks_offset = predicted_offset + applied_error;
										LOG_INF(
											"Offset update: pred=%d + err=%d/2 = new=%d",
											predicted_offset,
											sync_error,
											g_server_ticks_offset
										);
									}
								}
							} else {
								// Non-NORMAL flag or no feedback, just apply skew prediction
								g_server_ticks_offset = predicted_offset;
							}
							server_time_synced = true;

							// Convert to ms for human-readable display
							uint32_t server_time_ms = k_ticks_to_ms_near32(estimated_server_ticks);
							uint32_t server_ms = server_time_ms % 1000;
							uint32_t server_s = (server_time_ms / 1000) % 60;
							uint32_t server_m = (server_time_ms / 60000) % 60;
							uint32_t server_h = (server_time_ms / 3600000) % 24;
							LOG_INF(
								"estimated server time: %02u:%02u:%02u.%03u (ticks=%u)",
								server_h,
								server_m,
								server_s,
								server_ms,
								estimated_server_ticks
							);
						}
					} else {
						// No history found - likely too old or buffer wrapped
					}

					// handle remote commands and delayed execution
					if (pong_flags != ESB_PONG_FLAG_NORMAL) {
						if (received_remote_command == ESB_PONG_FLAG_NORMAL) {
							// new command received
							received_remote_command = pong_flags;
							remote_command_receive_time = k_uptime_get();

							// For SET_CHANNEL command, extract channel value from data[8-11]
							if (pong_flags == ESB_PONG_FLAG_SET_CHANNEL) {
								received_channel_value
									= ((uint32_t)rx_payload.data[8] << 24) | ((uint32_t)rx_payload.data[9] << 16)
									| ((uint32_t)rx_payload.data[10] << 8) | ((uint32_t)rx_payload.data[11]);
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
							case ESB_PONG_FLAG_REBOOT:
								cmd_name = "REBOOT";
								break;
							case ESB_PONG_FLAG_CLEAR:
								cmd_name = "CLEAR";
								break;
							case ESB_PONG_FLAG_DFU:
								cmd_name = "DFU";
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
							case ESB_PONG_FLAG_RESET_ZRO:
								cmd_name = "RESET_ZRO";
								break;
							case ESB_PONG_FLAG_RESET_ACC:
								cmd_name = "RESET_ACC";
								break;
							case ESB_PONG_FLAG_RESET_BAT:
								cmd_name = "RESET_BAT";
								break;
							case ESB_PONG_FLAG_PING:
								cmd_name = "PING";
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
								LOG_INF(
									"Remote command %s (0x%02X) received, will execute in %dms",
									cmd_name,
									pong_flags,
									REMOTE_COMMAND_DELAY_MS
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
			default:
				LOG_WRN("Ignoring invalid payload length %u", rx_payload.length);
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
		config.retransmit_count = CONNECTION_ENABLE_ACK ? 2 : 5;
		config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
		// config.use_fast_ramp_up = true;
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
		// config.payload_length = 32;
		config.selective_auto_ack = true;
		// config.use_fast_ramp_up = true;
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
		k_msleep(10); // wait for pending transmissions
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
	server_time_synced = false;
	g_time_initialized = false;
	g_last_sync_timestamp = 0;
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

	// Initialize TDMA timer with tracker ID
	timer_set_tracker_id(paired_addr[1]);
	timer_init();

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

void esb_write(uint8_t *data, bool no_ack, size_t data_length)
{
	if (!esb_initialized || !esb_paired) {
		return;
	}
	if (!clock_status) {
		clocks_start();
		k_usleep(500);
	}
	if (data_length < 1) {
		LOG_ERR("Invalid data length %u", data_length);
		return;
	}
	if (esb_tx_full()) {
		LOG_DBG("ESB TX fifo full");
		esb_flush_tx();
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
		ping_send_time = k_uptime_get();
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
	last_tx.is_ack_payload = false;
	last_tx.noack = no_ack;
	last_tx.length = data_length;
	last_tx.timestamp = k_uptime_get();

	// Try to queue the packet
	// esb_flush_tx();
	int queue_status = esb_write_payload(&tx_payload);

	// if sending ping packet, we need send another small packet to get ack result asap
	if (data[0] == ESB_PING_TYPE && queue_status == 0 && data_length == ESB_PING_LEN) {
		uint8_t this_ctr = tx_payload.data[2];
		ping_history[ping_history_idx].counter = this_ctr;
		ping_history[ping_history_idx].ping_ticks = sys_clock_tick_get_32();
		LOG_DBG("PING sent (ctr=%u)", (unsigned)tx_payload.data[2]);
		ping_pending = true;
		ping_ctr_sent = tx_payload.data[2];
		ping_send_time = k_uptime_get();
		// Record cycles for THIS counter in circular buffer (for accurate RTT calculation)
		ping_history_idx = (ping_history_idx + 1) % PING_HISTORY_SIZE;
		esb_start_tx();
		while (!esb_is_idle()) {
			k_usleep(10);
		}
		queue_status = esb_write_ack(ESB_PING_TYPE);
		// wait for esb send all packets
		esb_start_tx();
		return;
	}

	if (tx_payload.data[0] == ESB_PING_TYPE && queue_status != 0) {
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

	// Use timer for TDMA-controlled TX timing
	timer_signal_tx_pending();
	send_data = true;
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

	int64_t skew_correction = 0;
	if (g_clock_skew_fixed != 0) {
		uint32_t current_ticks = sys_clock_tick_get_32();
		// Calculate elapsed time since last sync point
		uint32_t elapsed = current_ticks - g_last_sync_local_ticks;
		skew_correction = ((int64_t)elapsed * g_clock_skew_fixed) >> SKEW_SHIFT;
	}

	return g_server_ticks_offset + sys_clock_tick_get_32() + skew_correction;
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

static void esb_thread(void)
{
#if CONFIG_CONNECTION_OVER_HID
	int64_t start_time = k_uptime_get();
#endif

	// Read paired address from retained
	memcpy(paired_addr, retained->paired_addr, sizeof(paired_addr));

	// Safe runtime LF clock switch to external XTAL for TDMA precision
	// This must be done very carefully to avoid system hang
#if defined(CONFIG_CLOCK_CONTROL_NRF_K32SRC_XTAL)
	LOG_INF("=== LF Clock Switch Start ===");
	LOG_INF("Current LF source: %d, running: %d",
		nrf_clock_lf_src_get(NRF_CLOCK),
		nrf_clock_lf_is_running(NRF_CLOCK));

	// Check if already using external XTAL
	nrf_clock_lfclk_t current_src = nrf_clock_lf_src_get(NRF_CLOCK);
	if (current_src == NRF_CLOCK_LFCLK_XTAL || current_src == NRF_CLOCK_LFCLK_XTAL_FULL_SWING) {
		LOG_INF("Already using external XTAL, no switch needed");
	} else {
		LOG_WRN("Switching LF clock from RC to external XTAL...");

		// CRITICAL: Disable interrupts during clock switch to prevent tick-dependent code
		unsigned int key = irq_lock();

		// Stop LF clock
		nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTOP);

		// Busy-wait for clock to stop (use CPU cycles, not system tick)
		// Typical stop time: < 1ms
		volatile uint32_t stop_timeout = 100000; // ~10ms @ 16MHz
		while (nrf_clock_lf_is_running(NRF_CLOCK) && stop_timeout > 0) {
			stop_timeout--;
		}

		if (stop_timeout == 0) {
			LOG_ERR("LF clock failed to stop!");
			irq_unlock(key);
		} else {
			// Set new clock source
			nrf_clock_lf_src_set(NRF_CLOCK, NRF_CLOCK_LFCLK_XTAL_FULL_SWING);

			// Start LF clock with new source
			nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);

			// Busy-wait for clock to start and stabilize
			// External XTAL startup: 250-500ms typical
			// Use longer timeout to be safe
			volatile uint32_t start_timeout = 10000000; // ~625ms @ 16MHz
			while (!nrf_clock_lf_is_running(NRF_CLOCK) && start_timeout > 0) {
				start_timeout--;
			}

			// Re-enable interrupts
			irq_unlock(key);

			if (start_timeout > 0) {
				LOG_INF("LF clock switch successful!");
				LOG_INF("New LF source: %d, running: %d",
					nrf_clock_lf_src_get(NRF_CLOCK),
					nrf_clock_lf_is_running(NRF_CLOCK));
			} else {
				LOG_ERR("LF clock failed to start! System may be unstable");
				LOG_ERR("This usually means no external XTAL is present on hardware");
			}
		}
	}
	LOG_INF("=== LF Clock Switch End ===");
#else
	LOG_INF("External XTAL not configured in prj.conf, using default LF clock");
#endif

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
		if (ping_failures >= TX_ERROR_THRESHOLD) {
#if CONFIG_CONNECTION_OVER_HID
			// only raise error while not potentially communicating by usb
			if (get_status(SYS_STATUS_CONNECTION_ERROR) == false && get_status(SYS_STATUS_USB_CONNECTED) == false)
#else
			if (get_status(SYS_STATUS_CONNECTION_ERROR) == false)
#endif
				set_status(SYS_STATUS_CONNECTION_ERROR, true);
#if USER_SHUTDOWN_ENABLED
			if (!shutdown_requested && connection_error_start_time > 0
				&& k_uptime_get() - connection_error_start_time
					   > CONFIG_CONNECTION_TIMEOUT_DELAY) // shutdown if receiver is not detected
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
			if (now_idle - remote_command_receive_time >= REMOTE_COMMAND_DELAY_MS) {
				switch (received_remote_command) {
				case ESB_PONG_FLAG_SHUTDOWN:
					LOG_WRN("Executing remote command: SHUTDOWN");
					sys_request_system_off(false);
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
#if DT_NODE_HAS_STATUS(DT_NODELABEL(mag), okay)
					LOG_INF("Executing remote command: MAG_CLEAR");
					sensor_calibration_clear_mag(NULL, true);
#else
					LOG_WRN("Remote command: MAG_CLEAR not supported (no magnetometer)");
#endif
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
					NRF_POWER->GPREGRET = 0x57;
					k_msleep(100);
#endif
					sys_request_system_reboot(false);
#else
					LOG_WRN("Remote command: DFU not supported (no bootloader)");
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

				case ESB_PONG_FLAG_PING:
					LOG_INF("Executing remote command: PING");
					cmd_ping_start();
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

		k_msleep(100);
	}
}
