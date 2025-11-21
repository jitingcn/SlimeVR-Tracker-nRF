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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/kernel.h>

#include "connection.h"
#include "globals.h"
#include "system/system.h"
#include "sensor/sensor.h"
#include "sensor/calibration.h"
#include "zephyr/logging/log.h"
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */
#include <zephyr/sys/crc.h>
#include <nrfx_timer.h>
#include <hal/nrf_timer.h>

#include "esb.h"

uint8_t last_reset = 0;
// const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);
bool esb_state = false;
bool timer_state = false;
bool send_data = false;
uint16_t led_clock = 0;
uint32_t led_clock_offset = 0;

int64_t connection_error_start_time = 0;
static bool shutdown_requested = false;
static bool pair_ack_pending
	= false;  // True once step 1 is sent and we expect a receiver response

static struct esb_payload rx_payload;
// Normal data payload (16+1 bytes when used), length set per write
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0);
static struct esb_payload tx_payload_pair
	= ESB_CREATE_PAYLOAD(0, 0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload ack_payload = ESB_CREATE_PAYLOAD(0);

static uint8_t paired_addr[8] = {0};

static bool esb_initialized = false;
static bool esb_paired = false;

#define TX_ERROR_THRESHOLD 100
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
K_THREAD_DEFINE(esb_thread_id, 512, esb_thread, NULL, NULL, NULL, 7, 0, 0);
static int64_t last_tx_time = 0;

static uint32_t ping_success_streak = 0;  // consecutive success counter
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
	uint32_t cycles;
};
static struct ping_history_entry ping_history[PING_HISTORY_SIZE] = {0};
static uint8_t ping_history_idx = 0;

static uint8_t received_remote_command = ESB_PONG_FLAG_NORMAL;
static uint8_t acked_remote_command = ESB_PONG_FLAG_NORMAL;
static int64_t remote_command_receive_time = 0;
static uint32_t received_channel_value = 0;  // Store channel value from PONG data[8-11]
#define REMOTE_COMMAND_DELAY_MS 3000

// Server time synchronization for TDMA scheduling (using cycles for μs precision)
static uint32_t last_server_time_cycles = 0;  // Server time in cycles
static bool server_time_synced = false;

// TDMA scheduling with hardware timer
// Target: 100 TPS per tracker for ALL non-PING packets (data, mag, quat, info, status)
// 100 TPS = 10ms interval between packets
// 10 trackers share each 5ms period with 1000μs slot per tracker
// Each packet transmission takes ~300μs radio time
#define TDMA_NUM_TRACKERS 10
#define TDMA_PACKETS_PER_SECOND 176 // Target TPS per tracker (all non-PING packets)
#define TDMA_PACKET_INTERVAL_US (1000000 / TDMA_PACKETS_PER_SECOND)
#define TDMA_SLOT_DURATION_US (TDMA_PACKET_INTERVAL_US / TDMA_NUM_TRACKERS)
#define TDMA_TX_TIME_US 200 // Actual radio transmission time per packet
#define TDMA_GUARD_TIME_US 170 // Guard time to wake up before slot starts
// 将 TDMA 参数转换为 Cycles (假设 64MHz, 1us = 64 cycles)
static uint32_t INTERVAL_CYCLES = k_us_to_cyc_ceil32(TDMA_PACKET_INTERVAL_US);
static uint64_t g_server_virtual_cycles = 0; // 虚拟的64位单调递增时间
static uint32_t g_last_rx_raw_cycles = 0;    // 上一次收到的原始32位时间
static uint32_t g_last_sync_local_cycles = 0; // 上一次同步时的本地时间
static bool     g_time_initialized = false;


static const nrfx_timer_t tdma_timer = NRFX_TIMER_INSTANCE(1);
static bool tdma_initialized = false;
static uint8_t tdma_tracker_id = 0;
static volatile bool tdma_in_tx_slot = false;
static uint32_t tdma_slot_counter = 0;
static K_SEM_DEFINE(tdma_slot_sem, 0, 1);

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
static const char *meow_suffixes[] = {
	" :3", " :3c", " ;3", " ;3c", " x3", " x3c", " X3", " X3c",
	" >:3", " >:3c", " >;3", " >;3c", ""
};

static void remote_print_meow(void) {
	int64_t ticks = k_uptime_ticks();
	ticks %= ARRAY_SIZE(meows) * ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes);
	uint8_t meow = ticks / (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	ticks %= (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	uint8_t punctuation = ticks / ARRAY_SIZE(meow_suffixes);
	uint8_t suffix = ticks % ARRAY_SIZE(meow_suffixes);
	LOG_INF("%s%s%s", meows[meow], meow_punctuations[punctuation], meow_suffixes[suffix]);
}

void esb_update_server_time(uint32_t server_raw_cycles, uint32_t rtt_cycles) {
	uint32_t now_local = k_cycle_get_32();

	if (!g_time_initialized) {
		// 第一次初始化
		g_server_virtual_cycles = server_raw_cycles;
		g_last_rx_raw_cycles = server_raw_cycles;
		g_time_initialized = true;
	} else {
		// 核心魔法：计算差值 (利用 int32_t 处理 32位回绕)
		// 即使 server_raw_cycles 溢出，diff 仍然是正确的正数增量
		int32_t diff = (int32_t)(server_raw_cycles - g_last_rx_raw_cycles);

		// 只有当 diff 为正值或极小的负值(乱序)时才更新，避免巨大跳变
		// 正常情况下 diff 应该是正的 (例如 1000ms 对应的 cycles)
		if (diff > 0) {
			g_server_virtual_cycles += diff;
			g_last_rx_raw_cycles = server_raw_cycles;
		}
	}

	// 补偿 RTT (RTT/2)
	// 注意：我们保存的是"不含RTT补偿的基准时间"，在获取时再算，或者这里直接加
	// 这里为了逻辑清晰，我们维护"收到包那一刻的服务器虚拟时间"
	// 修正：g_server_virtual_cycles 代表的是"服务器在发送这个包时的虚拟时间"

	// 记录同步时刻的本地时间
	g_last_sync_local_cycles = now_local;
	server_time_synced = true;
}

static uint8_t tracker_id = 0;
static void set_tracker_id(uint8_t id)
{
	tracker_id = id;
}

// --- esb_write() rate logging ---
static uint32_t esb_write_calls = 0;
static uint32_t esb_write_queued = 0;
static int64_t esb_rate_last_ts = 0;

void esb_write_rate_tick(void) {
    int64_t now = k_uptime_get();
    if (esb_rate_last_ts == 0) {
        esb_rate_last_ts = now;
    }
    esb_write_calls++;
    if (now - esb_rate_last_ts >= 3000) {
        LOG_INF("esb_write rate: calls=%u/s queued=%u/s", esb_write_calls / 3, esb_write_queued / 3);
        esb_write_calls = 0;
        esb_write_queued = 0;
        esb_rate_last_ts = now;
    }
}

// ESB recovery mechanism for persistent ENOMEM errors
static uint32_t consecutive_enomem_errors = 0;
static int64_t last_enomem_time = 0;
#define ENOMEM_ERROR_THRESHOLD 10  // Force recovery after N consecutive errors (increased from 5)
#define ENOMEM_ERROR_WINDOW_MS 2000  // Reset counter if no error for this duration (increased from 1s)

void esb_write_ack(uint8_t type) {
	// This small ACK packet is crucial for receiving the PONG ACK payload
	// ESB requires a transmission to trigger ACK payload reception

	// Check ESB TX FIFO before adding packet
	if (esb_tx_full()) {
		LOG_WRN("TX FIFO full when queuing ACK packet, skipping this ACK");
		// Don't try to recover here - let the main error handler deal with it
		// Forcing recovery for every ACK is too aggressive
		return;
	}

	// small packet with no data, just to get ack result
	ack_payload.pipe = 1 + (tracker_id % 7);
	ack_payload.noack = false;
	ack_payload.length = 1;
	ack_payload.data[0] = 0x00;  // Empty payload marker

	// Record ack_payload for diagnostics
	last_tx.type = type;  // ack for last sent packet type
	last_tx.is_ack_payload = true;
	last_tx.noack = false;
	last_tx.length = 1;
	last_tx.timestamp = k_uptime_get();

	int ack_status = esb_write_payload(&ack_payload);
	if (ack_status != 0) {
		const char* err_str = "unknown";
		if (ack_status == -ENOMEM) err_str = "ENOMEM (ESB not ready)";
		else if (ack_status == -ENOSPC) err_str = "ENOSPC (FIFO full)";

		LOG_DBG(
			"esb_write_ack: failed to queue ACK packet (err=%d %s)",
			ack_status,
			err_str
		);
		// Try flushing and retrying once
		if (ack_status == -ENOSPC || ack_status == -ENOMEM) {
			esb_flush_tx();
			k_msleep(2);
			ack_status = esb_write_payload(&ack_payload);
			if (ack_status == 0) {
				LOG_DBG("ACK packet queued successfully after retry");
			}
		}
	} else {
		LOG_DBG("ACK packet queued to trigger PONG reception (type=0x%02X)", type);
	}
}

void event_handler(struct esb_evt const* event) {
	static uint32_t tx_success_count = 0;
	static uint32_t tx_failed_count = 0;
	static uint32_t last_log_time = 0;

	switch (event->evt_id) {
		case ESB_EVENT_TX_SUCCESS:
			tx_success_count++;
			// Reset ENOMEM error counter on successful transmission
			consecutive_enomem_errors = 0;
			if (esb_paired) {
				clocks_stop();
			}
			break;
		case ESB_EVENT_TX_FAILED:
			esb_pop_tx();
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
			} else if (last_tx.type == 0xFF) {
				pkt_desc = "PING_ACK_RETRY";
			} else {
				pkt_desc = "OTHER";
			}

			// Critical: if ACK packet (used to retrieve PONG) failed
			if (last_tx.is_ack_payload && last_tx.type == ESB_PING_TYPE) {
				LOG_DBG("ACK packet (for PONG) failed");
				esb_write_ack(0xFF);	// Mark as retry
			} else if (last_tx.is_ack_payload && last_tx.type == 0xFF) {
				// Retry also failed
				LOG_DBG("ACK packet retry also failed");
			} else {
				LOG_DBG("TX FAILED: type=%s(0x%02X) len=%u noack=%d age=%lldms attempts=%u",
					pkt_desc, last_tx.type, last_tx.length, last_tx.noack,
					k_uptime_get() - last_tx.timestamp, event->tx_attempts);
			}

			// Log TX statistics every 20 failures for debugging
			uint32_t now = k_uptime_get_32();
			if (tx_failed_count % 20 == 0 || (now - last_log_time > get_ping_interval_ms())) {
				last_log_time = now;
				uint32_t total = tx_success_count + tx_failed_count;
				uint32_t fail_rate = total > 0 ? (tx_failed_count * 100 / total) : 0;
				LOG_WRN("TX Stats: success=%u failed=%u rate=%u%%",
					tx_success_count, tx_failed_count, fail_rate);
			}

			// Only count ping failures for connection timeout
			if (ping_pending && k_uptime_get() - ping_send_time > (get_ping_interval_ms() - 100)) {
				ping_failed = true;
				ping_pending = false;  // Clear the pending flag
				ping_success_streak = 0;  // Reset recovery streak on any failure
				ping_failures++;
			}

			if (ping_failures > 0 && ping_failures % 10 == 0 && last_tx.type == ESB_PING_TYPE)  // Log every 10 failures
			{
				LOG_WRN("Ping failed, total failures: %d", ping_failures);
			}
			if (ping_failures == TX_ERROR_THRESHOLD)  // consecutive ping failures
			{
				connection_error_start_time
					= k_uptime_get();  // Mark when connection errors started
				LOG_WRN(
					"Ping failure threshold reached (%d failures), starting "
					"timeout timer",
					TX_ERROR_THRESHOLD
				);
			}

			if (esb_paired) {
				clocks_stop();
			}
			break;
		case ESB_EVENT_RX_RECEIVED:
			int err = 0;
			err = esb_read_rx_payload(&rx_payload);
			if (err == -ENODATA)
			{
				return;
			}
			else if (err)
			{
				LOG_ERR("Error while reading rx packet: %d", err);
				return;
			}
			// Hex dump first up to 13 bytes for visibility
			char hexbuf[3 * 13 + 1] = {0};
			int dump_len = rx_payload.length < 13 ? rx_payload.length : 13;
			for (int i = 0; i < dump_len; i++) {
				snprintk(&hexbuf[i * 3], 4, "%02X ", rx_payload.data[i]);
			}
			LOG_DBG(
				"RX len=%u pipe=%u data=%s",
				rx_payload.length,
				rx_payload.pipe,
				hexbuf
			);
			LOG_DBG(
				"RX payload len=%u type=%u pipe=%u",
				rx_payload.length,
				rx_payload.data[0],
				rx_payload.pipe
			);
			if (!paired_addr[0])  // zero, not paired
			{
				LOG_DBG(
					"tx: %16llX rx: %16llX",
					*(uint64_t*)tx_payload_pair.data,
					*(uint64_t*)rx_payload.data
				);
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
					uint64_t local_addr
						= (*(uint64_t*)NRF_FICR->DEVICEADDR) & 0xFFFFFFFFFFFFULL;
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
					case 4:
					{
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
						led_clock = (rx_payload.data[0] << 8)
									+ rx_payload.data[1];  // sync led flashes :)
						led_clock_offset = 0;
						LOG_DBG("RX, timer reset");
						pair_ack_pending = false;
					}	break;
					case ESB_PONG_LEN:
					{
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
								LOG_WRN(
									"Received PONG for tracker ID %u (local ID %u)",
									rx_id,
									tracker_id
								);
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
							// Check counter with relaxed tolerance due to ACK payload delay
							// ACK payload is only sent with the NEXT packet, so we expect
							// PONG(N) when sending PING(N+1), causing a natural 1-count lag
							// Allow up to 5 counts difference to handle packet loss scenarios
							int counter_diff = (int)ping_counter - (int)rx_ctr;
							if (counter_diff < 0) counter_diff += 256;  // Handle wrap-around

							bool match_ctr = (counter_diff >= 0 && counter_diff <= 5);
							if (!match_ctr) {
								LOG_WRN(
									"unsynced counter %u (expected ~%u, diff=%d)",
									rx_ctr,
									ping_counter,
									counter_diff
								);
								// Don't break - still process the PONG to maintain connection
								// Just log the warning for debugging
							}							uint32_t ping_send_time_32 = ((uint32_t)rx_payload.data[3] << 24)
															| ((uint32_t)rx_payload.data[4] << 16)
															| ((uint32_t)rx_payload.data[5] << 8)
															| ((uint32_t)rx_payload.data[6]);

								uint32_t now32 = (uint32_t)k_uptime_get();
								uint32_t now_cyc = k_cycle_get_32();
								uint32_t rtt = (now32 - ping_send_time_32) / 2;

								// Find send cycles for this PONG's counter in history
								uint32_t ping_cycles_for_this_ctr = 0;
								for (int i = 0; i < PING_HISTORY_SIZE; i++) {
									if (ping_history[i].counter == rx_ctr && ping_history[i].cycles != 0) {
										ping_cycles_for_this_ctr = ping_history[i].cycles;
										break;
									}
								}

								uint32_t rtt_us = 0;
								bool rtt_valid = false;

								if (ping_cycles_for_this_ctr != 0) {
									// Calculate RTT using the correct send time for this counter
									uint32_t cyc_diff = now_cyc - ping_cycles_for_this_ctr;
									rtt_us = k_cyc_to_us_near32(cyc_diff) / 2;
									rtt_valid = true;
								} else {
									// No history found - likely too old or buffer wrapped
									// Use timestamp-based RTT converted to cycles for estimation
									rtt_us = rtt * 1000;  // ms to us
									rtt_valid = false;
								}

								// Check flags field (byte 7)
								uint8_t pong_flags = rx_payload.data[7];

								// Only parse server time if this is a normal heartbeat (flags == 0)
								if (pong_flags == ESB_PONG_FLAG_NORMAL) {
									// Capture receiver cycle timestamp from PONG param (bytes 8..11)
									uint32_t pong_tx_cycles = ((uint32_t)rx_payload.data[8] << 24)
															| ((uint32_t)rx_payload.data[9] << 16)
															| ((uint32_t)rx_payload.data[10] << 8)
															| ((uint32_t)rx_payload.data[11]);

									// log ping to pong rtt and pong to now rtt
									LOG_INF("PONG RTT: %u ms", rtt);

									// Calculate server cycles compensated with RTT (in cycles)
									uint32_t rtt_cycles = rtt * sys_clock_hw_cycles_per_sec() / 1000;
									uint32_t server_cycles = pong_tx_cycles + (rtt_cycles / 2);

									// Store server time for TDMA scheduling
									last_server_time_cycles = server_cycles;
									esb_update_server_time(server_cycles, rtt_cycles);

									// Convert to ms for human-readable display
									uint32_t server_time_ms = k_cyc_to_ms_near32(server_cycles);
									uint32_t server_ms = server_time_ms % 1000;
									uint32_t server_s = (server_time_ms / 1000) % 60;
									uint32_t server_m = (server_time_ms / 60000) % 60;
									uint32_t server_h = (server_time_ms / 3600000) % 24;
									LOG_INF(
										"Server time: %02u:%02u:%02u.%03u (cycles=%u)",
										server_h,
										server_m,
										server_s,
										server_ms,
										server_cycles
									);
								} else {
									// Command packet, log RTT only
									LOG_INF("PONG RTT: %u ms (command packet)", rtt);
								}

							// handle remote commands and delayed execution
							if (pong_flags != ESB_PONG_FLAG_NORMAL) {
								if (received_remote_command == ESB_PONG_FLAG_NORMAL) {
									// new command received
									received_remote_command = pong_flags;
									remote_command_receive_time = k_uptime_get();

									// For SET_CHANNEL command, extract channel value from data[8-11]
									if (pong_flags == ESB_PONG_FLAG_SET_CHANNEL) {
										received_channel_value = ((uint32_t)rx_payload.data[8] << 24)
																| ((uint32_t)rx_payload.data[9] << 16)
																| ((uint32_t)rx_payload.data[10] << 8)
																| ((uint32_t)rx_payload.data[11]);
									}

									const char* cmd_name = "UNKNOWN";
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
									}
									if (pong_flags == ESB_PONG_FLAG_SET_CHANNEL) {
										LOG_INF("Remote command %s (0x%02X) received, channel=%u, will execute in %dms",
											cmd_name, pong_flags, received_channel_value, REMOTE_COMMAND_DELAY_MS);
									} else {
										LOG_INF("Remote command %s (0x%02X) received, will execute in %dms",
											cmd_name, pong_flags, REMOTE_COMMAND_DELAY_MS);
									}
								}
							} else {
								// received NORMAL flag, indicates the receiver has confirmed our echo
								if (acked_remote_command != ESB_PONG_FLAG_NORMAL) {
									LOG_DBG("Receiver confirmed command 0x%02X, resetting state",
										acked_remote_command);
									received_remote_command = ESB_PONG_FLAG_NORMAL;
									acked_remote_command = ESB_PONG_FLAG_NORMAL;
									remote_command_receive_time = 0;
								}
							}

							// set ping valid
							ping_pending = false;
							ping_failed = false;
							ping_failures = 0;
							if (get_status(SYS_STATUS_CONNECTION_ERROR) == true) {
								ping_success_streak++;
								if (ping_success_streak >= PING_RECOVERY_THRESHOLD) {
									set_status(SYS_STATUS_CONNECTION_ERROR, false);
									connection_error_start_time = 0;
									shutdown_requested = false;

									if (rtt_us < 1000) {
										LOG_INF(
											"PONG ok - link restored after %u probes, "
											"rtt=%u us by cycles",
											ping_success_streak,
											(unsigned)rtt_us
										);
									} else {
										LOG_INF(
											"PONG ok - link restored after %u probes, "
											"rtt=%u.%03u ms by cycles",
											ping_success_streak,
											(unsigned)(rtt_us / 1000),
											(unsigned)(rtt_us % 1000)
										);
									}
									ping_success_streak = 0;
								}
							} else {
								ping_success_streak = 0;
								if (rtt_us < 1000) {
									LOG_INF(
										"PONG ok, rtt=%u us (ctr=%u)%s",
										(unsigned)rtt_us,
										rx_ctr,
										rtt_valid ? "" : " [estimated]"
									);
								} else {
									LOG_INF(
										"PONG ok, rtt=%u.%03u ms (ctr=%u)%s",
										(unsigned)(rtt_us / 1000),
										(unsigned)(rtt_us % 1000),
										rx_ctr,
										rtt_valid ? "" : " [estimated]"
									);
								}
							}
						}
					} break;
					default:
						LOG_WRN(
							"Ignoring invalid payload length %u",
							rx_payload.length
						);
				}
			} break;
	}
}

bool clock_status = false;

#if defined(CONFIG_CLOCK_CONTROL_NRF)
static struct onoff_manager* clk_mgr;

static int clocks_init(void) {
	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENOTSUP;
	}

	return 0;
}

SYS_INIT(clocks_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

int clocks_start(void) {
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

void clocks_stop(void) {
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

void clocks_request_start(uint32_t delay_us) {
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

void clocks_request_stop(uint32_t delay_us) {
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
static const uint8_t discovery_addr_prefix[8]
	= {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

int esb_initialize(bool tx) {
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
		config.retransmit_count = CONNECTION_ENABLE_ACK ? 1 : 2;
		// config.tx_mode = ESB_TXMODE_AUTO;
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

void esb_deinitialize(void) {
	if (esb_initialized) {
		esb_initialized = false;
		k_msleep(10);  // wait for pending transmissions
		esb_disable();
	}
	esb_initialized = false;
}

inline void esb_set_addr_discovery(void) {
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

inline void esb_set_addr_paired(void) {
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
			|| addr_buffer[i] == 0xAA) {  // Avoid invalid addresses (see nrf datasheet)
			addr_buffer[i] += 8;
		}
	}
	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
}

static int esb_send_pair_step(uint8_t step) {
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

void esb_set_pair(uint64_t addr) {
	uint64_t* device_addr
		= (uint64_t*)NRF_FICR
			  ->DEVICEADDR;  // Use device address as unique identifier (although it is
							 // not actually guaranteed, see datasheet)
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
	sys_write(
		PAIRED_ID,
		retained->paired_addr,
		paired_addr,
		sizeof(paired_addr)
	);  // Write new address and tracker id
}

void esb_pair(void) {
	// Reset ping state when starting pairing
	ping_failures = 0;
	set_status(SYS_STATUS_CONNECTION_ERROR, false);
	connection_error_start_time = 0;
	shutdown_requested = false;
	ping_failed = false;
	ping_pending = false;
	if (!paired_addr[0])  // zero, no receiver paired
	{
		LOG_INF("Pairing");
		esb_set_addr_discovery();
		esb_initialize(true);
		//		timer_init(); // TODO: shouldn't be here!!!
		tx_payload_pair.noack = false;
		uint64_t* addr
			= (uint64_t*)NRF_FICR
				  ->DEVICEADDR;  // Use device address as unique identifier (although it
								 // is not actually guaranteed, see datasheet)
		memcpy(&tx_payload_pair.data[2], addr, 6);
		LOG_INF("Device address: %012llX", *addr & 0xFFFFFFFFFFFF);
		uint8_t checksum = crc8_ccitt(0x07, &tx_payload_pair.data[2], 6);
		if (checksum == 0) {
			checksum = 8;
		}
		LOG_INF("Checksum: %02X", checksum);
		tx_payload_pair.data[0]
			= checksum;  // Use checksum to make sure packet is for this device
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
			if (!shutdown_requested
				&& (k_uptime_get() - pair_start_time)
					   > CONFIG_CONNECTION_TIMEOUT_DELAY) {
				LOG_WRN(
					"Pairing timeout after %dm",
					CONFIG_CONNECTION_TIMEOUT_DELAY / 60000
				);
				shutdown_requested = true;
				sys_request_system_off(false);
			}
#endif
			if (paired_addr[0]) {
				LOG_INF("Incorrect checksum: %02X", paired_addr[0]);
				paired_addr[0] = 0;  // Packet not for this device
			}
			esb_flush_rx();
			esb_flush_tx();
			pair_ack_pending = false;  // Reset before sending
			if (esb_send_pair_step(0)) {
				k_msleep(100);
				continue;
			}
			k_msleep(2);
			pair_ack_pending
				= true;  // Set before step 1 which expects receiver response
			if (esb_send_pair_step(1)) {
				pair_ack_pending = false;
				k_msleep(100);
				continue;
			}
			k_msleep(2);
			esb_send_pair_step(2);  // "acknowledge" pairing from receiver
			k_msleep(996);
		}
		set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_CONNECTION);
		LOG_INF("Paired");
		sys_write(
			PAIRED_ID,
			retained->paired_addr,
			paired_addr,
			sizeof(paired_addr)
		);  // Write new address and tracker id
		esb_deinitialize();
		k_msleep(1600);  // wait for led pattern
	}
	LOG_INF("Tracker ID: %u", paired_addr[1]);
	LOG_INF(
		"Receiver address: %012llX",
		(*(uint64_t*)&retained->paired_addr[0] >> 16) & 0xFFFFFFFFFFFF
	);

	connection_set_id(paired_addr[1]);
	set_tracker_id(paired_addr[1]);

	esb_set_addr_paired();
	esb_paired = true;
	clocks_stop();
}

void esb_reset_pair(void) {
	if (paired_addr[0] || esb_paired) {
		esb_deinitialize();  // make sure esb is off
		esb_paired = false;
		memset(paired_addr, 0, sizeof(paired_addr));
		LOG_INF("Pairing requested");
	}
}

void esb_clear_pair(void) {
	esb_reset_pair();
	sys_write(
		PAIRED_ID,
		&retained->paired_addr,
		paired_addr,
		sizeof(paired_addr)
	);  // write zeroes
	LOG_INF("Pairing data reset");
}

void esb_write(uint8_t* data, bool no_ack, size_t data_length) {
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
		// Set sequence number
		data[2] = ping_counter;
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
	int queue_status = esb_write_payload(&tx_payload);

	// if sending ping packet, we need send another small packet to get ack result asap
	if (data[0] == ESB_PING_TYPE && queue_status == 0 && data_length == ESB_PING_LEN) {
		k_usleep(300);
		// uint32_t now32 = (uint32_t)now;
		ping_pending = true;
		ping_ctr_sent = tx_payload.data[2];
		ping_send_time = k_uptime_get();

		// Record cycles for THIS counter in circular buffer (for accurate RTT calculation)
		uint8_t this_ctr = tx_payload.data[2];
		ping_history[ping_history_idx].counter = this_ctr;
		ping_history[ping_history_idx].cycles = k_cycle_get_32();
		ping_history_idx = (ping_history_idx + 1) % PING_HISTORY_SIZE;

		last_tx_time = k_uptime_get();
		LOG_INF(
			"PING sent (ctr=%u)",
			(unsigned)tx_payload.data[2]
		);

		esb_write_ack(ESB_PING_TYPE);
	} else if (tx_payload.data[0] == ESB_PING_TYPE && queue_status != 0) {
		// PING failed to queue - this is critical!
		const char* err_str = "unknown";
		if (queue_status == -ENOMEM) err_str = "ENOMEM (ESB not ready)";
		else if (queue_status == -ENOSPC) err_str = "ENOSPC (FIFO full)";
		else if (queue_status == -EACCES) err_str = "EACCES (access denied)";
		else if (queue_status == -ENODATA) err_str = "ENODATA (no data available)";

		// Only log if this is the first failure or every 10th failure
		if (consecutive_enomem_errors == 1 || consecutive_enomem_errors % 10 == 0) {
			LOG_ERR("esb_write: PING failed to queue (ctr=%u, err=%d %s, consecutive=%u)",
					tx_payload.data[2], queue_status, err_str, consecutive_enomem_errors);
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
			consecutive_enomem_errors = 0;  // Reset after successful flush
		} else if (flush_result == -EBUSY) {
			// ESB is busy transmitting, this is normal - just wait
			if (consecutive_enomem_errors >= ENOMEM_ERROR_THRESHOLD) {
				// Only log warning if we've hit threshold
				LOG_WRN(
					"ESB TX FIFO full for %u consecutive attempts (ESB busy)",
					consecutive_enomem_errors
				);

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
							k_msleep(10);
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
		LOG_ERR(
			"esb_write: failed to queue packet, err=%d (logged every 10 errors)",
			queue_status
		);
	}

	// Record last TX time for idle probe scheduling
	if (queue_status == 0) {
		last_tx_time = k_uptime_get();
		esb_write_queued++;
	}

	// esb_start_tx();
	send_data = true;
}

bool esb_ready(void) { return esb_initialized && esb_paired; }

uint8_t esb_get_ping_ack_flag(void) {
	if (acked_remote_command != ESB_PONG_FLAG_NORMAL) {
		return acked_remote_command;
	}
	if (received_remote_command != ESB_PONG_FLAG_NORMAL) {
		return received_remote_command;
	}
	return ESB_PONG_FLAG_NORMAL;
}


uint64_t esb_get_server_time_cycles_64(void) {
	if (!server_time_synced) {
		return 0;
	}

	uint32_t current_local = k_cycle_get_32();
	// 计算自上次同步以来经过的本地时间
	// 这里也要处理本地时钟的回绕
	uint32_t elapsed = current_local - g_last_sync_local_cycles;

	return g_server_virtual_cycles + elapsed;
}

uint64_t esb_get_server_time_us_64(void) {
		uint64_t cycles = esb_get_server_time_cycles_64(); // 调用之前定义的64位Cycle函数

		// 手动转换以支持 64 位，防止 k_cyc_to_us_near32 截断数据
		// 公式: (cycles * 1000000) / frequency
		return (cycles * 1000000ULL) / sys_clock_hw_cycles_per_sec();
}

// Legacy function for backward compatibility (returns milliseconds)
uint32_t esb_get_server_time(void) {
	uint64_t cycles = esb_get_server_time_cycles_64();
	if (cycles == 0) {
		return 0;
	}
	uint64_t time_us = esb_get_server_time_us_64();
	return (uint32_t)(time_us / 1000ULL);
}

// TDMA timer interrupt handler
// Note: This timer-based approach is simplified. The actual slot checking
// is done in tdma_wait_for_slot() for better precision.
static void tdma_timer_handler(nrf_timer_event_t event_type, void *p_context) {
	// Timer is used for time capture in tdma_wait_for_slot()
	// No interrupt-based slot detection needed
}

void tdma_init(uint8_t tracker_id) {
	if (tdma_initialized) {
		return;
	}

	tdma_tracker_id = tracker_id;

	// Configure timer at 1MHz (1μs resolution)
	nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(1000000);
	timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
	timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
	timer_cfg.mode = NRF_TIMER_MODE_TIMER;

	nrfx_err_t err = nrfx_timer_init(&tdma_timer, &timer_cfg, tdma_timer_handler);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("TDMA timer init failed: %d", err);
		return;
	}

	// No interrupt needed, just enable timer for time capture
	nrfx_timer_enable(&tdma_timer);
	tdma_initialized = true;

	LOG_INF("TDMA initialized: tracker_id=%d, slot=%dus, interval=%dus, slot_window=%dus",
		tracker_id,
		(tracker_id % TDMA_NUM_TRACKERS) * TDMA_SLOT_DURATION_US,
		TDMA_PACKET_INTERVAL_US,
		TDMA_TX_TIME_US + 100);
}

void tdma_deinit(void) {
	if (!tdma_initialized) {
		return;
	}

	nrfx_timer_disable(&tdma_timer);
	nrfx_timer_uninit(&tdma_timer);
	tdma_initialized = false;
	LOG_INF("TDMA deinitialized");
}

bool tdma_is_in_tx_slot(void) {
	if (!server_time_synced) return false;

	// 获取 64 位时间
	uint64_t server_cycles_64 = esb_get_server_time_cycles_64();

	// 核心修复：在 64位 域进行取模，消除了 32位 溢出导致的相位跳变
	// 只要 INTERVAL_CYCLES 不是极大的数，结果就是准确的
	uint32_t pos_in_interval = server_cycles_64 % INTERVAL_CYCLES;

	uint32_t slot_start_cycles = k_us_to_cyc_ceil32((tdma_tracker_id % TDMA_NUM_TRACKERS) * TDMA_SLOT_DURATION_US);

	int64_t diff = (int64_t)pos_in_interval - (int64_t)slot_start_cycles;

	// 处理循环边界 (例如我在 0ms, 当前是 9.9ms)
	if (diff < -((int64_t)INTERVAL_CYCLES / 2)) diff += INTERVAL_CYCLES;
	if (diff > ((int64_t)INTERVAL_CYCLES / 2)) diff -= INTERVAL_CYCLES;

	// 转换回微秒进行窗口判断 (或者直接用 cycles 判断更准)
	int32_t diff_us = k_cyc_to_us_near32((uint32_t)llabs(diff));
	if (diff < 0) diff_us = -diff_us;

	return (diff_us >= -100 && diff_us < (TDMA_TX_TIME_US + 20));
}

bool tdma_is_synced(void) {
	return server_time_synced && tdma_initialized;
}

// Get current slot number (0 to max) based on server time
// Used to prevent sending multiple packets in the same slot
uint32_t tdma_get_current_slot(void) {
	if (!server_time_synced) {
		return 0;
	}

	// 【关键修改】使用 64 位微秒时间
	uint64_t server_time_us = esb_get_server_time_us_64();

	// Calculate which packet interval we're in
	// 注意：这里 interval_count 会非常大，但只要 server_time_us 是连续的，
	// interval_count 就是连续的，不会发生跳变。
	// 强转为 uint32_t 会发生自然的溢出回绕，但这对于"相等性判断"是安全的。
	uint32_t interval_count = (uint32_t)(server_time_us / TDMA_PACKET_INTERVAL_US);

	// Calculate which slot within the interval
	uint32_t pos_in_interval = (uint32_t)(server_time_us % TDMA_PACKET_INTERVAL_US);
	uint32_t slot_in_interval = pos_in_interval / TDMA_SLOT_DURATION_US;

	// Combine to get unique slot identifier
	return (interval_count * TDMA_NUM_TRACKERS) + slot_in_interval;
}

// Check if we're in PING guard zone (±2ms around any tracker's PING slot)
bool tdma_is_in_ping_guard_zone(uint8_t tracker_id) {
	if (!server_time_synced) {
		return false;
	}

	uint32_t server_time = esb_get_server_time();
	uint32_t current_ms_in_second = server_time % 1000;

	// Check all 10 tracker PING slots with smart allocation:
	// T0-T6: 0, 100, 200, 300, 400, 500, 600ms
	uint32_t ping_slots[10];
	for (int i = 0; i < 10; i++) {
		ping_slots[i] = i * 100;
	}

	for (int i = 0; i < 10; i++) {
		uint32_t ping_slot_ms = ping_slots[i];

		// Calculate shortest distance to this PING slot (handle wrap-around)
		int32_t diff = (int32_t)current_ms_in_second - (int32_t)ping_slot_ms;

		// Normalize to -500 to +500 range
		if (diff > 500) {
			diff -= 1000;
		} else if (diff < -500) {
			diff += 1000;
		}

		int32_t abs_diff = (diff < 0) ? -diff : diff;

		// Guard zone: ±2ms around PING slot
		// PING packets need ACK, which takes additional time
		if (abs_diff <= 2) {
			return true;
		}
	}	return false;
}

// Calculate microseconds until next transmission slot
uint32_t tdma_get_next_slot_us(void) {
	if (!server_time_synced) {
		return 0;
	}

	uint64_t server_time_us = esb_get_server_time_us_64();

	// Our slot offset within each packet interval
	uint32_t our_slot_offset_us
		= (tdma_tracker_id % TDMA_NUM_TRACKERS) * TDMA_SLOT_DURATION_US;

	// Current position within the packet interval
	uint32_t pos_in_interval = (uint32_t)(server_time_us % TDMA_PACKET_INTERVAL_US);

	// Calculate time until our slot
	uint32_t time_until_slot;
	if (pos_in_interval <= our_slot_offset_us) {
		// Our slot is ahead in this interval
		time_until_slot = our_slot_offset_us - pos_in_interval;
	} else {
		// Our slot is in the next interval
		time_until_slot
			= (TDMA_PACKET_INTERVAL_US - pos_in_interval) + our_slot_offset_us;
	}

	return time_until_slot;
}

// Wait for next transmission slot (only if not already in slot)
void tdma_wait_for_slot(void) {
	if (!tdma_is_synced()) {
		return;
	}

	// If we're already in our slot, don't wait
	if (tdma_is_in_tx_slot()) {
		return;
	}

	uint32_t wait_us = tdma_get_next_slot_us();

	// Only wait if the next slot is reasonably soon (within 1 packet interval)
	// Otherwise just skip and try next time
	if (wait_us > TDMA_PACKET_INTERVAL_US) {
		return; // Too far away, skip this cycle
	}

	// Apply guard time - wake up slightly before slot
	if (wait_us > TDMA_GUARD_TIME_US) {
		wait_us -= TDMA_GUARD_TIME_US;
	}

	// For waits longer than 1ms, use sleep
	if (wait_us > 1000) {
		k_usleep(wait_us);
	} else {
		// For very short waits (<100us)
		k_busy_wait(wait_us);
	}
}
static void esb_thread(void) {
#if CONFIG_CONNECTION_OVER_HID
	int64_t start_time = k_uptime_get();
#endif

	// Read paired address from retained
	memcpy(paired_addr, retained->paired_addr, sizeof(paired_addr));

	while (1) {
#if CONFIG_CONNECTION_OVER_HID
		if (!esb_paired && get_status(SYS_STATUS_USB_CONNECTED) == false
			&& k_uptime_get() - 750
				   > start_time)  // only automatically enter pairing while not
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
			if (get_status(SYS_STATUS_CONNECTION_ERROR) == false
				&& get_status(SYS_STATUS_USB_CONNECTED)
					   == false)  // only raise error while not potentially
								  // communicating by usb
#else
			if (get_status(SYS_STATUS_CONNECTION_ERROR) == false)
#endif
				set_status(SYS_STATUS_CONNECTION_ERROR, true);
#if USER_SHUTDOWN_ENABLED
			if (!shutdown_requested && connection_error_start_time > 0
				&& k_uptime_get() - connection_error_start_time
					   > CONFIG_CONNECTION_TIMEOUT_DELAY)  // shutdown if receiver is
														   // not detected
			{
				LOG_WRN(
					"No response from receiver in %dm",
					CONFIG_CONNECTION_TIMEOUT_DELAY / 60000
				);
				shutdown_requested = true;
				sys_request_system_off(false);
			}
#endif
		}
		// Check PING timeout and schedule idle PINGs
		int64_t now_idle = k_uptime_get();

		// 检查是否需要延迟执行远程命令
		if (received_remote_command != ESB_PONG_FLAG_NORMAL
			&& received_remote_command != acked_remote_command
			&& remote_command_receive_time > 0) {
			if (now_idle - remote_command_receive_time >= REMOTE_COMMAND_DELAY_MS) {
				// 延迟时间已到，执行命令
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
						NRF_POWER->GPREGRET = 0x57; // DFU_MAGIC_UF2_RESET
						k_msleep(100); // Wait for register to be written
#endif
						sys_request_system_reboot(false);
#else
						LOG_WRN("Remote command: DFU not supported (no bootloader)");
#endif
						break;

					case ESB_PONG_FLAG_SET_CHANNEL:
					{
						// Validate channel value (0-100)
						if (received_channel_value <= 100) {
							LOG_INF("Executing remote command: SET_CHANNEL to %u", received_channel_value);
							// Save to retained memory
							retained->rf_channel = (uint8_t)received_channel_value;
							retained_update();
							// Save to NVS
							sys_write(RF_CHANNEL_ID, &retained->rf_channel, &retained->rf_channel, sizeof(retained->rf_channel));
							LOG_INF("RF channel saved to NVS: %u", retained->rf_channel);
							// Reinitialize ESB with new channel
							esb_deinitialize();
							k_msleep(10);
							esb_initialize(true);  // Channel will be applied inside esb_initialize
							LOG_INF("ESB reinitialized with channel %u", retained->rf_channel);
						} else {
							LOG_ERR("Invalid channel value: %u (must be 0-100)", received_channel_value);
						}
					}
						break;

					case ESB_PONG_FLAG_CLEAR_CHANNEL:
						LOG_INF("Executing remote command: CLEAR_CHANNEL (restore default)");
						// Clear saved channel (set to 0xFF = use default)
						retained->rf_channel = 0xFF;
						retained_update();
						sys_write(RF_CHANNEL_ID, &retained->rf_channel, &retained->rf_channel, sizeof(retained->rf_channel));
						LOG_INF("RF channel cleared, will use default on next boot");
						// Reinitialize ESB with default channel
						esb_deinitialize();
						k_msleep(10);
						esb_initialize(true);  // Will use default channel since rf_channel is 0xFF
						LOG_INF("ESB reinitialized with default channel %u", RADIO_RF_CHANNEL);
						break;

					default:
						LOG_WRN("Unknown remote command: 0x%02X", received_remote_command);
						break;
				}

				// 标记命令已执行
				acked_remote_command = received_remote_command;

				// 对于shutdown命令，不再继续
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
