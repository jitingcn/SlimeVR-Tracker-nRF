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
#include "zephyr/logging/log.h"
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */
#include <zephyr/sys/crc.h>

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

static uint8_t paired_addr[8] = {0};

static bool esb_initialized = false;
static bool esb_paired = false;

#define TX_ERROR_THRESHOLD 30
#define RADIO_RETRANSMIT_DELAY CONFIG_RADIO_RETRANSMIT_DELAY
#define RADIO_RF_CHANNEL CONFIG_RADIO_RF_CHANNEL

// Require N consecutive successful ACK probes before clearing connection error
#ifndef PING_RECOVERY_THRESHOLD
#define PING_RECOVERY_THRESHOLD 3
#endif
// Allow longer wait before declaring PING timeout
#ifndef PING_TIMEOUT_MS
#define PING_TIMEOUT_MS (2 * PING_INTERVAL_MS)
#endif

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 512, esb_thread, NULL, NULL, NULL, 6, 0, 0);
static K_MUTEX_DEFINE(esb_tx_mutex);
static int64_t last_tx_time = 0;

static uint32_t ping_success_streak = 0;  // consecutive success counter
static bool ping_pending = false;
static bool ping_failed = false;

static uint32_t ping_failures = 0;
static uint32_t ping_ctr_sent = 0;
static uint8_t ping_counter = 0;
static int64_t ping_send_time = 0;
static uint32_t ping_send_cycles = 0;

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

void event_handler(struct esb_evt const* event) {
	switch (event->evt_id) {
		case ESB_EVENT_TX_SUCCESS:
			if (esb_paired) {
				clocks_stop();
			}
			break;
		case ESB_EVENT_TX_FAILED:
			// Only count ping failures for connection timeout
			if (ping_pending) {
				ping_failed = true;
				ping_pending = false;  // Clear the pending flag
				ping_success_streak = 0;  // Reset recovery streak on any failure
				ping_failures++;
				if (ping_failures % 10 == 0)  // Log every 10 failures
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
			}
			LOG_DBG("TX FAILED");
			if (esb_paired) {
				clocks_stop();
			}
			// esb_pop_tx();
			break;
		case ESB_EVENT_RX_RECEIVED:
			if (!esb_read_rx_payload(&rx_payload))  // zero, rx success
			{
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
				LOG_INF(
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
									LOG_WRN(
										"Ignoring PONG for different tracker ID %u (expected %u)",
										rx_id,
										tracker_id
									);
									break;
								}

								uint8_t rx_ctr = rx_payload.data[2];
								// Reset ping counter if unsynced
								bool match_ctr = abs(rx_ctr - ping_counter) <= 3 ||
												 abs(rx_ctr + 256 - ping_counter) <= 3 ||
												 abs(rx_ctr - 256 - ping_counter) <= 3;
								if (!match_ctr) {
									LOG_WRN(
										"unsynced counter %u (expected ~%u)",
										rx_ctr,
										ping_counter
									);
									break;
								}
								// calcul ping send time
								uint32_t ping_send_time = ((uint32_t)rx_payload.data[3] << 24)
														 | ((uint32_t)rx_payload.data[4] << 16)
														 | ((uint32_t)rx_payload.data[5] << 8)
														 | ((uint32_t)rx_payload.data[6]);
								uint32_t now32 = (uint32_t)k_uptime_get();
								uint32_t now_cyc = k_cycle_get_32();
								uint32_t rtt = (now32 - ping_send_time);
								uint32_t rtt_us = k_cyc_to_us_near32(now_cyc - ping_send_cycles);
								// Capture receiver time from PONG param (bytes 8..11)
								uint32_t pong_rx_time = ((uint32_t)rx_payload.data[8] << 24)
														| ((uint32_t)rx_payload.data[9] << 16)
														| ((uint32_t)rx_payload.data[10] << 8)
														| ((uint32_t)rx_payload.data[11]);

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
												"rtt=%u us",
												ping_success_streak,
												(unsigned)rtt_us
											);
										} else {
											LOG_INF(
												"PONG ok - link restored after %u probes, "
												"rtt=%u.%03u ms",
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
											"PONG ok, rtt=%u us (ctr=%u)",
											(unsigned)rtt_us,
											rx_ctr
										);
									} else {
										LOG_INF(
											"PONG ok, rtt=%u.%03u ms (ctr=%u)",
											(unsigned)(rtt_us / 1000),
											(unsigned)(rtt_us % 1000),
											rx_ctr
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
				}
			}
			break;
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

	uint16_t jitter = (rand() % 200) - 100;  // ±100 µs
	uint16_t retransmit_delay_with_jitter = RADIO_RETRANSMIT_DELAY + jitter;

	if (tx) {
		config.protocol = ESB_PROTOCOL_ESB_DPL;
		// config.mode = ESB_MODE_PTX;
		config.event_handler = event_handler;
		config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = retransmit_delay_with_jitter;
		config.retransmit_count = 2;
		config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
		config.use_fast_ramp_up = true;
	} else {
		config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = retransmit_delay_with_jitter;
		// config.retransmit_count = 3;
		// config.tx_mode = ESB_TXMODE_AUTO;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
		// config.use_fast_ramp_up = true;
	}

	err = esb_init(&config);

	if (!err) {
		esb_set_rf_channel(RADIO_RF_CHANNEL);
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
		while (paired_addr[0] != checksum) {
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

void esb_write(uint8_t* data, bool no_ack) {
	if (!esb_initialized || !esb_paired) {
		return;
	}
	if (!clock_status) {
		clocks_start();
	}

	int64_t now = k_uptime_get();
	// Tick rate counter
	esb_write_rate_tick();
	// Set ACK behavior per call
	tx_payload.noack = no_ack;
	// Send on a unique ESB pipe per tracker (0–7)
	tx_payload.pipe = tracker_id % 8;

	// Determine payload length from packet type
	// Normal data packets are 16 bytes + 1 sequence = 17 total
	// PING packets use ESB_PING_LEN (13)
	if (data[0] == ESB_PING_TYPE) {
		// Ping packet
		tx_payload.length = ESB_PING_LEN;
		// Set sequence number
		data[2] = ping_counter;
		// Calculate crc8 checksum over first 12 bytes
		uint8_t crc_calc = crc8_ccitt(0x07, data, ESB_PING_LEN - 1);
		data[ESB_PING_LEN - 1] = crc_calc;
		ping_counter++;
	} else {
		// Normal tracker data
		tx_payload.length = 17;
	}
	memcpy(tx_payload.data, data, tx_payload.length);

	esb_flush_tx();
	int queue_status = esb_write_payload(&tx_payload);  // queue normal data first
	// int retry_count = 0;
	// while (queue_status != 0)  // TX buffer full
	// {
	// 	if (retry_count >= 2) {
	// 		break;  // drop packet after 2 retries
	// 	}
	// 	// Drop one oldest packet instead of flushing all
	// 	esb_pop_tx();
	// 	queue_status = esb_write_payload(&tx_payload);
	// 	retry_count++;
	// }

	// Record last TX time for idle probe scheduling
	if (queue_status == 0) {
		last_tx_time = now;
		esb_write_queued++;
	}

	// If we sent a heartbeat (ACK requested), mark pending and record timing
	if (!no_ack && tx_payload.data[0] == ESB_PING_TYPE && queue_status == 0) {
		uint32_t now32 = (uint32_t)now;
		ping_pending = true;
		ping_ctr_sent = tx_payload.data[2];
		ping_send_time = now32;
		ping_send_cycles = k_cycle_get_32();
		last_tx_time = now;
		LOG_INF(
			"PING sent (ctr=%u)",
			(unsigned)tx_payload.data[2]
		);
	}

	// Ensure TX progresses (manual mode) only if we queued something
	if (queue_status == 0) {
		int tx_err = esb_start_tx();
		if (tx_err == -EBUSY) {
			// Radio is already transmitting; not an error in manual mode
			LOG_DBG("esb_start_tx busy");
		} else if (tx_err) {
			LOG_WRN("esb_start_tx error: %d", tx_err);
		} else {
			LOG_DBG("esb_start_tx ok");
		}
	} else {
		// We failed to queue this packet (FIFO full), but there are already
		// packets in the FIFO. Ensure TX is started to drain the queue.
		int tx_err = esb_start_tx();
		if (tx_err == -EBUSY) {
			LOG_DBG("esb_start_tx busy");
		} else if (tx_err && tx_err != -ENOENT) {
			// -ENOENT would indicate no payload pending; ignore others
			LOG_WRN("esb_start_tx (drain) error: %d", tx_err);
		}
	}
}

bool esb_ready(void) { return esb_initialized && esb_paired; }


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
		if (ping_pending && (now_idle - ping_send_time) > PING_TIMEOUT_MS) {
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
