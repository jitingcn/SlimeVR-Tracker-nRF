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

uint32_t heartbeat_failures = 0;  // Separate counter for heartbeat failures
int64_t connection_error_start_time = 0;
int64_t last_heartbeat_time = 0;
static bool shutdown_requested = false;
static bool heartbeat_failed = false;
static bool heartbeat_pending
	= false;  // Flag to track if a heartbeat is currently being sent
static bool pair_ack_pending
	= false;  // True once step 1 is sent and we expect a receiver response

static struct esb_payload rx_payload;
static struct esb_payload tx_payload
	= ESB_CREATE_PAYLOAD(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_pair
	= ESB_CREATE_PAYLOAD(0, 0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_heartbeat
	= ESB_CREATE_PAYLOAD(0, 0xFF);  // Heartbeat packet

static uint8_t paired_addr[8] = {0};

static bool esb_initialized = false;
static bool esb_paired = false;

#define TX_ERROR_THRESHOLD 30
#define HEARTBEAT_INTERVAL_MS 1000
#define RADIO_RETRANSMIT_DELAY CONFIG_RADIO_RETRANSMIT_DELAY
#define RADIO_RF_CHANNEL CONFIG_RADIO_RF_CHANNEL
// Require N consecutive successful ACK probes before clearing connection error
#ifndef HEARTBEAT_RECOVERY_THRESHOLD
#define HEARTBEAT_RECOVERY_THRESHOLD 3
#endif
// Allow longer wait before declaring PING timeout
#ifndef PING_TIMEOUT_MS
#define PING_TIMEOUT_MS (2 * HEARTBEAT_INTERVAL_MS)
#endif
// Stagger PINGs when multiple trackers share ACK pipes
#ifndef PING_SLOTS_PER_PIPE
#define PING_SLOTS_PER_PIPE 6
#endif
// Minimum guard time at slot edges to reduce boundary collisions
#ifndef PING_GUARD_MS
#define PING_GUARD_MS 50
#endif

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 512, esb_thread, NULL, NULL, NULL, 6, 0, 0);
static K_MUTEX_DEFINE(esb_tx_mutex);
static void esb_ping_thread(void);
K_THREAD_DEFINE(esb_ping_thread_id, 512, esb_ping_thread, NULL, NULL, NULL, 6, 0, 0);
static int64_t last_tx_time = 0;
static uint32_t heartbeat_success_streak = 0;  // consecutive success counter
static bool ping_pending = false;
static uint8_t ping_counter = 0;
static uint8_t ping_ctr_sent = 0;
static uint8_t ping_tracker_id = 0;
static uint32_t ping_send_time = 0;  // uptime in ms (low 32 bits)
static uint32_t ping_send_cycles = 0;  // high-resolution cycle timestamp
static uint16_t ping_phase_ms = 0;  // per-tracker phase offset within interval
static uint16_t base_phase_ms = 0;  // baseline phase computed at pairing
// Dynamic TDMA scheduler parameters
static uint16_t sched_cycle_ms = HEARTBEAT_INTERVAL_MS;
static uint8_t sched_slots = PING_SLOTS_PER_PIPE;
static uint16_t sched_slot_width_ms = HEARTBEAT_INTERVAL_MS / PING_SLOTS_PER_PIPE;
static uint16_t ping_slot_width_ms
	= HEARTBEAT_INTERVAL_MS
	/ PING_SLOTS_PER_PIPE;  // kept for logs; mirrors sched_slot_width_ms
// Dynamic TDMA rephase to mitigate collisions
static uint16_t tdma_shift_ms = 0;
static uint32_t last_collision_time = 0;
static uint8_t collision_count = 0;
#ifndef PING_COLLISION_WINDOW_MS
#define PING_COLLISION_WINDOW_MS 1000
#endif
#ifndef PING_COLLISION_THRESHOLD
#define PING_COLLISION_THRESHOLD 3
#endif
static inline uint16_t tdma_phase_ms(void) {
	return (ping_phase_ms + tdma_shift_ms) % sched_cycle_ms;
}
// Cooldown to avoid flapping when adapting TDMA phase on foreign PONGs
#ifndef TDMA_ADAPT_COOLDOWN_MS
#define TDMA_ADAPT_COOLDOWN_MS 500
#endif
static uint32_t last_tdma_adapt_ms = 0;
// TDMA anchoring to receiver time: use last receiver uptime from PONG to align windows
static uint32_t rx_time_base_ms
	= 0;  // receiver uptime low32 captured at last valid PONG
static uint32_t rx_time_base_local_ms = 0;  // local uptime at capture
static bool rx_time_valid = false;  // whether receiver time is known
/* deprecated: use window/cycle gating */
// static int last_ping_slot_num = -1;
static int last_tdma_window_idx = -1;  // window index within interval
static int last_tdma_cycle = -1;  // receiver cycle index
// Forward declaration for adaptive TDMA parameter updates
static void tdma_update_params(uint16_t cycle_ms, uint8_t slots, const char* why);
// Forward declaration to set absolute TDMA phase
static void tdma_set_phase_absolute(uint16_t new_phase_ms, const char* why);

void event_handler(struct esb_evt const* event) {
	switch (event->evt_id) {
		case ESB_EVENT_TX_SUCCESS:
			// Do not clear heartbeat state on generic TX success.
			// We only consider heartbeats successful upon validated PONG reception.
			if (esb_paired) {
				clocks_stop();
			}
			break;
		case ESB_EVENT_TX_FAILED:
			// Only count heartbeat failures for connection timeout
			if (heartbeat_pending) {
				heartbeat_failed = true;
				heartbeat_pending = false;  // Clear the pending flag
				ping_pending = false;  // Allow new PINGs to be scheduled
				heartbeat_success_streak = 0;  // Reset recovery streak on any failure
				heartbeat_failures++;
				if (heartbeat_failures % 10 == 0)  // Log every 10 failures
				{
					LOG_WRN("Heartbeat failed, total failures: %d", heartbeat_failures);
				}
				if (heartbeat_failures
					== TX_ERROR_THRESHOLD)  // consecutive heartbeat failures
				{
					connection_error_start_time
						= k_uptime_get();  // Mark when connection errors started
					LOG_WRN(
						"Heartbeat failure threshold reached (%d failures), starting "
						"timeout timer",
						TX_ERROR_THRESHOLD
					);
				}
			}
			LOG_DBG("TX FAILED");
			if (esb_paired) {
				clocks_stop();
			}
			break;
		case ESB_EVENT_RX_RECEIVED:
			// Read all RX payloads
			while (!esb_read_rx_payload(&rx_payload))  // zero, rx success
			{
				// Hex dump first up to 13 bytes for visibility
				char hexbuf[3 * 13 + 1] = {0};
				int dump_len = rx_payload.length < 13 ? rx_payload.length : 13;
				for (int i = 0; i < dump_len; i++) {
					snprintk(&hexbuf[i * 3], 4, "%02X ", rx_payload.data[i]);
				}
				LOG_INF(
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
					if (rx_payload.length == 4) {
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
					} else if (rx_payload.length == ESB_PONG_LEN
							   && rx_payload.data[0] == ESB_PONG_TYPE) {
						uint8_t crc_calc
							= crc8_ccitt(0x07, rx_payload.data, ESB_PONG_LEN - 1);
						if (rx_payload.length < ESB_PONG_LEN
							|| rx_payload.data[ESB_PONG_LEN - 1] != crc_calc) {
							LOG_WRN("PONG CRC mismatch");
							continue;
						}
						uint8_t rx_id = rx_payload.data[1];
						uint8_t rx_ctr = rx_payload.data[2];
						// Accept current PONG or up to two late counters
						bool match_current
							= ((uint8_t)rx_ctr == (uint8_t)ping_ctr_sent);
						bool match_previous
							= ((uint8_t)rx_ctr == (uint8_t)(ping_ctr_sent - 1));
						bool match_prev2
							= ((uint8_t)rx_ctr == (uint8_t)(ping_ctr_sent - 2));
						if (rx_id == ping_tracker_id
							&& (match_current || match_previous || match_prev2)) {
							uint32_t now32 = (uint32_t)k_uptime_get();
							// uint32_t rtt = (now32 - ping_send_time); // superseded by
							// cycle-based RTT
							uint32_t now_cyc = k_cycle_get_32();
							uint32_t rtt_us
								= k_cyc_to_us_near32(now_cyc - ping_send_cycles);
							// Capture receiver time from PONG param (bytes 8..11) to
							// anchor TDMA slots
							uint32_t pong_rx_time = ((uint32_t)rx_payload.data[8] << 24)
												  | ((uint32_t)rx_payload.data[9] << 16)
												  | ((uint32_t)rx_payload.data[10] << 8)
												  | ((uint32_t)rx_payload.data[11]);
							rx_time_base_ms = pong_rx_time;
							rx_time_base_local_ms = now32;
							rx_time_valid = true;
							heartbeat_failures = 0;
							heartbeat_failed = false;
							heartbeat_pending = false;
							ping_pending = false;
							if (get_status(SYS_STATUS_CONNECTION_ERROR) == true) {
								heartbeat_success_streak++;
								if (heartbeat_success_streak
									>= HEARTBEAT_RECOVERY_THRESHOLD) {
									set_status(SYS_STATUS_CONNECTION_ERROR, false);
									connection_error_start_time = 0;
									shutdown_requested = false;
									// Log receiver time anchor and TDMA slot position
									uint32_t est_rx_now
										= rx_time_base_ms
										+ (uint32_t)(now32 - rx_time_base_local_ms);
									uint32_t rx_slot_pos
										= est_rx_now % HEARTBEAT_INTERVAL_MS;
									LOG_INF(
										"PONG time: rx_ms=%u est_rx_now=%u slot_pos=%u "
										"phase=%u width=%u",
										pong_rx_time,
										est_rx_now,
										rx_slot_pos,
										ping_phase_ms,
										ping_slot_width_ms
									);
									if (rtt_us < 1000) {
										LOG_INF(
											"PONG ok - link restored after %u probes, "
											"rtt=%u us",
											heartbeat_success_streak,
											(unsigned)rtt_us
										);
									} else {
										LOG_INF(
											"PONG ok - link restored after %u probes, "
											"rtt=%u.%03u ms",
											heartbeat_success_streak,
											(unsigned)(rtt_us / 1000),
											(unsigned)(rtt_us % 1000)
										);
									}
									heartbeat_success_streak = 0;
								} else {
									// Log receiver time anchor and TDMA slot position
									uint32_t est_rx_now
										= rx_time_base_ms
										+ (uint32_t)(now32 - rx_time_base_local_ms);
									uint32_t rx_slot_pos
										= est_rx_now % HEARTBEAT_INTERVAL_MS;
									LOG_INF(
										"PONG time: rx_ms=%u est_rx_now=%u slot_pos=%u "
										"phase=%u width=%u",
										pong_rx_time,
										est_rx_now,
										rx_slot_pos,
										ping_phase_ms,
										ping_slot_width_ms
									);
									if (rtt_us < 1000) {
										LOG_INF(
											"PONG ok %u/%u, rtt=%u us",
											heartbeat_success_streak,
											HEARTBEAT_RECOVERY_THRESHOLD,
											(unsigned)rtt_us
										);
									} else {
										LOG_INF(
											"PONG ok %u/%u, rtt=%u.%03u ms",
											heartbeat_success_streak,
											HEARTBEAT_RECOVERY_THRESHOLD,
											(unsigned)(rtt_us / 1000),
											(unsigned)(rtt_us % 1000)
										);
									}
									// Success mode: compress slots per cycle and
									// restore baseline phase
									if (sched_slots > PING_SLOTS_PER_PIPE) {
										tdma_update_params(
											sched_cycle_ms,
											PING_SLOTS_PER_PIPE,
											"success"
										);
									}
									tdma_set_phase_absolute(base_phase_ms, "success");
								}
							} else {
								if (heartbeat_success_streak
									> HEARTBEAT_RECOVERY_THRESHOLD) {
									heartbeat_success_streak
										= HEARTBEAT_RECOVERY_THRESHOLD;
								}
								// Log receiver time anchor and TDMA slot position
								uint32_t est_rx_now
									= rx_time_base_ms
									+ (uint32_t)(now32 - rx_time_base_local_ms);
								uint32_t rx_slot_pos
									= est_rx_now % HEARTBEAT_INTERVAL_MS;
								LOG_INF(
									"PONG time: rx_ms=%u est_rx_now=%u slot_pos=%u "
									"phase=%u width=%u",
									pong_rx_time,
									est_rx_now,
									rx_slot_pos,
									ping_phase_ms,
									ping_slot_width_ms
								);
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
						} else {
							LOG_INF(
								"Ignore PONG (id=%u ctr=%u pending=%d "
								"expected=%u/%u/%u)",
								rx_id,
								rx_ctr,
								ping_pending,
								ping_ctr_sent,
								(uint8_t)(ping_ctr_sent - 1),
								(uint8_t)(ping_ctr_sent - 2)
							);
							// Detect collisions: foreign PONG on our pipe indicates
							// contention
							uint8_t my_pipe = 1 + (ping_tracker_id % 7);
							if (rx_payload.pipe == my_pipe
								&& rx_id != ping_tracker_id) {
								uint32_t nowc = (uint32_t)k_uptime_get();
								// Backoff immediately: cancel current pending PING and
								// defer to next window
								if (ping_pending) {
									ping_pending = false;
									heartbeat_pending = false;
									last_heartbeat_time
										= nowc;  // schedule next attempt after interval
									// Prevent scheduler from firing again in this
									// window
									uint32_t est_rx_ms
										= rx_time_valid
											? (rx_time_base_ms
											   + (uint32_t)(nowc
															- rx_time_base_local_ms))
											: nowc;
									int delta = (int)(est_rx_ms % sched_cycle_ms)
											  - (int)tdma_phase_ms();
									if (delta < 0) {
										delta += sched_cycle_ms;
									}
									int window_idx = delta / (int)sched_slot_width_ms;
									int cycle_idx = est_rx_ms / sched_cycle_ms;
									last_tdma_window_idx = window_idx;
									last_tdma_cycle = cycle_idx;
									LOG_WRN(
										"TDMA backoff: my_id=%u pipe=%u win=%d cyc=%d",
										ping_tracker_id,
										my_pipe,
										window_idx,
										cycle_idx
									);
									// Aggressive mode: increase slots per cycle
									// temporarily to probe separation
									if (sched_slots < 12) {
										tdma_update_params(
											sched_cycle_ms,
											sched_slots * 2,
											"backoff"
										);
									}
								}
								if (nowc - last_collision_time
									> PING_COLLISION_WINDOW_MS) {
									collision_count = 0;
									last_collision_time = nowc;
								}
								collision_count++;
								// Immediate rephase on first observed foreign PONG if
								// cooldown elapsed
								if (nowc - last_tdma_adapt_ms
									> TDMA_ADAPT_COOLDOWN_MS) {
									uint32_t other_rx_time
										= ((uint32_t)rx_payload.data[8] << 24)
										| ((uint32_t)rx_payload.data[9] << 16)
										| ((uint32_t)rx_payload.data[10] << 8)
										| ((uint32_t)rx_payload.data[11]);
									uint16_t other_pos
										= (uint16_t)(other_rx_time % sched_cycle_ms);
									uint16_t width = sched_slot_width_ms;
									uint16_t phase = tdma_phase_ms();
									// Aim our start roughly opposite to foreign PONG
									// within slot
									uint16_t desired_start
										= (uint16_t)((other_pos + (width / 2))
													 % sched_cycle_ms);
									int32_t rel = (int32_t)desired_start
												- (int32_t)ping_phase_ms;
									while (rel < 0) {
										rel += HEARTBEAT_INTERVAL_MS;
									}
									uint16_t new_shift = (uint16_t)(rel % width);
									// Respect guard margin
									uint16_t slot_margin = (width > PING_GUARD_MS)
															 ? (width - PING_GUARD_MS)
															 : width;
									if (new_shift > slot_margin) {
										new_shift = slot_margin;
									}
									tdma_shift_ms = new_shift;
									last_tdma_adapt_ms = nowc;
									LOG_WRN(
										"TDMA adapt on foreign PONG: my_id=%u "
										"other_id=%u pipe=%u other_pos=%u my_phase=%u "
										"width=%u new_shift=%u new_phase=%u",
										ping_tracker_id,
										rx_id,
										rx_payload.pipe,
										other_pos,
										phase,
										width,
										tdma_shift_ms,
										tdma_phase_ms()
									);
									// After adapt, recompute and mark current window
									// only; allow next window to emit
									uint32_t est_rx2
										= rx_time_valid
											? (rx_time_base_ms
											   + (uint32_t)(nowc
															- rx_time_base_local_ms))
											: nowc;
									int d2 = (int)(est_rx2 % sched_cycle_ms)
										   - (int)tdma_phase_ms();
									if (d2 < 0) {
										d2 += sched_cycle_ms;
									}
									last_tdma_window_idx
										= d2 / (int)sched_slot_width_ms;
									last_tdma_cycle = est_rx2 / sched_cycle_ms;
								}
								if (collision_count >= PING_COLLISION_THRESHOLD) {
									// Rephase within slot to reduce overlap
									uint16_t slot_margin
										= (ping_slot_width_ms > PING_GUARD_MS)
											? (ping_slot_width_ms - PING_GUARD_MS)
											: ping_slot_width_ms;
									uint16_t new_shift
										= slot_margin ? (rand() % slot_margin) : 0;
									tdma_shift_ms = new_shift;
									collision_count = 0;
									last_collision_time = nowc;
									LOG_WRN(
										"TDMA rephase: id=%u pipe=%u shift=%u phase=%u "
										"width=%u",
										ping_tracker_id,
										my_pipe,
										tdma_shift_ms,
										tdma_phase_ms(),
										ping_slot_width_ms
									);
								}
							}
						}
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
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = retransmit_delay_with_jitter;
		config.retransmit_count = 3;
		config.tx_mode = ESB_TXMODE_MANUAL;  // Ensure we explicitly start TX when ready
		// config.payload_length = 32;
		config.selective_auto_ack
			= true;  // TODO: while pairing, should be set to false
		// config.use_fast_ramp_up = true;
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
	// Reset heartbeat state when starting pairing
	heartbeat_failures = 0;
	set_status(SYS_STATUS_CONNECTION_ERROR, false);
	connection_error_start_time = 0;
	shutdown_requested = false;
	heartbeat_failed = false;
	heartbeat_pending = false;
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
	ping_tracker_id = paired_addr[1];

	esb_set_addr_paired();
	esb_paired = true;
	clocks_stop();

	// Compute per-tracker PING phase to stagger across shared pipes
	uint8_t pipe_index_calc = 1 + (ping_tracker_id % 7);
	uint8_t group_index = ping_tracker_id / 7;  // trackers beyond first 7 share pipes
	// Distribute subslots across pipes to reduce cross-pipe collisions
	uint8_t subslot = (group_index + (pipe_index_calc - 1)) % PING_SLOTS_PER_PIPE;
	// Derive a stable per-tracker intra-slot offset from unique device address
	uint64_t dev64 = (*(uint64_t*)NRF_FICR->DEVICEADDR) & 0xFFFFFFFFFFFFULL;
	uint32_t hash = (uint32_t)(dev64 ^ (dev64 >> 16) ^ (dev64 >> 32));
	uint16_t slot_margin = (ping_slot_width_ms > PING_GUARD_MS)
							 ? (ping_slot_width_ms - PING_GUARD_MS)
							 : ping_slot_width_ms;
	uint16_t intra_offset = (slot_margin > 0) ? (hash % slot_margin) : 0;
	ping_phase_ms = subslot * ping_slot_width_ms + intra_offset;
	base_phase_ms = ping_phase_ms;
	LOG_INF(
		"PING TDMA: id=%u pipe=%u subslot=%u intra=%u ms phase=%u ms width=%u ms",
		ping_tracker_id,
		pipe_index_calc,
		subslot,
		intra_offset,
		ping_phase_ms,
		ping_slot_width_ms
	);
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

void esb_write(uint8_t* data) {
	if (!esb_initialized || !esb_paired) {
		return;
	}
	if (!clock_status) {
		clocks_start();
	}

	k_mutex_lock(&esb_tx_mutex, K_FOREVER);
	// Heartbeat handled by scheduler thread, do not piggyback here
	int64_t now = k_uptime_get();
	tx_payload.noack = true;  // normal data should not request ACK
	// Select a unique ESB pipe based on tracker ID to avoid cross-ACKs
	// Keep sensor data on pipe 0
	tx_payload.pipe = 0;

	memcpy(tx_payload.data, data, tx_payload.length);

	int queue_status = esb_write_payload(&tx_payload);  // queue normal data first
	int retry_count = 0;
	while (queue_status != 0)  // TX buffer full
	{
		if (retry_count >= 3) {
			LOG_WRN("Failed to queue data after %d retries", retry_count);
			esb_flush_tx();
		}
		esb_pop_tx();
		queue_status = esb_write_payload(&tx_payload);
		retry_count++;
	}

	// Do not enqueue PING here; scheduler handles it

	// Record last TX time for idle probe scheduling
	if (queue_status == 0) {
		last_tx_time = now;
	}

	// Ensure TX progresses (manual mode)
	int tx_err = esb_start_tx();
	if (tx_err && tx_err != -EBUSY) {
		LOG_WRN("esb_start_tx error: %d", tx_err);
	} else if (tx_err == -EBUSY) {
		LOG_INF("esb_start_tx busy, pending TX continues");
	}

	k_mutex_unlock(&esb_tx_mutex);
	send_data = true;
}

bool esb_ready(void) { return esb_initialized && esb_paired; }

static void esb_send_idle_probe(void) {
	if (!esb_initialized || !esb_paired) {
		return;
	}
	if (heartbeat_pending || ping_pending) {
		return;
	}

	k_mutex_lock(&esb_tx_mutex, K_FOREVER);
	if (!clock_status) {
		clocks_start();
	}

	// Deprecated: PINGs are handled by scheduler; do not enqueue here
	uint32_t now32 = (uint32_t)k_uptime_get();
	// Align TDMA slot using receiver time if available
	uint32_t est_rx_ms
		= rx_time_valid ? (rx_time_base_ms + (uint32_t)(now32 - rx_time_base_local_ms))
						: (uint32_t)now32;
	int pos_ms = (int)(est_rx_ms % HEARTBEAT_INTERVAL_MS);
	LOG_DBG(
		"Skip idle PING: pos=%d phase=%u width=%u",
		pos_ms,
		ping_phase_ms,
		ping_slot_width_ms
	);
	k_mutex_unlock(&esb_tx_mutex);
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
		if (heartbeat_failures >= TX_ERROR_THRESHOLD) {
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
		if (ping_pending && (now_idle - last_heartbeat_time) > PING_TIMEOUT_MS) {
			// Consider missing PONG a failure, clear pending
			heartbeat_failed = true;
			heartbeat_pending = false;
			ping_pending = false;
			heartbeat_success_streak = 0;
			heartbeat_failures++;
			LOG_WRN("PING timeout, failures=%u", heartbeat_failures);
			if (heartbeat_failures == TX_ERROR_THRESHOLD) {
				connection_error_start_time = now_idle;
				LOG_WRN(
					"Heartbeat failure threshold reached (%d failures), starting "
					"timeout timer",
					TX_ERROR_THRESHOLD
				);
			}
		}

		if (esb_initialized && esb_paired && !heartbeat_pending && !ping_pending
			&& (now_idle - last_tx_time) > HEARTBEAT_INTERVAL_MS) {
			esb_send_idle_probe();
		}
		k_msleep(100);
	}
}
// Dedicated PING scheduler thread: emits at most one PING per TDMA slot
static void esb_ping_thread(void) {
	while (1) {
		if (!esb_initialized || !esb_paired) {
			k_msleep(50);
			continue;
		}
		if (heartbeat_pending || ping_pending) {
			k_msleep(10);
			continue;
		}
		uint32_t now32 = (uint32_t)k_uptime_get();
		// Align TDMA slot using receiver time if available
		uint32_t est_rx_ms
			= rx_time_valid
				? (rx_time_base_ms + (uint32_t)(now32 - rx_time_base_local_ms))
				: (uint32_t)now32;
		uint16_t pos_ms = (uint16_t)(est_rx_ms % sched_cycle_ms);
		uint16_t phase = tdma_phase_ms();
		uint16_t width = ping_slot_width_ms;
		// Compute window index aligned to phase within interval
		int delta = (int)pos_ms - (int)phase;
		if (delta < 0) {
			delta += HEARTBEAT_INTERVAL_MS;
		}
		int window_idx = delta / (int)width;
		int cycle_idx = est_rx_ms / sched_cycle_ms;
		// Clear window gating at cycle boundary to avoid stuck suppression
		if (cycle_idx != last_tdma_cycle) {
			last_tdma_window_idx = -1;
		}
		bool in_window = (delta >= 0) && (delta < (int)width);
		// Suppress repeats only within the same window AND same cycle
		if (!in_window
			|| (window_idx == last_tdma_window_idx && cycle_idx == last_tdma_cycle)) {
			k_msleep(5);
			continue;
		}
		// Build PING
		tx_heartbeat.noack = false;
		tx_heartbeat.length = ESB_PING_LEN;
		tx_heartbeat.data[0] = ESB_PING_TYPE;
		tx_heartbeat.data[1] = ping_tracker_id;
		ping_ctr_sent = ping_counter;
		tx_heartbeat.data[2] = ping_ctr_sent;
		uint8_t pipe_index = 1 + (ping_tracker_id % 7);
		tx_heartbeat.pipe = pipe_index;
		tx_heartbeat.data[3] = (now32 >> 24) & 0xFF;
		tx_heartbeat.data[4] = (now32 >> 16) & 0xFF;
		tx_heartbeat.data[5] = (now32 >> 8) & 0xFF;
		tx_heartbeat.data[6] = (now32) & 0xFF;
		tx_heartbeat.data[7] = 0x00;  // flags
		memset(&tx_heartbeat.data[8], 0x00, 4);  // param reserved
		tx_heartbeat.data[12] = crc8_ccitt(0x07, tx_heartbeat.data, ESB_PING_LEN - 1);

		k_mutex_lock(&esb_tx_mutex, K_FOREVER);
		int w = esb_write_payload(&tx_heartbeat);
		if (w == -ENOSPC) {
			esb_pop_tx();
			w = esb_write_payload(&tx_heartbeat);
		}
		if (w == 0) {
			heartbeat_pending = true;
			ping_pending = true;
			ping_send_time = now32;
			ping_send_cycles = k_cycle_get_32();
			last_heartbeat_time = k_uptime_get();
			last_tx_time = last_heartbeat_time;
			int tx_err = esb_start_tx();
			if (tx_err && tx_err != -EBUSY) {
				LOG_WRN("PING scheduler start_tx error: %d", tx_err);
			}
			LOG_INF(
				"Scheduled PING id=%u ctr=%u pipe=%u pos=%u phase=%u width=%u win=%d "
				"cyc=%d",
				ping_tracker_id,
				ping_ctr_sent,
				pipe_index,
				pos_ms,
				phase,
				sched_slot_width_ms,
				window_idx,
				cycle_idx
			);
			last_tdma_window_idx = window_idx;
			last_tdma_cycle = cycle_idx;
			ping_counter++;
		} else {
			LOG_WRN("PING scheduler failed to queue: %d", w);
		}
		k_mutex_unlock(&esb_tx_mutex);
		// Short sleep to react quickly to next window when probing aggressively
		k_msleep(2);
	}
}
// Helper to update TDMA scheduler parameters
static void tdma_update_params(uint16_t cycle_ms, uint8_t slots, const char* why) {
	if (cycle_ms == 0 || slots == 0) {
		return;
	}
	sched_cycle_ms = cycle_ms;
	sched_slots = slots;
	sched_slot_width_ms = (uint16_t)(sched_cycle_ms / sched_slots);
	ping_slot_width_ms = sched_slot_width_ms;
	LOG_INF(
		"TDMA params update: cycle=%u slots=%u width=%u (%s)",
		sched_cycle_ms,
		sched_slots,
		sched_slot_width_ms,
		why ? why : ""
	);
}

// Set absolute TDMA phase (in ms within sched_cycle_ms) by adjusting tdma_shift_ms
static void tdma_set_phase_absolute(uint16_t new_phase_ms, const char* why) {
	uint16_t cycle = sched_cycle_ms ? sched_cycle_ms : HEARTBEAT_INTERVAL_MS;
	uint16_t desired = (uint16_t)(new_phase_ms % cycle);
	// Compute shift relative to baseline ping_phase_ms
	int32_t rel = (int32_t)desired - (int32_t)ping_phase_ms;
	while (rel < 0) {
		rel += cycle;
	}
	tdma_shift_ms = (uint16_t)(rel % cycle);
	LOG_INF(
		"TDMA set phase: base=%u desired=%u shift=%u cycle=%u (%s)",
		ping_phase_ms,
		desired,
		tdma_shift_ms,
		cycle,
		why ? why : ""
	);
	// Allow scheduler to emit in the next matching window immediately
	last_tdma_window_idx = -1;
}
