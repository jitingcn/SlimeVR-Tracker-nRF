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
#include "system/system.h"
#include "connection.h"
#include "zephyr/logging/log.h"

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */
#include <zephyr/sys/crc.h>

#include "esb.h"

uint8_t last_reset = 0;
//const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);
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
static bool heartbeat_pending = false;  // Flag to track if a heartbeat is currently being sent
static bool pair_ack_pending = false;   // True once step 1 is sent and we expect a receiver response

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_heartbeat = ESB_CREATE_PAYLOAD(0, 0xFF); // Heartbeat packet

static uint8_t paired_addr[8] = {0};

static bool esb_initialized = false;
static bool esb_paired = false;

#define TX_ERROR_THRESHOLD 60
#define HEARTBEAT_INTERVAL_MS 1000
#define RADIO_RETRANSMIT_DELAY CONFIG_RADIO_RETRANSMIT_DELAY
#define RADIO_RF_CHANNEL CONFIG_RADIO_RF_CHANNEL

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 512, esb_thread, NULL, NULL, NULL, 6, 0, 0);

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		// Reset heartbeat failures only if this was a heartbeat packet
		if (heartbeat_pending)
		{
			heartbeat_failures = 0;
			heartbeat_failed = false;
			heartbeat_pending = false;  // Clear the pending flag
			connection_error_start_time = 0; // Reset timeout timer on heartbeat success
			shutdown_requested = false; // Reset shutdown request
			if (get_status(SYS_STATUS_CONNECTION_ERROR) == true)
			{
				set_status(SYS_STATUS_CONNECTION_ERROR, false);
				LOG_INF("Heartbeat success - connection restored");
			}
			else
			{
				LOG_DBG("Heartbeat success");
			}
		}
		if (esb_paired)
			clocks_stop();
		break;
	case ESB_EVENT_TX_FAILED:
		// Only count heartbeat failures for connection timeout
		if (heartbeat_pending)
		{
			heartbeat_failed = true;
			heartbeat_pending = false;  // Clear the pending flag
			heartbeat_failures++;
			if (heartbeat_failures % 10 == 0)  // Log every 10 failures
			{
				LOG_WRN("Heartbeat failed, total failures: %d", heartbeat_failures);
			}
			if (heartbeat_failures == TX_ERROR_THRESHOLD) // consecutive heartbeat failures
			{
				connection_error_start_time = k_uptime_get(); // Mark when connection errors started
				LOG_WRN("Heartbeat failure threshold reached (%d failures), starting timeout timer", TX_ERROR_THRESHOLD);
			}
		}
		LOG_DBG("TX FAILED");
		if (esb_paired)
			clocks_stop();
		break;
	case ESB_EVENT_RX_RECEIVED:
		// TODO: have to read rx until -ENODATA (or -EACCES/-EINVAL)
		if (!esb_read_rx_payload(&rx_payload)) // zero, rx success
		{
			if (!paired_addr[0]) // zero, not paired
			{
				LOG_DBG("tx: %16llX rx: %16llX", *(uint64_t *)tx_payload_pair.data, *(uint64_t *)rx_payload.data);
				if (rx_payload.length == 8)
				{
					if (!pair_ack_pending)
					{
						LOG_DBG("Ignoring unsolicited pairing response");
						break;
					}
					if (rx_payload.data[0] != tx_payload_pair.data[0])
					{
						LOG_DBG("Ignoring pairing response with mismatched checksum %02X", rx_payload.data[0]);
						pair_ack_pending = false;
						break;
					}
					uint64_t responder_addr = 0;
					memcpy(&responder_addr, &rx_payload.data[2], 6);
					responder_addr &= 0xFFFFFFFFFFFFULL;
					uint64_t local_addr = (*(uint64_t *)NRF_FICR->DEVICEADDR) & 0xFFFFFFFFFFFFULL;
					if (responder_addr == local_addr)
					{
						LOG_WRN("Ignoring pairing response sourced from local device address");
						pair_ack_pending = false;
						break;
					}
					memcpy(paired_addr, rx_payload.data, sizeof(paired_addr));
					pair_ack_pending = false;
				}
			}
			else
			{
				if (rx_payload.length == 4)
				{
					// TODO: Device should never receive packets if it is already paired, why is this packet received?
					// This may be part of acknowledge
//					if (!nrfx_timer_init_check(&m_timer))
					{
						LOG_WRN("Timer not initialized");
						break;
					}
					if (timer_state == false)
					{
//						nrfx_timer_resume(&m_timer);
						timer_state = true;
					}
//					nrfx_timer_clear(&m_timer);
					last_reset = 0;
					led_clock = (rx_payload.data[0] << 8) + rx_payload.data[1]; // sync led flashes :)
					led_clock_offset = 0;
					LOG_DBG("RX, timer reset");
					pair_ack_pending = false;
				}
			}
		}
		break;
	}
}

bool clock_status = false;

#if defined(CONFIG_CLOCK_CONTROL_NRF)
static struct onoff_manager *clk_mgr;

static int clocks_init(void)
{
	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr)
						pair_ack_pending = false;
	{
		LOG_ERR("Unable to get the Clock manager");
		return -ENOTSUP;
	}

	return 0;
}

SYS_INIT(clocks_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

int clocks_start(void)
{
						pair_ack_pending = true;
	if (clock_status)
		return 0;
	int err;
	int res;
	struct onoff_client clk_cli;
	int fetch_attempts = 0;

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0)
	{
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do
	{
		k_usleep(100);
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res)
		{
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

	LOG_DBG("HF clock started");
	clock_status = true;
	return 0;
}

void clocks_stop(void)
{
	if (!clock_status)
		return;
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
	k_thread_create(&clocks_thread_id, clocks_thread_id_stack, K_THREAD_STACK_SIZEOF(clocks_thread_id_stack), (k_thread_entry_t)clocks_start, NULL, NULL, NULL, 5, 0, K_USEC(delay_us));
}

static struct k_thread clocks_stop_thread_id;
static K_THREAD_STACK_DEFINE(clocks_stop_thread_id_stack, 128);

void clocks_request_stop(uint32_t delay_us)
{
	k_thread_create(&clocks_stop_thread_id, clocks_stop_thread_id_stack, K_THREAD_STACK_SIZEOF(clocks_stop_thread_id), (k_thread_entry_t)clocks_stop, NULL, NULL, NULL, 5, 0, K_USEC(delay_us));
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

	uint16_t jitter = (rand() % 200) - 100;  // ±100 µs
	uint16_t retransmit_delay_with_jitter = RADIO_RETRANSMIT_DELAY + jitter;

	if (tx)
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		// config.mode = ESB_MODE_PTX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = retransmit_delay_with_jitter;
		config.retransmit_count = 3;
		//config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;
		config.selective_auto_ack = true; // TODO: while pairing, should be set to false
		config.use_fast_ramp_up = true;
	}
	else
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
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
		config.use_fast_ramp_up = true;
	}

	err = esb_init(&config);

	if (!err)
		esb_set_rf_channel(RADIO_RF_CHANNEL);

	if (!err)
		esb_set_base_address_0(base_addr_0);

	if (!err)
		esb_set_base_address_1(base_addr_1);

	if (!err)
		esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

	if (err)
	{
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
	if (esb_initialized)
	{
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
	for (int i = 0; i < 4; i++)
	{
		addr_buffer[i] = paired_addr[i + 2];
		addr_buffer[i + 4] = paired_addr[i + 2] + paired_addr[6];
	}
	for (int i = 0; i < 8; i++)
		addr_buffer[i + 8] = paired_addr[7] + i;
	for (int i = 0; i < 16; i++)
	{
		if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55 || addr_buffer[i] == 0xAA) // Avoid invalid addresses (see nrf datasheet)
			addr_buffer[i] += 8;
	}
	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
}

static int esb_send_pair_step(uint8_t step)
{
	tx_payload_pair.data[1] = step;
	int err = esb_write_payload(&tx_payload_pair);
	if (err == -ENOSPC)
	{
		esb_flush_tx();
		err = esb_write_payload(&tx_payload_pair);
	}
	if (err)
	{
		LOG_ERR("Failed to queue pairing burst step %u: %d", step, err);
		return err;
	}
	err = esb_start_tx();
	if (err == -EBUSY)
	{
		LOG_DBG("Pairing burst step %u already pending", step);
		err = 0;
	}
	else if (err)
	{
		LOG_ERR("Failed to start pairing burst step %u: %d", step, err);
	}
	return err;
}

void esb_set_pair(uint64_t addr)
{
	uint64_t *device_addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	uint8_t buf[6] = {0};
	memcpy(buf, device_addr, 6);
	uint8_t checksum = crc8_ccitt(0x07, buf, 6);
	if (checksum == 0)
		checksum = 8;
	if ((addr & 0xFF) != checksum)
	{
		LOG_INF("Incorrect checksum");
		return;
	}
	esb_reset_pair();
	memcpy(paired_addr, &addr, sizeof(paired_addr));
	LOG_INF("Paired");
	sys_write(PAIRED_ID, retained->paired_addr, paired_addr, sizeof(paired_addr)); // Write new address and tracker id
}

void esb_pair(void)
{
	// Reset heartbeat state when starting pairing
	heartbeat_failures = 0;
	set_status(SYS_STATUS_CONNECTION_ERROR, false);
	if (!paired_addr[0]) // zero, no receiver paired
	{
		LOG_INF("Pairing");
		esb_set_addr_discovery();
		esb_initialize(true);
//		timer_init(); // TODO: shouldn't be here!!!
		tx_payload_pair.noack = false;
		uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
		memcpy(&tx_payload_pair.data[2], addr, 6);
		LOG_INF("Device address: %012llX", *addr & 0xFFFFFFFFFFFF);
		uint8_t checksum = crc8_ccitt(0x07, &tx_payload_pair.data[2], 6);
		if (checksum == 0)
			checksum = 8;
		LOG_INF("Checksum: %02X", checksum);
		tx_payload_pair.data[0] = checksum; // Use checksum to make sure packet is for this device
		set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
		while (paired_addr[0] != checksum)
		{
			if (!esb_initialized)
			{
				esb_set_addr_discovery();
				esb_initialize(true);
			}
			if (!clock_status)
				clocks_start();
			if (paired_addr[0])
			{
				LOG_INF("Incorrect checksum: %02X", paired_addr[0]);
				paired_addr[0] = 0; // Packet not for this device
			}
			esb_flush_rx();
			esb_flush_tx();
			if (esb_send_pair_step(0))
			{
				k_msleep(100);
				continue;
			}
			k_msleep(2);
			if (esb_send_pair_step(1))
			{
				k_msleep(100);
				continue;
			}
			k_msleep(2);
			esb_send_pair_step(2); // "acknowledge" pairing from receiver
			k_msleep(996);
		}
		set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_CONNECTION);
		LOG_INF("Paired");
		sys_write(PAIRED_ID, retained->paired_addr, paired_addr, sizeof(paired_addr)); // Write new address and tracker id
		esb_deinitialize();
		k_msleep(1600); // wait for led pattern
	}
	LOG_INF("Tracker ID: %u", paired_addr[1]);
	LOG_INF("Receiver address: %012llX", (*(uint64_t *)&retained->paired_addr[0] >> 16) & 0xFFFFFFFFFFFF);

	connection_set_id(paired_addr[1]);

	esb_set_addr_paired();
	esb_paired = true;
	clocks_stop();
}

void esb_reset_pair(void)
{
	if (paired_addr[0] || esb_paired)
	{
		esb_deinitialize(); // make sure esb is off
		esb_paired = false;
		memset(paired_addr, 0, sizeof(paired_addr));
		LOG_INF("Pairing requested");
	}
}

void esb_clear_pair(void)
{
	esb_reset_pair();
	sys_write(PAIRED_ID, &retained->paired_addr, paired_addr, sizeof(paired_addr)); // write zeroes
	LOG_INF("Pairing data reset");
}

void esb_write(uint8_t *data)
{
	if (!esb_initialized || !esb_paired)
		return;
	if (!clock_status)
		clocks_start();
#if defined(NRF54L15_XXAA) // TODO: esb halts with ack and tx fail
	tx_payload.noack = true;
#else
	tx_payload.noack = true;
#endif
	memcpy(tx_payload.data, data, tx_payload.length);

	int queue_status = esb_write_payload(&tx_payload); // Add transmission to queue
	int retry_count = 0;
	while (queue_status != 0) // TX buffer full
	{
		if (retry_count >= 5) {
			esb_flush_tx();
		}
		esb_pop_tx();
		queue_status = esb_write_payload(&tx_payload);
		retry_count++;
	}
	send_data = true;
}

bool esb_ready(void)
{
	return esb_initialized && esb_paired;
}

static void esb_send_heartbeat(void)
{
	if (!esb_initialized || !esb_paired)
		return;

	// Don't send if there's already a heartbeat pending
	if (heartbeat_pending)
	{
		LOG_DBG("Heartbeat already pending, skipping");
		return;
	}

	if (!clock_status)
		clocks_start();

	// Send heartbeat with ACK to detect connection status
	tx_heartbeat.noack = false;
	tx_heartbeat.length = 1;
	tx_heartbeat.data[0] = 0xFF; // Heartbeat marker

	LOG_DBG("Sending heartbeat (failures: %d/%d) noack=%d", heartbeat_failures, TX_ERROR_THRESHOLD, tx_heartbeat.noack);
	last_heartbeat_time = k_uptime_get();
	heartbeat_pending = true;  // Set flag to indicate heartbeat is being sent

	// Flush TX queue to ensure heartbeat is sent immediately
	esb_pop_tx();
	int queue_status = esb_write_payload(&tx_heartbeat);
	if (queue_status != 0) {
		LOG_ERR("Failed to queue heartbeat: %d", queue_status);
		heartbeat_pending = false;
	} else {
		// Force immediate transmission
		esb_start_tx();
	}
}

static void esb_thread(void)
{
#if CONFIG_CONNECTION_OVER_HID
	int64_t start_time = k_uptime_get();
#endif

	// Read paired address from retained
	memcpy(paired_addr, retained->paired_addr, sizeof(paired_addr));

	while (1)
	{
#if CONFIG_CONNECTION_OVER_HID
		if (!esb_paired && get_status(SYS_STATUS_USB_CONNECTED) == false && k_uptime_get() - 750 > start_time) // only automatically enter pairing while not potentially communicating by usb
#else
		if (!esb_paired)
#endif
		{
			esb_pair();
			esb_initialize(true);
		}
		// Check for shutdown timeout if connection errors persist
		if (heartbeat_failures >= TX_ERROR_THRESHOLD)
		{
#if CONFIG_CONNECTION_OVER_HID
			if (get_status(SYS_STATUS_CONNECTION_ERROR) == false && get_status(SYS_STATUS_USB_CONNECTED) == false) // only raise error while not potentially communicating by usb
#else
			if (get_status(SYS_STATUS_CONNECTION_ERROR) == false)
#endif
				set_status(SYS_STATUS_CONNECTION_ERROR, true);
#if USER_SHUTDOWN_ENABLED
			if (!shutdown_requested && connection_error_start_time > 0 &&
			    k_uptime_get() - connection_error_start_time > CONFIG_CONNECTION_TIMEOUT_DELAY) // shutdown if receiver is not detected
			{
				LOG_WRN("No response from receiver in %dm", CONFIG_CONNECTION_TIMEOUT_DELAY / 60000);
				shutdown_requested = true;
				sys_request_system_off(false);
			}
#endif
		}
		// Send periodic heartbeat to detect connection status
		int64_t now = k_uptime_get();
		int64_t time_since_last_heartbeat = now - last_heartbeat_time;

		if (time_since_last_heartbeat > HEARTBEAT_INTERVAL_MS)
		{
			esb_send_heartbeat();
		}
		k_msleep(100);
	}
}
