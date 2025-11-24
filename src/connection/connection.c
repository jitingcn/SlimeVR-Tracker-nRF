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
#include "util.h"
#include "esb.h"
#include "build_defines.h"
#include "hid.h"

#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

static uint8_t tracker_id, batt, batt_v, sensor_temp, imu_id, mag_id, tracker_status;
static uint8_t tracker_svr_status = SVR_STATUS_OK;
static float sensor_q[4], sensor_a[3], sensor_m[3];

static uint8_t packet_sequence = 0;
static int64_t last_ping_time = 0;
static uint32_t ping_interval_ms = PING_INTERVAL_MS;

LOG_MODULE_REGISTER(connection, LOG_LEVEL_INF);

#ifndef CONFIG_CONNECTION_ENABLE_ACK
static bool no_ack = true;
#else
static bool no_ack = false;
#endif

uint32_t get_ping_interval_ms(void)
{
	return ping_interval_ms;
}

static void connection_thread(void);
K_THREAD_DEFINE(connection_thread_id, 512, connection_thread, NULL, NULL, NULL, 6, 0, 0);

void connection_clocks_request_start(void)
{
	clocks_request_start(0);
}

void connection_clocks_request_start_delay_us(uint32_t delay_us)
{
	clocks_request_start(delay_us);
}

void connection_clocks_request_stop(void)
{
	clocks_stop();
}

void connection_clocks_request_stop_delay_us(uint32_t delay_us)
{
	clocks_request_stop(delay_us);
}

uint8_t connection_get_id(void)
{
	return tracker_id;
}

void connection_set_id(uint8_t id)
{
	tracker_id = id;
}

uint8_t connection_get_packet_sequence(void)
{
	return packet_sequence;
}

// Helper to send packet (TDMA scheduling done in main loop)
static void send_packet(const uint8_t *data, size_t len) {
	if (!esb_ready()) {
		return;
	}

	// Prepare ESB packet with sequence number
	uint8_t esb_packet[17];
	memcpy(esb_packet, data, len);
	esb_packet[16] = packet_sequence++;

	// Send immediately
	esb_write(esb_packet, no_ack, sizeof(esb_packet));
}

void connection_update_sensor_ids(int imu, int mag)
{
	imu_id = get_server_constant_imu_id(imu);
	mag_id = get_server_constant_mag_id(mag);
}

static int64_t quat_update_time = 0;
static int64_t last_quat_time = 0;
static bool send_precise_quat;

void connection_update_sensor_data(float *q, float *a, int64_t data_time)
{
	// data_time is in system ticks, nonzero means valid measurement
	// TODO: use data_time to measure latency! the latency should be calculated up to before radio sent data
	send_precise_quat = q_epsilon(q, sensor_q, 0.005);
	memcpy(sensor_q, q, sizeof(sensor_q));
	memcpy(sensor_a, a, sizeof(sensor_a));
	quat_update_time = k_uptime_get();
}

static int64_t mag_update_time = 0;
#ifdef CONFIG_SENSOR_USE_MAG
static int64_t last_mag_time = 0;
#endif

void connection_update_sensor_mag(float *m)
{
	memcpy(sensor_m, m, sizeof(sensor_m));
	mag_update_time = k_uptime_get();
}

void connection_update_sensor_temp(float temp)
{
	// sensor_temp == zero means no data
	if (temp < -38.5f)
		sensor_temp = 1;
	else if (temp > 88.5f)
		sensor_temp = 255;
	else
		sensor_temp = ((temp - 25) * 2 + 128.5f); // -38.5 - +88.5 -> 1-255
}

void connection_update_battery(bool battery_available, bool plugged, uint32_t battery_pptt, int battery_mV) // format for packet send
{
	if (!battery_available) // No battery, and voltage is <=1500mV
	{
		batt = 0;
		batt_v = 0;
		return;
	}

	battery_pptt /= 100;
	batt = battery_pptt;
	batt |= 0x80; // battery_available, server will show a battery indicator

	if (plugged) // Charging
		battery_mV = MAX(battery_mV, 4310); // server will show a charging indicator

	battery_mV /= 10;
	battery_mV -= 245;
	if (battery_mV < 0) // Very dead but it is what it is
		batt_v = 0;
	else if (battery_mV > 255)
		batt_v = 255;
	else
		batt_v = battery_mV; // 0-255 -> 2.45-5.00V
}

void connection_update_status(int status)
{
	tracker_status = status;
	tracker_svr_status = get_server_constant_tracker_status(status);
}

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |id      |packet data                                                                                                                  |
//|0       |id      |batt    |batt_v  |temp    |brd_id  |mcu_id  |resv    |imu_id  |mag_id  |fw_date          |major   |minor   |patch   |rssi    |
//|1       |id      |q0               |q1               |q2               |q3               |a0               |a1               |a2               |
//|2       |id      |batt    |batt_v  |temp    |q_buf                              |a0               |a1               |a2               |rssi    |
//|3	   |id      |svr_stat|status  |resv                                                                                              |rssi    |

void connection_write_packet_0() // device info
{
	uint8_t data[16] = {0};
	data[0] = 0; // packet 0
	data[1] = tracker_id;
	data[2] = batt;
	data[3] = batt_v;
	data[4] = sensor_temp; // temp
	data[5] = FW_BOARD; // brd_id
	data[6] = FW_MCU; // mcu_id
	data[7] = 0; // resv
	data[8] = imu_id; // imu_id
	data[9] = mag_id; // mag_id
	uint16_t *buf = (uint16_t *)&data[10];
	buf[0] = ((BUILD_YEAR - 2020) & 127) << 9 | (BUILD_MONTH & 15) << 5 | (BUILD_DAY & 31); // fw_date
	data[12] = FW_VERSION_MAJOR & 255; // fw_major
	data[13] = FW_VERSION_MINOR & 255; // fw_minor
	data[14] = FW_VERSION_PATCH & 255; // fw_patch
	data[15] = 0; // rssi (supplied by receiver)

	send_packet(data, 16);
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_1() // full precision quat and accel
{
	uint8_t data[16] = {0};
	data[0] = 1; // packet 1
	data[1] = tracker_id;
	uint16_t *buf = (uint16_t *)&data[2];
	buf[0] = TO_FIXED_15(sensor_q[1]); // ±1.0
	buf[1] = TO_FIXED_15(sensor_q[2]);
	buf[2] = TO_FIXED_15(sensor_q[3]);
	buf[3] = TO_FIXED_15(sensor_q[0]);
	buf[4] = TO_FIXED_7(sensor_a[0]); // range is ±256m/s² or ±26.1g
	buf[5] = TO_FIXED_7(sensor_a[1]);
	buf[6] = TO_FIXED_7(sensor_a[2]);

	send_packet(data, 16);
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_2() // reduced precision quat and accel with battery, temp, and rssi
{
	uint8_t data[16] = {0};
	data[0] = 2; // packet 2
	data[1] = tracker_id;
	data[2] = batt;
	data[3] = batt_v;
	data[4] = sensor_temp; // temp
	float v[3] = {0};
	q_fem(sensor_q, v); // exponential map
	for (int i = 0; i < 3; i++)
		v[i] = (v[i] + 1) / 2; // map -1-1 to 0-1
	uint16_t v_buf[3] = {SATURATE_UINT10((1 << 10) * v[0]), SATURATE_UINT11((1 << 11) * v[1]), SATURATE_UINT11((1 << 11) * v[2])}; // fill 32 bits
	uint32_t *q_buf = (uint32_t *)&data[5];
	*q_buf = v_buf[0] | (v_buf[1] << 10) | (v_buf[2] << 21);

//	v[0] = FIXED_10_TO_DOUBLE(*q_buf & 1023);
//	v[1] = FIXED_11_TO_DOUBLE((*q_buf >> 10) & 2047);
//	v[2] = FIXED_11_TO_DOUBLE((*q_buf >> 21) & 2047);
//	for (int i = 0; i < 3; i++)
//	v[i] = v[i] * 2 - 1;
//	float q[4] = {0};
//	q_iem(v, q); // inverse exponential map

	uint16_t *buf = (uint16_t *)&data[9];
	buf[0] = TO_FIXED_7(sensor_a[0]);
	buf[1] = TO_FIXED_7(sensor_a[1]);
	buf[2] = TO_FIXED_7(sensor_a[2]);
	data[15] = 0; // rssi (supplied by receiver)

	send_packet(data, 16);
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_3() // status
{
	uint8_t data[16] = {0};
	data[0] = 3; // packet 3
	data[1] = tracker_id;
	data[2] = tracker_svr_status;
	data[3] = tracker_status;
	data[15] = 0; // rssi (supplied by receiver)

	send_packet(data, 16);
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_4() // full precision quat and magnetometer
{
	uint8_t data[16] = {0};
	data[0] = 4; // packet 4
	data[1] = tracker_id;
	uint16_t *buf = (uint16_t *)&data[2];
	buf[0] = TO_FIXED_15(sensor_q[1]);
	buf[1] = TO_FIXED_15(sensor_q[2]);
	buf[2] = TO_FIXED_15(sensor_q[3]);
	buf[3] = TO_FIXED_15(sensor_q[0]);
	buf[4] = TO_FIXED_10(sensor_m[0]); // range is ±32G
	buf[5] = TO_FIXED_10(sensor_m[1]);
	buf[6] = TO_FIXED_10(sensor_m[2]);

	send_packet(data, 16);
	hid_write_packet_n(data); // TODO:
}

// TODO: get radio channel from receiver
// TODO: new packet format

// TODO: use timing from IMU to get actual delay in tracking
// TODO: aware of sensor state? error status, timing/phase, maybe "send_precise_quat"

// TODO: queuing, status is lowest priority, info low priority, existing data highest priority (from sensor loop)

// TODO: queue packets directly for HID, or maintain separate loop while connected by USB

static int64_t last_info_time = 0;
static int64_t last_status_time = 0;

static int64_t last_sensor_quat_time = 0;
#define SENSOR_QUAT_INTERVAL_MS 6

// TDMA statistics
static int64_t last_data_tx_time = 0;

void connection_thread(void)
{
	// Global TX limiter
	static int64_t last_tx_time = 0;
	static bool tdma_init_done = false;
	static int64_t last_sync_check_time = 0;

	// TODO: checking for connection_update events from sensor_loop, here we will time and send them out
	while (1)
	{
		int64_t	now = k_uptime_get();

		// Periodic sync status check (every 5 seconds)
		if (now - last_sync_check_time > 5000) {
			bool is_synced = tdma_is_synced();
			bool time_synced = esb_is_time_synced();
			if (!is_synced) {
				LOG_WRN("TDMA not synced: tdma_init_done=%d, time_synced=%d, tdma_is_synced=%d",
					tdma_init_done, time_synced, is_synced);
				if (tdma_init_done && time_synced && !is_synced) {
					LOG_ERR("CRITICAL: tdma_init_done=1 but tdma not synced - init likely failed!");
					// Try to reinitialize
					tdma_init_done = false;
				}
				// Log recovery stats
				uint32_t ping_interval = get_ping_interval_ms();
				LOG_INF("Recovery mode: ping_interval=%ums, using basic rate limiting", ping_interval);
			}
			last_sync_check_time = now;
		}

		// Adjust PING interval based on connection health
		if (get_status(SYS_STATUS_CONNECTION_ERROR)) {
			// During connection errors, slow down PING to every 2.5 seconds
			// This reduces radio congestion and gives ESB time to recover
			ping_interval_ms = 2500;
		} else {
			// Normal operation - default interval
			ping_interval_ms = PING_INTERVAL_MS;
		}

		if (!esb_ready()) {
			k_msleep(100);
			continue;
		}

		// Initialize TDMA once server time is synced (don't check tdma_is_synced - circular dependency!)
		if (!tdma_init_done && esb_is_time_synced()) {
			LOG_INF("Attempting TDMA initialization for tracker %u...", tracker_id);
			tdma_init(tracker_id);
			// Verify initialization succeeded
			if (tdma_is_synced()) {
				tdma_init_done = true;
				LOG_INF("TDMA initialization successful");
			} else {
				LOG_ERR("TDMA initialization failed - tdma_initialized not set");
			}
		}

		// PING has highest priority - send every second, using TDMA slot to avoid collision
		// With smart slot allocation to minimize pipe conflicts (trackers sharing same pipe
		// are separated by maximum time to reduce ACK payload collisions)
		bool should_send_ping = false;
		uint64_t server_time_us = esb_get_server_time_us_64();
		bool time_synced = esb_is_server_time_synced();

		// Check if it's time to send PING (every ping_interval_ms, typically 1000ms)
		if (now - last_ping_time >= (ping_interval_ms - 10)) {
			if (time_synced && server_time_us > 0) {
				// Slot mapping: T0=0ms, T1=100ms, T2=200ms, etc
				uint32_t slot_offset_us = tracker_id * 100000; // 100ms in microseconds
				uint32_t current_us_in_second = server_time_us % 1000000; // Position in 1s cycle

				// Calculate shortest distance to our slot (handle wrap-around)
				int32_t slot_diff_us = (int32_t)current_us_in_second - (int32_t)slot_offset_us;
				// Normalize to -500ms to +500ms range
				if (slot_diff_us > 500000) {
					slot_diff_us -= 1000000;
				} else if (slot_diff_us < -500000) {
					slot_diff_us += 1000000;
				}

				// Send if within our slot window (4ms tolerance for better reliability)
				if (slot_diff_us >= -1000 && slot_diff_us <= 1000) {
					should_send_ping = true;
				}
			} else {
				// Not synced: send immediately to re-establish sync
				should_send_ping = true;
				if (!time_synced) {
					LOG_WRN("Time sync lost, sending PING to re-sync");
				}
			}
		}

		if (should_send_ping)
		{
			uint8_t ping[ESB_PING_LEN] = {0};
			ping[0] = ESB_PING_TYPE;
			ping[1] = connection_get_id();
			ping[2] = 0; // ping counter, set in esb_write
			uint32_t now32 = (uint32_t)now;
			ping[3] = (now32 >> 24) & 0xFF;
			ping[4] = (now32 >> 16) & 0xFF;
			ping[5] = (now32 >> 8) & 0xFF;
			ping[6] = (now32) & 0xFF;
			ping[7] = esb_get_ping_ack_flag();
			memset(&ping[8], 0x00, 4); // reserved
			ping[12] = 0;

			// Debug: log slot offset for diagnosis
			if (server_time_us > 0) {
				uint32_t slot_offset_us = tracker_id * 100000;
				uint32_t current_us = server_time_us % 1000000;
				int32_t diff = (int32_t)current_us - (int32_t)slot_offset_us;
				if (diff > 500000) diff -= 1000000;
				else if (diff < -500000) diff += 1000000;
				LOG_DBG("PING slot: target=%uus, current=%uus, diff=%dus",
					slot_offset_us, current_us, diff);
			}

			esb_write(ping, false, ESB_PING_LEN);
			last_ping_time = now;
			last_tx_time = now;
			continue;
		}

		// skip sensor data if connection error
		if (get_status(SYS_STATUS_CONNECTION_ERROR)) {
			k_msleep(100);
			continue;
		}

		// ========== TDMA SLOT WAIT ==========
		// Wait for our transmission slot before selecting packet
		// This ensures we only send ONE packet per slot
		if (tdma_is_synced()) {
			tdma_sleep_until_next_slot();
		}

		// ========== PACKET PRIORITY QUEUE ==========
		// Select highest priority packet to send
		// Priority order: mag > quat > info > status

		// Priority 1: Magnetometer data (if enabled and ready)
#ifdef CONFIG_SENSOR_USE_MAG
		if (mag_update_time && now - last_mag_time > 200) {
			mag_update_time = 0;
			last_mag_time = now;
			connection_write_packet_4();
			last_data_tx_time = now;
			last_tx_time = now;
			if (tdma_is_synced()) tdma_mark_slot_used();
			continue;
		}
#endif

		// Priority 2: Quaternion data (main tracking data)
		if (quat_update_time && now - last_sensor_quat_time >= SENSOR_QUAT_INTERVAL_MS) {
			quat_update_time = 0;
			last_quat_time = now;
			last_sensor_quat_time = now;

			// Use packet_2 if it's time for info and precise quat not needed
			if (!send_precise_quat && now - last_info_time > 100) {
				last_info_time = now;
				connection_write_packet_2();
			} else {
				connection_write_packet_1();
			}
			last_data_tx_time = now;
			last_tx_time = now;
			if (tdma_is_synced()) tdma_mark_slot_used();
			continue;
		}

		// Priority 3: Device info
		if (now - last_info_time > 100) {
			last_info_time = now;
			connection_write_packet_0();
			last_data_tx_time = now;
			last_tx_time = now;
			if (tdma_is_synced()) tdma_mark_slot_used();
			continue;
		}

		// Priority 4: Status (lowest priority)
		if (now - last_status_time > 1000) {
			last_status_time = now;
			connection_write_packet_3();
			last_data_tx_time = now;
			last_tx_time = now;
			if (tdma_is_synced()) tdma_mark_slot_used();
			continue;
		}

		// No packet to send, can stop clocks
		connection_clocks_request_stop();
		if (tdma_is_synced()) {
			k_usleep(300);  // Sleep when idle with TDMA
		} else {
			// When not synced, sleep longer to reduce CPU usage and allow PING recovery
			k_msleep(3);
		}
	}
}
