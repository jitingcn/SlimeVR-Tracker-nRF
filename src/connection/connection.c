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

#define PACKET_BUFFER_SIZE 4
static uint8_t packet_buffer[PACKET_BUFFER_SIZE][16] = {0};
static atomic_t write_idx = ATOMIC_INIT(0);
static atomic_t read_idx = ATOMIC_INIT(0);
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
K_THREAD_DEFINE(connection_thread_id, 512, connection_thread, NULL, NULL, NULL, 7, 0, 0);

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

// Prepare packet data into the next available buffer slot
// Returns true if packet was prepared, false if buffer full
static bool prepare_packet_data(const uint8_t *data)
{
	// 使用原子操作的循环缓冲区
	atomic_val_t current_write = atomic_get(&write_idx);
	atomic_val_t current_read = atomic_get(&read_idx);

	// 检查缓冲区是否已满
	atomic_val_t next_write = (current_write + 1) % PACKET_BUFFER_SIZE;
	if (next_write == current_read) {
		// Buffer full - this is normal under TDMA rate limiting
		return false;
	}

	// 写入数据到当前写索引位置
	memcpy(packet_buffer[current_write], data, 16);

	// 使用内存屏障确保写入完成后再更新索引
	atomic_set(&write_idx, next_write);
	return true;
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

	prepare_packet_data(data);
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

	prepare_packet_data(data);
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

	prepare_packet_data(data);
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

	prepare_packet_data(data);
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

	prepare_packet_data(data);
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

// TDMA enforcement: track slot usage to ensure only ONE packet per slot
static uint32_t last_tdma_slot_used = 0;
static int64_t last_data_tx_time = 0;
#define MIN_TX_INTERVAL_US 200  // Minimum 400μs between packets (radio TX time)

void connection_thread(void)
{
	uint8_t esb_packet[17];
	// Global TX limiter
	static int64_t last_tx_time = 0;
	static bool tdma_init_done = false;

	// TODO: checking for connection_update events from sensor_loop, here we will time and send them out
	while (1)
	{
		int64_t	now = k_uptime_get();

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

		// Initialize TDMA once we're ready and synced
		if (!tdma_init_done && tdma_is_synced()) {
			tdma_init(tracker_id);
			tdma_init_done = true;
		}

		// PING has highest priority - send every second, using TDMA slot to avoid collision
		// With smart slot allocation to minimize pipe conflicts (trackers sharing same pipe
		// are separated by maximum time to reduce ACK payload collisions)
		bool should_send_ping = false;
		uint32_t server_time = esb_get_server_time();

		// Check if it's time to send PING (every ping_interval_ms, typically 1000ms)
		if (now - last_ping_time >= (ping_interval_ms - 100)) {
			if (server_time > 0) {
				// Some trackers sharing same pipe (e.g., T0 & T7 both use pipe 1)
				// Slot mapping: T0=0ms, T1=100ms, T2=200ms, T3=300ms, T4=400ms, etc
				uint32_t slot_offset_ms;
				slot_offset_ms = tracker_id * 100;
				uint32_t current_ms_in_second = server_time % 1000; // Position in 1s cycle

				// Calculate if we're in our slot window (±10ms tolerance)
				int32_t slot_diff_ms = (int32_t)current_ms_in_second - (int32_t)slot_offset_ms;
				if (slot_diff_ms < 0) slot_diff_ms += 1000;

				// Send if within our slot window (±3ms)
				if (slot_diff_ms >= -3 && slot_diff_ms <= 3) {
					should_send_ping = true;
				}
				// If we missed our slot in this second, wait for next second
			} else {
				// Fallback: not synced yet, use original time-based scheduling
				should_send_ping = true;
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

		// Skip data transmission if we're near any PING slot (±2ms guard zone)
		if (tdma_is_synced() && tdma_is_in_ping_guard_zone(tracker_id)) {
			k_usleep(500);
			continue;
		}

		// ========== TDMA SLOT ENFORCEMENT ==========
		// CRITICAL: Each slot can send ONLY ONE packet to maintain 100 TPS limit
		// We must wait for our slot AND ensure we haven't sent in current slot

		if (tdma_is_synced()) {
			// Wait for our transmission slot
			tdma_wait_for_slot();

			// Verify we're actually in our slot now
			if (!tdma_is_in_tx_slot()) {
				k_busy_wait(50);
				continue;
			}

			// Get current slot number to prevent double-sending in same slot
			uint32_t current_slot = tdma_get_current_slot();
			if (current_slot == last_tdma_slot_used) {
				// Already sent a packet in this slot, wait for next slot
				k_msleep(1);
				continue;
			}

			// Enforce minimum interval between packets (hardware TX time)
			int64_t time_since_last_tx_us = (now - last_data_tx_time) * 1000;
			if (time_since_last_tx_us < MIN_TX_INTERVAL_US) {
				// Too soon since last TX, skip this slot
				k_msleep(1);
				continue;
			}
		}

		// ========== PACKET PRIORITY QUEUE ==========
		// Priority order: buffered packets > mag > quat > info > status
		// Select ONLY ONE packet to send per slot

		bool packet_sent = false;

		// Priority 1: Buffered packets from sensor loop (highest priority)
		atomic_val_t current_read = atomic_get(&read_idx);
		atomic_val_t current_write = atomic_get(&write_idx);

		if (current_read != current_write) {
			// Send buffered packet
			memcpy(esb_packet, packet_buffer[current_read], 16);
			esb_packet[16] = packet_sequence++;
			atomic_set(&read_idx, (current_read + 1) % PACKET_BUFFER_SIZE);

			esb_write(esb_packet, no_ack, sizeof(esb_packet));
			packet_sent = true;
		}
		// Priority 2: Magnetometer data (if enabled and ready)
#ifdef CONFIG_SENSOR_USE_MAG
		else if (mag_update_time && now - last_mag_time > 200) {
			mag_update_time = 0;
			last_mag_time = now;
			connection_write_packet_4();
		}
#endif
		// Priority 3: Quaternion data (main tracking data)
		else if (quat_update_time && now - last_sensor_quat_time >= SENSOR_QUAT_INTERVAL_MS) {
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
		}
		// Priority 4: Device info
		else if (now - last_info_time > 100) {
			last_info_time = now;
			connection_write_packet_0();
		}
		// Priority 5: Status (lowest priority)
		else if (now - last_status_time > 1000) {
			last_status_time = now;
			connection_write_packet_3();
		}

		// Update last TX tracking if packet was sent
		if (packet_sent) {
			if (tdma_is_synced()) {
				last_tdma_slot_used = tdma_get_current_slot();
			}
			last_data_tx_time = now;
			last_tx_time = now;
		} else {
			// No packet to send, can stop clocks
			connection_clocks_request_stop();
		}
		k_msleep(1);
	}
}
