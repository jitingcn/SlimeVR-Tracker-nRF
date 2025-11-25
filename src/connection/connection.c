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
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "build_defines.h"
#include "esb.h"
#include "globals.h"
#include "hid.h"
#include "util.h"

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

static void write_packet_data(const uint8_t *data)
{
	atomic_val_t current_write = atomic_get(&write_idx);
	atomic_val_t current_read = atomic_get(&read_idx);

	atomic_val_t next_write = (current_write + 1) % PACKET_BUFFER_SIZE;
	if (next_write == current_read) {
		LOG_WRN("Packet buffer full, dropping packet");
		return;
	}

	memcpy(packet_buffer[current_write], data, 16);

	atomic_set(&write_idx, next_write);
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
	if (temp < -38.5f) {
		sensor_temp = 1;
	} else if (temp > 88.5f) {
		sensor_temp = 255;
	} else {
		sensor_temp = ((temp - 25) * 2 + 128.5f); // -38.5 - +88.5 -> 1-255
	}
}

// format for packet send
void connection_update_battery(bool battery_available, bool plugged, uint32_t battery_pptt, int battery_mV)
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

	if (plugged) { // Charging
		battery_mV = MAX(battery_mV, 4310); // server will show a charging indicator
	}

	battery_mV /= 10;
	battery_mV -= 245;
	if (battery_mV < 0) { // Very dead but it is what it is
		batt_v = 0;
	} else if (battery_mV > 255) {
		batt_v = 255;
	} else {
		batt_v = battery_mV; // 0-255 -> 2.45-5.00V
	}
}

void connection_update_status(int status)
{
	tracker_status = status;
	tracker_svr_status = get_server_constant_tracker_status(status);
}

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9
//|b10     |b11     |b12     |b13     |b14     |b15     | |type    |id      |packet data
//| |0       |id      |batt    |batt_v  |temp    |brd_id  |mcu_id  |resv    |imu_id
//|mag_id  |fw_date          |major   |minor   |patch   |rssi    | |1       |id      |q0
//|q1               |q2               |q3               |a0               |a1 |a2 | |2
//|id      |batt    |batt_v  |temp    |q_buf                              |a0 |a1 |a2
//|rssi    | |3	   |id      |svr_stat|status  |resv |rssi    |

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

	write_packet_data(data);
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

	write_packet_data(data);
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_2() // reduced precision quat and accel with battery,
								 // temp, and rssi
{
	uint8_t data[16] = {0};
	data[0] = 2; // packet 2
	data[1] = tracker_id;
	data[2] = batt;
	data[3] = batt_v;
	data[4] = sensor_temp; // temp
	float v[3] = {0};
	q_fem(sensor_q, v); // exponential map
	for (int i = 0; i < 3; i++) {
		v[i] = (v[i] + 1) / 2; // map -1-1 to 0-1
	}
	uint16_t v_buf[3]
		= {SATURATE_UINT10((1 << 10) * v[0]),
		   SATURATE_UINT11((1 << 11) * v[1]),
		   SATURATE_UINT11((1 << 11) * v[2])}; // fill 32 bits
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

	write_packet_data(data);
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

	write_packet_data(data);
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

	write_packet_data(data);
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
#define SENSOR_QUAT_INTERVAL_MS 7

void connection_thread(void)
{
	uint8_t esb_packet[17];
	// Adaptive PING interval based on connection state
	// TODO: checking for connection_update events from sensor_loop, here we will time and send them out
	while (1) {
		int64_t now = k_uptime_get();

		// Adjust PING interval based on connection health
		if (get_status(SYS_STATUS_CONNECTION_ERROR)) {
			// During connection errors, slow down PING to every 2.5 seconds
			ping_interval_ms = 2500;
		} else {
			// Normal operation - default interval
			ping_interval_ms = PING_INTERVAL_MS;
		}

		if (!esb_ready()) {
			k_msleep(100);
			continue;
		}

		// PING has highest priority
		// This ensures connection recovery attempts continue even during errors
		bool should_send_ping = false;
		uint32_t server_time = esb_get_server_time();

		if (server_time > 0) {
			// TDMA scheduling: 10 trackers, 100ms slot each in 1000ms period
			uint32_t slot_offset = (tracker_id % 10) * 100; // ms
			uint32_t current_slot = server_time % 1000;

			// Calculate slot difference with wrap-around handling
			int32_t slot_diff = (int32_t)current_slot - (int32_t)slot_offset;
			if (slot_diff < 0) {
				slot_diff += 1000;
			}

			// Send if within slot window (+5ms) and minimum interval elapsed
			if (slot_diff >= 0 && slot_diff <= 5 && now - last_ping_time >= (ping_interval_ms - 100)) {
				should_send_ping = true;
			}
		} else {
			// Fallback: not synced yet, use original time-based scheduling
			if (now - last_ping_time >= ping_interval_ms) {
				should_send_ping = true;
			}
		}

		if (should_send_ping) {
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
			continue;
		}

		// skip sensor data if connection error		// skip sensor data if connection
		// error
		if (get_status(SYS_STATUS_CONNECTION_ERROR)) {
			k_msleep(100);
			continue;
		}

		atomic_val_t current_read = atomic_get(&read_idx);
		atomic_val_t current_write = atomic_get(&write_idx);

		if (current_read != current_write) {
			memcpy(esb_packet, packet_buffer[current_read], 16);
			esb_packet[16] = packet_sequence++;

			atomic_set(&read_idx, (current_read + 1) % PACKET_BUFFER_SIZE);

			esb_write(esb_packet, no_ack, sizeof(esb_packet)); // normal data: no ACK
		}
		// mag is higher priority (skip accel, quat is full precision)
#ifdef CONFIG_SENSOR_USE_MAG
		else if (mag_update_time && now - last_mag_time > 200) {
			mag_update_time = 0; // data has been sent
			last_mag_time = now;
			connection_write_packet_4();
			continue;
		}
#endif
		// if time for info and precise quat not needed
		else if (quat_update_time && !send_precise_quat && now - last_info_time > 100) {
			if (now - last_sensor_quat_time >= SENSOR_QUAT_INTERVAL_MS) {
				quat_update_time = 0;
				last_quat_time = now;
				last_sensor_quat_time = now;
				last_info_time = now;
				connection_write_packet_2();
				continue;
			}
		}
		// send quat otherwise
		else if (quat_update_time) {
			if (now - last_sensor_quat_time >= SENSOR_QUAT_INTERVAL_MS) {
				quat_update_time = 0;
				last_quat_time = now;
				last_sensor_quat_time = now;
				connection_write_packet_1();
				continue;
			}
		} else if (now - last_info_time > 100) {
			last_info_time = now;
			connection_write_packet_0();
			continue;
		} else if (now - last_status_time > 1000) {
			last_status_time = now;
			connection_write_packet_3();
			continue;
		} else {
			connection_clocks_request_stop();
		}
		k_msleep(1);
	}
}
