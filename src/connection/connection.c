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
#include "zephyr/sys/time_units.h"
// #include "hid.h"

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

static uint8_t tracker_id, batt, batt_v, sensor_temp, imu_id, mag_id, tracker_status;
static uint8_t tracker_svr_status = SVR_STATUS_OK;
static float sensor_q[4], sensor_a[3], sensor_m[3];

// Packet sequence number for data packets
static uint8_t packet_sequence = 0;

LOG_MODULE_REGISTER(connection, LOG_LEVEL_INF);

#ifndef CONFIG_CONNECTION_ENABLE_ACK
static bool no_ack = true;
#else
static bool no_ack = false;
#endif

typedef enum {
	SLOT_TYPE_PING,   // Sync frame (1Hz, distributed)
	SLOT_TYPE_STATUS, // Status frame (1Hz)
	SLOT_TYPE_INFO,   // Device info (10Hz)
	SLOT_TYPE_MAG,    // Magnetometer (5Hz, if enabled)
	SLOT_TYPE_QUAT    // Quaternion data (fills remaining slots)
} slot_type_t;

// Schedule tick counter
static uint32_t schedule_tick = 0;

// Get slot type for current tick
static slot_type_t get_slot_type(uint32_t tick, uint8_t tracker_id)
{
	// PING: Each tracker at different time (0ms, 100ms, 200ms, ...)
	// Distributed across 1 second to avoid receiver overload
	uint32_t ping_interval = TDMA_PACKETS_PER_SECOND / 10;
	uint32_t is_ping_tick = (tracker_id * ping_interval) % TDMA_PACKETS_PER_SECOND;
	if (tick == is_ping_tick) {
		return SLOT_TYPE_PING;
	}

	// Status: 500ms after PING
	uint32_t is_status_tick = (is_ping_tick + (TDMA_PACKETS_PER_SECOND / 2)) % TDMA_PACKETS_PER_SECOND;
	if (tick == is_status_tick) {
		return SLOT_TYPE_STATUS;
	}

	// Info: 10Hz
	uint32_t info_interval = TDMA_PACKETS_PER_SECOND / 10;
	if (tick % info_interval == (info_interval / 2)) {
		return SLOT_TYPE_INFO;
	}

#ifdef CONFIG_SENSOR_USE_MAG
	// Mag: 5Hz
	uint32_t mag_interval = TDMA_PACKETS_PER_SECOND / 5;
	if (tick % mag_interval == 1) {
		return SLOT_TYPE_MAG;
	}
#endif

	// Quat: Fill remaining slots
	return SLOT_TYPE_QUAT;
}

// Slot-based scheduling state
static int64_t last_slot_time = 0;                // Last time we processed a slot
static uint32_t last_processed_tick = UINT32_MAX; // Last slot tick processed (UINT32_MAX = no slot processed yet)

// Get current slot tick based on sync status
// Returns: slot tick (0 to TDMA_PACKETS_PER_SECOND-1)
static uint32_t get_current_slot_tick(void)
{
	uint32_t server_time = esb_get_server_time();

	if (server_time > 0) {
		// Synchronized: use server time based TDMA
		uint64_t server_time_us = esb_get_server_time_us_64();
		uint32_t slot_in_second = (server_time_us / TDMA_PACKET_INTERVAL_US) % TDMA_PACKETS_PER_SECOND;
		return slot_in_second;
	} else {
		// Not synchronized: use schedule_tick
		return schedule_tick % TDMA_PACKETS_PER_SECOND;
	}
}

// Sleep until next slot starts
// Returns: time slept (0 if already in slot window)
static uint32_t sleep_until_next_slot()
{
	uint32_t server_time = esb_get_server_time();

	if (server_time > 0) {
		// Synchronized: calculate sleep time to next slot
		uint64_t server_time_us = esb_get_server_time_us_64();
		uint32_t current_slot_us = server_time_us % TDMA_PACKET_INTERVAL_US;

		// Always sleep until the NEXT slot starts
		// If we're in the current slot, wait for it to end
		uint32_t sleep_us = TDMA_PACKET_INTERVAL_US - current_slot_us + TDMA_GUARD_TIME_US;
		if (sleep_us < 300) {
			k_busy_wait(sleep_us);
		} else {
			k_usleep(sleep_us);
		}
		return sleep_us;
	} else {
		// Not synchronized: use schedule_tick with fixed interval
		k_usleep(600);
		return 600;
	}
}

uint32_t get_ping_interval_ms(void)
{
	return PING_INTERVAL_MS;
}

static void connection_thread(void);
K_THREAD_DEFINE(connection_thread_id, 1024, connection_thread, NULL, NULL, NULL, 5, 0, 0);

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

void connection_update_sensor_ids(int imu, int mag)
{
	imu_id = get_server_constant_imu_id(imu);
	mag_id = get_server_constant_mag_id(mag);
}

static int64_t quat_update_time = 0;
static bool send_precise_quat;

void connection_update_sensor_data(float *q, float *a, int64_t data_time)
{
	// data_time is in system ticks, nonzero means valid measurement
	// TODO: use data_time to measure latency! the latency should be calculated up to before radio sent data
	send_precise_quat = q_epsilon(q, sensor_q, 0.005f);
	memcpy(sensor_q, q, sizeof(sensor_q));
	memcpy(sensor_a, a, sizeof(sensor_a));
	quat_update_time = k_uptime_get();
}

static int64_t mag_update_time = 0;

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

	if (plugged) {                          // Charging
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
	uint8_t data[17] = {0};
	data[0] = 0; // packet 0
	data[1] = tracker_id;
	data[2] = batt;
	data[3] = batt_v;
	data[4] = sensor_temp; // temp
	data[5] = FW_BOARD;    // brd_id
	data[6] = FW_MCU;      // mcu_id
	data[7] = 0;           // resv
	data[8] = imu_id;      // imu_id
	data[9] = mag_id;      // mag_id
	uint16_t *buf = (uint16_t *)&data[10];
	buf[0] = ((BUILD_YEAR - 2020) & 127) << 9 | (BUILD_MONTH & 15) << 5 | (BUILD_DAY & 31); // fw_date
	data[12] = FW_VERSION_MAJOR & 255;                                                      // fw_major
	data[13] = FW_VERSION_MINOR & 255;                                                      // fw_minor
	data[14] = FW_VERSION_PATCH & 255;                                                      // fw_patch
	data[15] = 0;                 // rssi (supplied by receiver)
	data[16] = packet_sequence++; // sequence number

	esb_write(data, no_ack, sizeof(data));
	// hid_write_packet_n(data); // TODO:
}

void connection_write_packet_1() // full precision quat and accel
{
	uint8_t data[17] = {0};
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
	data[16] = packet_sequence++; // sequence number

	esb_write(data, no_ack, sizeof(data));
	// hid_write_packet_n(data); // TODO:
}

void connection_write_packet_2() // reduced precision quat and accel with battery,
								 // temp, and rssi
{
	uint8_t data[17] = {0};
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
	data[15] = 0;                 // rssi (supplied by receiver)
	data[16] = packet_sequence++; // sequence number

	esb_write(data, no_ack, sizeof(data));
	// hid_write_packet_n(data); // TODO:
}

void connection_write_packet_3() // status
{
	uint8_t data[17] = {0};
	data[0] = 3; // packet 3
	data[1] = tracker_id;
	data[2] = tracker_svr_status;
	data[3] = tracker_status;
	data[15] = 0;                 // rssi (supplied by receiver)
	data[16] = packet_sequence++; // sequence number

	esb_write(data, no_ack, sizeof(data));
	// hid_write_packet_n(data); // TODO:
}

void connection_write_packet_4() // full precision quat and magnetometer
{
	uint8_t data[17] = {0};
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
	data[16] = packet_sequence++; // sequence number

	esb_write(data, no_ack, sizeof(data));
	// hid_write_packet_n(data); // TODO:
}

// TODO: get radio channel from receiver
// TODO: new packet format

// TODO: use timing from IMU to get actual delay in tracking
// TODO: aware of sensor state? error status, timing/phase, maybe "send_precise_quat"

// TODO: queuing, status is lowest priority, info low priority, existing data highest priority (from sensor loop)

// TODO: queue packets directly for HID, or maintain separate loop while connected by USB
static int64_t last_ping_time = 0;

void connection_thread(void)
{
	// Slot-based scheduling
	while (1) {
		int64_t now = k_uptime_get();

		// Wait for ESB ready
		if (!esb_ready()) {
			k_msleep(100);
			continue;
		}

		// Skip sensor data if connection error
		if (get_status(SYS_STATUS_CONNECTION_ERROR)) {
			// During connection errors, try to send PING at reduced rate
			uint32_t server_time = esb_get_server_time();
			if (server_time == 0) {
				// Not synced, use time-based PING every 2.5s
				if (now - last_slot_time >= 2500) {
					uint8_t ping[ESB_PING_LEN] = {0};
					ping[0] = ESB_PING_TYPE;
					ping[1] = connection_get_id();
					ping[2] = 0;
					memset(&ping[3], 0x00, 4);
					ping[7] = esb_get_ping_ack_flag();
					memset(&ping[8], 0x00, 4);
					ping[ESB_PING_LEN - 1] = 0;
					esb_write(ping, false, ESB_PING_LEN);
					last_slot_time = now;
					last_ping_time = k_uptime_get();
				}
			}
			k_msleep(100);
			continue;
		}

		// Get current slot and determine packet type
		uint32_t current_tick = get_current_slot_tick();

		// Skip if we already processed this slot
		if (current_tick == last_processed_tick) {
			k_usleep(300);
			continue;
		}

		now = k_uptime_get();
		slot_type_t slot = get_slot_type(current_tick, tracker_id);

		// Process slot based on type
		bool packet_sent = false;
		switch (slot) {
		case SLOT_TYPE_PING: {
			if ((now - last_ping_time) > (PING_INTERVAL_MS - 100)) {
				last_ping_time = k_uptime_get();
				uint8_t ping[ESB_PING_LEN] = {0};
				ping[0] = ESB_PING_TYPE;
				ping[1] = connection_get_id();
				ping[2] = 0;               // ping counter, set in esb_write
				memset(&ping[3], 0x00, 4); // reserved
				ping[7] = esb_get_ping_ack_flag();
				memset(&ping[8], 0x00, 4);  // reserved
				ping[ESB_PING_LEN - 1] = 0; // crc bit, set in esb_write
				esb_write(ping, false, ESB_PING_LEN);
				packet_sent = true;
				break;
			}
		}

		case SLOT_TYPE_STATUS:
			connection_write_packet_3();
			packet_sent = true;
			break;

		case SLOT_TYPE_INFO:
			connection_write_packet_0();
			packet_sent = true;
			break;

		case SLOT_TYPE_MAG:
#ifdef CONFIG_SENSOR_USE_MAG
			if (mag_update_time) {
				mag_update_time = 0;
				connection_write_packet_4();
				packet_sent = true;
			} else if (quat_update_time) {
				// No mag data, send quat instead
				quat_update_time = 0;
				if (send_precise_quat) {
					connection_write_packet_1();
				} else {
					connection_write_packet_2();
				}
				packet_sent = true;
			}
#endif
			break;

		case SLOT_TYPE_QUAT:
			if (quat_update_time) {
				quat_update_time = 0;
				if (send_precise_quat) {
					connection_write_packet_1();
				} else {
					connection_write_packet_2();
				}
				packet_sent = true;
			}
			break;
		}

		// Update slot state
		last_processed_tick = current_tick;
		last_slot_time = k_uptime_get();
		if (esb_get_server_time() == 0) {
			schedule_tick++;
		}

		sleep_until_next_slot();
	}
}
