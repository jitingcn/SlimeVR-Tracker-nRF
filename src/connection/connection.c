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

static uint8_t tracker_id, batt, batt_v, sensor_temp, imu_id, mag_id, tracker_status;
static uint8_t tracker_svr_status = SVR_STATUS_OK;
static float sensor_q[4], sensor_a[3], sensor_m[3];

LOG_MODULE_REGISTER(connection, LOG_LEVEL_INF);

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

void connection_update_sensor_data(float *q, float *a)
{
	memcpy(sensor_q, q, sizeof(sensor_q));
	memcpy(sensor_a, a, sizeof(sensor_a));
}

void connection_update_sensor_mag(float *m)
{
	memcpy(sensor_m, m, sizeof(sensor_m));
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
	esb_write(data);
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
	esb_write(data);
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
	esb_write(data);
}

void connection_write_packet_3() // status
{
	uint8_t data[16] = {0};
	data[0] = 3; // packet 3
	data[1] = tracker_id;
	data[2] = tracker_svr_status;
	data[3] = tracker_status;
	data[15] = 0; // rssi (supplied by receiver)
	esb_write(data);
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
	esb_write(data);
}

#include <zephyr/kernel.h>

#define MAX_LOGGED_PACKETS 500

static uint32_t p_id = 0;
static uint32_t logged_packets = 0;

static uint32_t logged_id[MAX_LOGGED_PACKETS] = {0};
static int64_t logged_time[MAX_LOGGED_PACKETS] = {0};

static float logged_mag[MAX_LOGGED_PACKETS] = {0};
static float logged_q[MAX_LOGGED_PACKETS][4] = {0};
static float logged_cur_q[MAX_LOGGED_PACKETS][4] = {0};
static float logged_last_q[MAX_LOGGED_PACKETS][4] = {0};
static uint8_t logged_packet_type[MAX_LOGGED_PACKETS] = {0};
static uint8_t logged_cur_p[MAX_LOGGED_PACKETS] = {0};
static uint8_t logged_last_p[MAX_LOGGED_PACKETS] = {0};
static uint32_t logged_q_buf[MAX_LOGGED_PACKETS] = {0};
static uint32_t logged_cur_v[MAX_LOGGED_PACKETS] = {0};
static uint32_t logged_last_v[MAX_LOGGED_PACKETS] = {0};

// last valid data
static float last_q[4] = {0};
static uint32_t last_v = 0;
static uint8_t last_p = 0;
static int last_valid = 0;
// last sent data
static float cur_q[4] = {0};
static uint32_t cur_v = 0;
static uint8_t cur_p = 0;

static void connection_log_error(float mag, float *q, uint8_t packet_type, uint32_t q_buf)
{
	if (logged_packets >= MAX_LOGGED_PACKETS)
	{
		logged_packets++;
		LOG_WRN("Too many logged packets");
		return;
	}

	logged_id[logged_packets] = p_id++;
	logged_time[logged_packets] = k_uptime_get();
	logged_mag[logged_packets] = mag;
	memcpy(logged_q[logged_packets], q, sizeof(logged_q[0]));
	memcpy(logged_cur_q[logged_packets], cur_q, sizeof(logged_cur_q[0]));
	memcpy(logged_last_q[logged_packets], last_q, sizeof(logged_last_q[0]));
	logged_packet_type[logged_packets] = packet_type;
	logged_cur_p[logged_packets] = cur_p;
	logged_last_p[logged_packets] = last_p;
	logged_q_buf[logged_packets] = q_buf;
	logged_cur_v[logged_packets] = cur_v;
	logged_last_v[logged_packets] = last_v;

	logged_packets++;
}

void connection_check_packet(uint8_t *data)
{
	// discard packets with abnormal rotation // TODO:
	if (data[0] == 1 || data[0] == 2 || data[0] == 4)
	{
		float v[3] = {0};
		float q[4] = {0};
		int16_t *buf = (int16_t *)&data[2];
		uint32_t *q_buf = (uint32_t *)&data[5];
		if (data[0] == 1 || data[0] == 4)
		{
			q[0] = FIXED_15_TO_DOUBLE(buf[3]);
			q[1] = FIXED_15_TO_DOUBLE(buf[0]);
			q[2] = FIXED_15_TO_DOUBLE(buf[1]);
			q[3] = FIXED_15_TO_DOUBLE(buf[2]);
		}
		else
		{
			v[0] = FIXED_10_TO_DOUBLE(*q_buf & 1023);
			v[1] = FIXED_11_TO_DOUBLE((*q_buf >> 10) & 2047);
			v[2] = FIXED_11_TO_DOUBLE((*q_buf >> 21) & 2047);
			for (int i = 0; i < 3; i++)
				v[i] = v[i] * 2 - 1;
			q_iem(v, q);
		}
		float mag = q_diff_mag(q, last_q); // difference between last valid
		float mag_cur = q_diff_mag(q, cur_q); // difference between last received
		bool mag_invalid = mag > 0.5f && mag < 6.28f - 0.5f; // possibly invalid rotation
		bool mag_cur_invalid = mag_cur > 0.5f && mag_cur < 6.28f - 0.5f; // possibly inconsistent rotation
		if (mag_invalid)
		{
			// HWID, ID, packet type, rotation difference (rad), last valid packet
			LOG_ERR("Abnormal rot. i%d p%d m%.2f/%.2f v%d", data[1], data[0], (double)mag, (double)mag_cur, last_valid);
			// decoded quat, packet type, q_buf
			printk("a: %5.2f %5.2f %5.2f %5.2f p%d:%08X\n", (double)q[0], (double)q[1], (double)q[2], (double)q[3], data[0], *q_buf);
			printk("b: %5.2f %5.2f %5.2f %5.2f p%d:%08X\n", (double)cur_q[0], (double)cur_q[1], (double)cur_q[2], (double)cur_q[3], cur_p, cur_v);
			printk("c: %5.2f %5.2f %5.2f %5.2f p%d:%08X\n", (double)last_q[0], (double)last_q[1], (double)last_q[2], (double)last_q[3], last_p, last_v);
			last_valid++;
			memcpy(cur_q, q, sizeof(q));
			cur_v = *q_buf;
			cur_p = data[0];
			if (!mag_cur_invalid && last_valid > 2) // reset last_q
			{
				connection_log_error(mag, q, data[0], *q_buf);
				LOG_WRN("Reset rotation, ID %d", data[1]);
				last_valid = 0;
				memcpy(last_q, q, sizeof(q));
				last_v = *q_buf;
				last_p = data[0];
				return;
			}
			return;
		}
		last_valid = 0;
		memcpy(cur_q, q, sizeof(q));
		cur_v = *q_buf;
		cur_p = data[0];
		memcpy(last_q, q, sizeof(q));
		last_v = *q_buf;
		last_p = data[0];
	}
	p_id++;
}

void connection_get_errors(void)
{
	if (logged_packets > MAX_LOGGED_PACKETS)
	{
		LOG_WRN("Too many logged packets, not showing all");
		logged_packets = MAX_LOGGED_PACKETS;
	}
	for (uint32_t i = 0; i < logged_packets; i++)
	{
		uint32_t id = logged_id[i];
		int64_t t = logged_time[i];
		float mag = logged_mag[i];
		float *q = logged_q[i];
		float *cur_q = logged_cur_q[i];
		float *last_q = logged_last_q[i];
		uint8_t packet_type = logged_packet_type[i];
		uint32_t q_buf = logged_q_buf[i];
		uint8_t cur_p = logged_cur_p[i];
		uint8_t last_p = logged_last_p[i];
		uint32_t cur_v = logged_cur_v[i];
		uint32_t last_v = logged_last_v[i];
		printk("i%d t%lld m%.2f; ", id, t, (double)mag);
		printk("c: %5.2f %5.2f %5.2f %5.2f p%d:%08X; ", (double)q[0], (double)q[1], (double)q[2], (double)q[3], packet_type, q_buf);
		printk("l: %5.2f %5.2f %5.2f %5.2f p%d:%08X; ", (double)cur_q[0], (double)cur_q[1], (double)cur_q[2], (double)cur_q[3], cur_p, cur_v);
		printk("v: %5.2f %5.2f %5.2f %5.2f p%d:%08X\n", (double)last_q[0], (double)last_q[1], (double)last_q[2], (double)last_q[3], last_p, last_v);
		k_msleep(10); // give some time to the console
	}
}
