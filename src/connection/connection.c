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
#include "sensor/fusion/vqf/vqf.h"
#include "sensor/sensor.h"
#include "connection.h"
#include "util.h"
#include "esb.h"
#include "tdma.h"
#include "build_defines.h"
#include "hid.h"
#include "system/battery_tracker.h"
#include "system/watchdog.h"

#include <stdbool.h>
#include <zephyr/kernel.h>

static uint8_t tracker_id, batt, batt_v, sensor_temp, imu_id, mag_id, tracker_status;
static uint8_t tracker_svr_status = SVR_STATUS_OK;
static float sensor_q[4], sensor_a[3], sensor_m[3];

static uint8_t packet_sequence = 0;
static int64_t last_ping_time = 0;
static uint32_t ping_interval_ms = PING_INTERVAL_MS;

#define PING_RESYNC_MIN_INTERVAL_MS 500

LOG_MODULE_REGISTER(connection, LOG_LEVEL_INF);

#ifndef CONFIG_CONNECTION_ENABLE_ACK
static bool no_ack = true;
#else
static bool no_ack = false;
#endif

uint32_t get_ping_interval_ms(void)
{
	return ping_interval_ms + esb_get_ping_backoff_ms();
}

static void connection_thread(void);
K_THREAD_DEFINE(connection_thread_id, 2048, connection_thread, NULL, NULL, NULL, 5, 0, 0);

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
	tdma_init(id);
}

uint8_t connection_get_packet_sequence(void)
{
	return packet_sequence;
}

/*
 * Sub-packet data sizes (payload only, excluding type byte).
 * These define how much data each sub-packet type contributes
 * when embedded inside a composite packet.
 */
#define SUB_DATA_LEN_INFO   13  /* type 0: batt..patch (no rssi) */
#define SUB_DATA_LEN_QUAT   14  /* type 1: q0-q3 + a0-a2 */
#define SUB_DATA_LEN_COMPACT 13 /* type 2: batt+temp+q_buf+a (no rssi) */
#define SUB_DATA_LEN_STATUS  2  /* type 3: svr_stat + status */
#define SUB_DATA_LEN_MAG    14  /* type 4: q0-q3 + m0-m2 */
#define SUB_DATA_LEN_RUNTIME 8  /* type 5: remaining runtime estimate */

/* Fill sub-packet payload (without type/id prefix) into buf, return bytes written */
static int fill_sub_info(uint8_t *buf)
{
	buf[0] = batt;
	buf[1] = batt_v;
	buf[2] = sensor_temp;
	buf[3] = FW_BOARD;
	buf[4] = FW_MCU;
	buf[5] = 0; /* resv */
	buf[6] = imu_id;
	buf[7] = mag_id;
	uint16_t *fw = (uint16_t *)&buf[8];
	fw[0] = ((BUILD_YEAR - 2020) & 127) << 9 | (BUILD_MONTH & 15) << 5 | (BUILD_DAY & 31);
	buf[10] = FW_VERSION_MAJOR & 255;
	buf[11] = FW_VERSION_MINOR & 255;
	buf[12] = FW_VERSION_PATCH & 255;
	return SUB_DATA_LEN_INFO;
}

static int fill_sub_quat_accel(uint8_t *buf)
{
	uint16_t *b = (uint16_t *)buf;
	b[0] = TO_FIXED_15(sensor_q[1]);
	b[1] = TO_FIXED_15(sensor_q[2]);
	b[2] = TO_FIXED_15(sensor_q[3]);
	b[3] = TO_FIXED_15(sensor_q[0]);
	b[4] = TO_FIXED_7(sensor_a[0]);
	b[5] = TO_FIXED_7(sensor_a[1]);
	b[6] = TO_FIXED_7(sensor_a[2]);
	return SUB_DATA_LEN_QUAT;
}

static int fill_sub_compact_quat(uint8_t *buf)
{
	buf[0] = batt;
	buf[1] = batt_v;
	buf[2] = sensor_temp;
	float v[3] = {0};
	q_fem(sensor_q, v);
	for (int i = 0; i < 3; i++)
		v[i] = (v[i] + 1) / 2;
	uint16_t v_buf[3] = {
		SATURATE_UINT10((1 << 10) * v[0]),
		SATURATE_UINT11((1 << 11) * v[1]),
		SATURATE_UINT11((1 << 11) * v[2])
	};
	uint32_t *q_buf = (uint32_t *)&buf[3];
	*q_buf = v_buf[0] | (v_buf[1] << 10) | (v_buf[2] << 21);
	uint16_t *ab = (uint16_t *)&buf[7];
	ab[0] = TO_FIXED_7(sensor_a[0]);
	ab[1] = TO_FIXED_7(sensor_a[1]);
	ab[2] = TO_FIXED_7(sensor_a[2]);
	return SUB_DATA_LEN_COMPACT;
}

static int fill_sub_status(uint8_t *buf)
{
	buf[0] = tracker_svr_status;
	buf[1] = tracker_status;
	return SUB_DATA_LEN_STATUS;
}

static int fill_sub_mag(uint8_t *buf)
{
	uint16_t *b = (uint16_t *)buf;
	b[0] = TO_FIXED_15(sensor_q[1]);
	b[1] = TO_FIXED_15(sensor_q[2]);
	b[2] = TO_FIXED_15(sensor_q[3]);
	b[3] = TO_FIXED_15(sensor_q[0]);
	b[4] = TO_FIXED_10(sensor_m[0]);
	b[5] = TO_FIXED_10(sensor_m[1]);
	b[6] = TO_FIXED_10(sensor_m[2]);
	return SUB_DATA_LEN_MAG;
}

static int fill_sub_runtime(uint8_t *buf)
{
	uint64_t runtime_us = k_ticks_to_us_floor64(sys_get_battery_remaining_time_estimate());
	memcpy(buf, &runtime_us, sizeof(runtime_us));
	return SUB_DATA_LEN_RUNTIME;
}

/* Return sub-packet data size for a given type */
static int sub_data_len(uint8_t type)
{
	switch (type) {
	case 0: return SUB_DATA_LEN_INFO;
	case 1: return SUB_DATA_LEN_QUAT;
	case 2: return SUB_DATA_LEN_COMPACT;
	case 3: return SUB_DATA_LEN_STATUS;
	case 4: return SUB_DATA_LEN_MAG;
	case 5: return SUB_DATA_LEN_RUNTIME;
	default: return 0;
	}
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
	send_precise_quat = q_epsilon(q, sensor_q, 0.005f);
	memcpy(sensor_q, q, sizeof(sensor_q));
	memcpy(sensor_a, a, sizeof(sensor_a));
	quat_update_time = k_uptime_get();
}

static int64_t mag_update_time = 0;
static int64_t last_mag_time = 0;

void connection_update_sensor_mag(float *m)
{
	memcpy(sensor_m, m, sizeof(sensor_m));
	mag_update_time = k_uptime_get();
}

void connection_update_sensor_temp(float temp)
{
	// sensor_temp == zero means no data
#if CONFIG_SENSOR_USE_VQF
	if (sensor_get_mag_available() && sensor_get_mag_enabled()) {
		// temp hack to display vqf mag disturbance detection status
		vqf_debug_info_t vqf_info;
		vqf_get_debug_info(&vqf_info);
		if (vqf_info.mag_dist_detected) {
			if (temp < 38.5f) {  // assume normal operating should be below 38.5C
				temp = -temp; // invert temp to indicate mag disturbance, negative means disturbed, positive means normal
			}
		}
	}
#endif
	if (temp < -38.5f) {
		sensor_temp = 1;
	} else if (temp > 88.5f) {
		sensor_temp = 255;
	} else {
		sensor_temp = ((temp - 25) * 2 + 128.5f); // -38.5 - +88.5 -> 1-255
	}
}

// format for packet send
void connection_update_battery(bool battery_available, bool plugged, bool charged, uint32_t battery_pptt, int battery_mV)
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

	if (charged) { // 255, server will show fully charged indicator (not yet)
		batt = 255;
	}

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
//|rssi    | |3      |id      |svr_stat|status  |resv |rssi    |
//| |4      |id      |q0               |q1               |q2               |q3               |m0 |m1 |m2 |
//| |5      |id      |runtime (uint64, us)                              |resv              |rssi |

void connection_write_packet_0() // device info
{
	uint8_t data[16] = {0};
	data[0] = 0;
	data[1] = tracker_id;
	fill_sub_info(&data[2]);
	data[15] = 0; // rssi (supplied by receiver)

	uint8_t esb_pkt[ESB_SENSOR_DATA_LEN];
	memcpy(esb_pkt, data, 16);
	esb_pkt[16] = packet_sequence++;
	esb_write(esb_pkt, no_ack, ESB_SENSOR_DATA_LEN);
}

void connection_write_packet_1() // full precision quat and accel
{
	uint8_t data[16] = {0};
	data[0] = 1;
	data[1] = tracker_id;
	fill_sub_quat_accel(&data[2]);

	uint8_t esb_pkt[ESB_SENSOR_DATA_LEN];
	memcpy(esb_pkt, data, 16);
	esb_pkt[16] = packet_sequence++;
	esb_write(esb_pkt, no_ack, ESB_SENSOR_DATA_LEN);
}

void connection_write_packet_2() // reduced precision quat and accel with battery,
								 // temp, and rssi
{
	uint8_t data[16] = {0};
	data[0] = 2;
	data[1] = tracker_id;
	fill_sub_compact_quat(&data[2]);
	data[15] = 0; // rssi (supplied by receiver)

	uint8_t esb_pkt[ESB_SENSOR_DATA_LEN];
	memcpy(esb_pkt, data, 16);
	esb_pkt[16] = packet_sequence++;
	esb_write(esb_pkt, no_ack, ESB_SENSOR_DATA_LEN);
}

void connection_write_packet_3() // status
{
	uint8_t data[16] = {0};
	data[0] = 3;
	data[1] = tracker_id;
	fill_sub_status(&data[2]);
	data[15] = 0; // rssi (supplied by receiver)

	uint8_t esb_pkt[ESB_SENSOR_DATA_LEN];
	memcpy(esb_pkt, data, 16);
	esb_pkt[16] = packet_sequence++;
	esb_write(esb_pkt, no_ack, ESB_SENSOR_DATA_LEN);
}

void connection_write_packet_4() // full precision quat and magnetometer
{
	uint8_t data[16] = {0};
	data[0] = 4;
	data[1] = tracker_id;
	fill_sub_mag(&data[2]);

	uint8_t esb_pkt[ESB_SENSOR_DATA_LEN];
	memcpy(esb_pkt, data, 16);
	esb_pkt[16] = packet_sequence++;
	esb_write(esb_pkt, no_ack, ESB_SENSOR_DATA_LEN);
}

void connection_write_packet_5() // runtime estimate
{
	uint8_t data[16] = {0};
	data[0] = 5;
	data[1] = tracker_id;
	uint64_t runtime_us = k_ticks_to_us_floor64(sys_get_battery_remaining_time_estimate());
	memcpy(&data[2], &runtime_us, sizeof(runtime_us));
	data[15] = 0; // rssi (supplied by receiver)

	uint8_t esb_pkt[ESB_SENSOR_DATA_LEN];
	memcpy(esb_pkt, data, 16);
	esb_pkt[16] = packet_sequence++;
	esb_write(esb_pkt, no_ack, ESB_SENSOR_DATA_LEN);
}

static int64_t last_info_time = 0;
static int64_t last_status_time = 0;
static int64_t last_runtime_time = 0;

/*
 * Raw sensor data collection subsystem.
 *
 * Runtime-controlled: activated via ESB_PONG_FLAG_DATA_COLLECT_ON command.
 * Sensor thread queues raw IMU/mag samples via message queues.
 * Connection thread drains them and sends ESB packets with floats.
 *
 * ESB Raw IMU packet (type 0x10, 48 bytes):
 *   [0]    type 0x10
 *   [1]    tracker_id
 *   [2-3]  sequence (16-bit BE)
 *   [4-15] gyro x,y,z (float × 3, deg/s)
 *   [16-27] accel x,y,z (float × 3, g)
 *   [28-39] mag x,y,z (float × 3, or zeros)
 *   [40]   flags (bit0: has_new_mag)
 *   [41]   temperature (uint8)
 *   [42-47] reserved
 */
#include <zephyr/sys/byteorder.h>

#define RAW_IMU_QUEUE_SIZE  16

struct raw_imu_queued {
	float gyro[3];
	float accel[3];
};

K_MSGQ_DEFINE(raw_imu_msgq, sizeof(struct raw_imu_queued), RAW_IMU_QUEUE_SIZE, 4);

static uint16_t raw_sequence = 0;
static bool data_collection_active = false;

/*
 * ARQ ring buffer: stores last RAW_RING_SIZE sent packets for retransmission.
 * Indexed by (sequence % RAW_RING_SIZE).
 */
#define RAW_RING_SIZE 512
static uint8_t raw_ring[RAW_RING_SIZE][ESB_MAX_PAYLOAD_LEN];
static bool    raw_ring_valid[RAW_RING_SIZE];

/*
 * Retransmit queue: filled by ESB event handler when ACK payload carries
 * retransmit requests (marker 0xAA).  Up to RAW_RETX_MAX entries.
 * Connection thread drains this before sending new data.
 */
#define RAW_RETX_MAX 16
#define RAW_ARQ_MARKER 0xAA
volatile uint16_t raw_retx_queue[RAW_RETX_MAX];
volatile uint8_t  raw_retx_count;
static volatile uint32_t raw_retx_total;  /* lifetime retransmit count */
static bool raw_metadata_sent = false;
static int64_t raw_metadata_last_ms = 0;
#define RAW_METADATA_RESEND_MS 60000

/* Latest mag data for piggybacking onto IMU packets */
static float latest_mag[3] = {0};
static bool latest_mag_valid = false;

void connection_set_data_collection(bool enable)
{
	if (enable && !data_collection_active) {
		/* Flush any stale data in queues */
		k_msgq_purge(&raw_imu_msgq);
		raw_sequence = 0;
		raw_metadata_sent = false;
		latest_mag_valid = false;
		/* Reset ARQ state */
		memset(raw_ring_valid, 0, sizeof(raw_ring_valid));
		raw_retx_count = 0;
		raw_retx_total = 0;
	}
	data_collection_active = enable;
	LOG_INF("Data collection %s", enable ? "STARTED" : "STOPPED");
}

bool connection_get_data_collection(void)
{
	return data_collection_active;
}

void connection_queue_raw_sample(const struct raw_imu_sample *sample)
{
	if (!data_collection_active) return;

	struct raw_imu_queued entry;
	memcpy(entry.gyro, sample->gyro, sizeof(entry.gyro));
	memcpy(entry.accel, sample->accel, sizeof(entry.accel));

	if (k_msgq_put(&raw_imu_msgq, &entry, K_NO_WAIT) != 0) {
		struct raw_imu_queued discard;
		k_msgq_get(&raw_imu_msgq, &discard, K_NO_WAIT);
		k_msgq_put(&raw_imu_msgq, &entry, K_NO_WAIT);
	}
}

void connection_queue_raw_mag(const float mag[3])
{
	if (!data_collection_active) return;

	/* Store latest mag for piggybacking onto IMU packets */
	memcpy(latest_mag, mag, sizeof(latest_mag));
	latest_mag_valid = true;
}

void connection_send_raw_metadata(float gyro_range, float accel_range,
				  float gyro_odr, float accel_odr,
				  float mag_odr, uint8_t imu, uint8_t mag)
{
	uint8_t buf[ESB_MAX_PAYLOAD_LEN];
	memset(buf, 0, sizeof(buf));
	buf[0] = ESB_RAW_META_TYPE;
	buf[1] = tracker_id;
	memcpy(&buf[2], &gyro_range, 4);
	memcpy(&buf[6], &accel_range, 4);
	memcpy(&buf[10], &gyro_odr, 4);
	memcpy(&buf[14], &accel_odr, 4);
	memcpy(&buf[18], &mag_odr, 4);
	buf[22] = imu;
	buf[23] = mag;

	esb_write(buf, false, ESB_MAX_PAYLOAD_LEN);
	raw_metadata_sent = true;
	raw_metadata_last_ms = k_uptime_get();
}

bool connection_raw_metadata_resend_due(void)
{
	if (!data_collection_active || !raw_metadata_sent) return false;
	return (k_uptime_get() - raw_metadata_last_ms) >= RAW_METADATA_RESEND_MS;
}

bool connection_process_raw_data(void)
{
	if (!data_collection_active)
		return false;

	/* Send metadata once when data collection starts */
	if (!raw_metadata_sent) {
		/* Metadata will be sent by sensor_loop after it detects
		 * data_collection_active. Skip until it's sent. */
		return false;
	}

	/* Priority 1: Process retransmit requests from ARQ ACK payloads */
	if (raw_retx_count > 0) {
		uint16_t seq = raw_retx_queue[0];
		uint16_t idx = seq % RAW_RING_SIZE;

		if (raw_ring_valid[idx]) {
			/* Retransmit from ring buffer */
			esb_write(raw_ring[idx], false, ESB_MAX_PAYLOAD_LEN);
			raw_retx_total++;
		}

		/* Remove from queue (shift remaining entries) */
		unsigned irq_key = irq_lock();
		for (uint8_t i = 0; i + 1 < raw_retx_count; i++) {
			raw_retx_queue[i] = raw_retx_queue[i + 1];
		}
		raw_retx_count--;
		irq_unlock(irq_key);
		return true;
	}

	/* Priority 2: Send new IMU sample */
	struct raw_imu_queued sample;
	if (k_msgq_get(&raw_imu_msgq, &sample, K_NO_WAIT) == 0) {
		uint8_t buf[ESB_MAX_PAYLOAD_LEN];
		memset(buf, 0, sizeof(buf));

		buf[0] = ESB_RAW_IMU_TYPE;
		buf[1] = tracker_id;
		uint16_t seq = raw_sequence++;
		sys_put_be16(seq, &buf[2]);

		/* Gyro float × 3 */
		memcpy(&buf[4], &sample.gyro[0], 4);
		memcpy(&buf[8], &sample.gyro[1], 4);
		memcpy(&buf[12], &sample.gyro[2], 4);

		/* Accel float × 3 */
		memcpy(&buf[16], &sample.accel[0], 4);
		memcpy(&buf[20], &sample.accel[1], 4);
		memcpy(&buf[24], &sample.accel[2], 4);

		/* Piggyback latest mag if available */
		uint8_t flags = 0;
		if (latest_mag_valid) {
			memcpy(&buf[28], &latest_mag[0], 4);
			memcpy(&buf[32], &latest_mag[1], 4);
			memcpy(&buf[36], &latest_mag[2], 4);
			flags |= 0x01; /* has_new_mag */
			latest_mag_valid = false;
		}

		buf[40] = flags;
		buf[41] = sensor_temp;

		/* Save to ring buffer for potential retransmission */
		uint16_t ring_idx = seq % RAW_RING_SIZE;
		memcpy(raw_ring[ring_idx], buf, ESB_MAX_PAYLOAD_LEN);
		raw_ring_valid[ring_idx] = true;

		esb_write(buf, false, ESB_MAX_PAYLOAD_LEN);
		return true;
	}

	return false;
}

static int64_t last_sensor_quat_time = 0;
#define SENSOR_QUAT_INTERVAL_TDMA_MS   1
#define SENSOR_QUAT_INTERVAL_NOTDMA_MS 6

/* Lookahead window: if a low-freq packet is within this many ms of being due,
 * piggyback it onto the current transmission as a composite sub-packet. */
#define COMPOSITE_LOOKAHEAD_MS 10

/* Max sub-packet payload in a composite: ESB_MAX_PAYLOAD_LEN - 3 (header) - 1 (sequence) */
#define COMPOSITE_MAX_SUB_DATA (ESB_MAX_PAYLOAD_LEN - 4)

/*
 * Build and send a composite packet (type 0x05) containing multiple sub-packets.
 * types[] / lens[] describe the sub-packets to include; n is the count.
 * Each sub-packet is: [type_byte][data...].
 * Format: [0x05][tracker_id][sub_count][sub0_type][sub0_data...][sub1_type][sub1_data...]...[sequence]
 */
static void send_composite(const uint8_t *types, int n)
{
	uint8_t buf[ESB_MAX_PAYLOAD_LEN];
	int pos = 0;

	buf[pos++] = ESB_COMPOSITE_TYPE;
	buf[pos++] = tracker_id;
	buf[pos++] = (uint8_t)n;

	for (int i = 0; i < n; i++) {
		uint8_t t = types[i];
		buf[pos++] = t;
		switch (t) {
		case 0: pos += fill_sub_info(&buf[pos]); break;
		case 1: pos += fill_sub_quat_accel(&buf[pos]); break;
		case 2: pos += fill_sub_compact_quat(&buf[pos]); break;
		case 3: pos += fill_sub_status(&buf[pos]); break;
		case 4: pos += fill_sub_mag(&buf[pos]); break;
		case 5: pos += fill_sub_runtime(&buf[pos]); break;
		default: break;
		}
	}

	buf[pos++] = packet_sequence++;
	esb_write(buf, no_ack, pos);
}

/*
 * Try to append a sub-packet type to the pending composite list.
 * Returns true if it fits within COMPOSITE_MAX_SUB_DATA.
 */
static bool composite_try_add(uint8_t *types, int *n, int *used, uint8_t type)
{
	int need = 1 + sub_data_len(type); /* 1 for type byte + data */
	if (*used + need > COMPOSITE_MAX_SUB_DATA)
		return false;
	types[*n] = type;
	(*n)++;
	*used += need;
	return true;
}

void connection_thread(void)
{
	/* Register connection thread with watchdog */
	watchdog_register_thread(WDT_CHANNEL_CONNECTION, 0);

	while (1) {
		int64_t now = k_uptime_get();

		watchdog_feed(WDT_CHANNEL_CONNECTION);

		/* Adaptive PING interval based on connection health */
		if (get_status(SYS_STATUS_CONNECTION_ERROR)) {
			ping_interval_ms = 1450;
		} else {
			ping_interval_ms = PING_INTERVAL_MS;
		}

		if (!esb_ready()) {
			k_msleep(100);
			continue;
		}

		/* PING has highest priority.
		 *
		 * When TDMA is enabled and the last sync is getting stale
		 * (>2× PING interval), force an early PING to re-sync before
		 * the TDMA slot estimate drifts too far.  This prevents the
		 * gradual TPS degradation caused by transmitting in wrong slots.
		 */
		uint32_t effective_ping_interval_ms = get_ping_interval_ms();
		bool ping_due = (now - last_ping_time >= effective_ping_interval_ms);
#if CONFIG_CONNECTION_TDMA
		if (!ping_due && tdma_is_enabled()) {
			int64_t sync_age = esb_get_sync_age_ms();
			uint32_t resync_interval_ms = effective_ping_interval_ms / 2;
			if (resync_interval_ms < PING_RESYNC_MIN_INTERVAL_MS) {
				resync_interval_ms = PING_RESYNC_MIN_INTERVAL_MS;
			}
			if (sync_age > (int64_t)effective_ping_interval_ms * 2 && (now - last_ping_time) >= resync_interval_ms) {
				ping_due = true;
			}
		}
#endif
		if (ping_due) {
			uint8_t ping[ESB_PING_LEN] = {0};
			ping[0] = ESB_PING_TYPE;
			ping[1] = connection_get_id();
			ping[2] = 0;
			memset(&ping[3], 0x00, 4);
			ping[7] = esb_get_ping_ack_flag();
			memset(&ping[8], 0x00, 4);
			ping[ESB_PING_LEN - 1] = 0;
			esb_write(ping, false, ESB_PING_LEN);
			last_ping_time = now;
			// k_usleep(400);
			continue;
		}

		/* Skip sensor data during connection error */
		if (get_status(SYS_STATUS_CONNECTION_ERROR)) {
			k_msleep(100);
			continue;
		}

		/* Raw data has priority over fusion data to minimize latency */
		if (connection_process_raw_data()) {
			// k_usleep(300); /* Brief delay for ESB TX completion */
			continue;
		}

		/* During data collection, throttle fusion data
		 * to leave radio bandwidth for raw data. */
		if (data_collection_active) {
			static int64_t last_fusion_dc_time;
			if (now - last_fusion_dc_time < 9) {
				k_usleep(300);
				continue;
			}
			last_fusion_dc_time = now;
		}

		/* Determine which data types are due or nearly due */
		int quat_interval_ms = tdma_is_enabled()
			? SENSOR_QUAT_INTERVAL_TDMA_MS
			: SENSOR_QUAT_INTERVAL_NOTDMA_MS;
		bool quat_ready = quat_update_time &&
				  (now - last_sensor_quat_time >= quat_interval_ms);
		bool mag_due = mag_update_time && (now - last_mag_time > 100);
		bool info_due = (now - last_info_time > 100);
		bool status_due = (now - last_status_time > 1000);
		bool runtime_due = (now - last_runtime_time > 1000);

		/* Lookahead: consider nearly-due low-freq packets for piggybacking */
		bool info_soon = (now - last_info_time > 100 - COMPOSITE_LOOKAHEAD_MS);
		bool status_soon = (now - last_status_time > 1000 - COMPOSITE_LOOKAHEAD_MS);
		bool runtime_soon = (now - last_runtime_time > 1000 - COMPOSITE_LOOKAHEAD_MS);
		bool mag_soon = mag_update_time && (now - last_mag_time > 100 - COMPOSITE_LOOKAHEAD_MS);

		if (quat_ready) {
			/* Count how many extras we can piggyback */
			int extras = (mag_due || mag_soon ? 1 : 0)
				   + (info_soon && !send_precise_quat ? 0 : (info_due || info_soon ? 1 : 0))
				   + (status_due || status_soon ? 1 : 0)
				   + (runtime_due || runtime_soon ? 1 : 0);

			if (extras > 0) {
				/* Build composite packet */
				uint8_t types[5];
				int n = 0, used = 0;

				/* Primary: quat sub-packet */
				if (mag_due || mag_soon) {
					/* mag includes full quat, use type 4 instead of separate quat+mag */
					composite_try_add(types, &n, &used, 4);
					mag_update_time = 0;
					last_mag_time = now;
				} else if (!send_precise_quat && (info_due || info_soon)) {
					/* compact quat (type 2) already contains batt/temp like info */
					composite_try_add(types, &n, &used, 2);
					last_info_time = now;
				} else {
					composite_try_add(types, &n, &used, 1);
				}

				/* Piggyback low-freq sub-packets if they fit */
				if ((status_due || status_soon) && composite_try_add(types, &n, &used, 3))
					last_status_time = now;
				if ((runtime_due || runtime_soon) && composite_try_add(types, &n, &used, 5))
					last_runtime_time = now;
				if ((info_due || info_soon) && now - last_info_time != now)
					; /* info already handled via compact quat or not due */
				else if ((info_due || info_soon) && composite_try_add(types, &n, &used, 0))
					last_info_time = now;

				if (n > 1) {
					send_composite(types, n);
				} else {
					/* Only one sub-packet — send as standard packet instead */
					uint8_t t = types[0];
					if (t == 1)
						connection_write_packet_1();
					else if (t == 2)
						connection_write_packet_2();
					else if (t == 4)
						connection_write_packet_4();
					else
						connection_write_packet_1();
				}
			} else {
				/* No extras to piggyback — send standard quat packet */
				if (!send_precise_quat && info_due) {
					last_info_time = now;
					connection_write_packet_2();
				} else {
					connection_write_packet_1();
				}
			}
			quat_update_time = 0;
			last_quat_time = now;
			last_sensor_quat_time = now;
			continue;
		}

		/* No quat ready — handle standalone low-freq packets */
		if (mag_due) {
			/* Mag with optional low-frequency piggyback */
			if (status_due || status_soon || runtime_due || runtime_soon) {
				uint8_t types[3];
				int n = 0, used = 0;
				composite_try_add(types, &n, &used, 4);
				if ((status_due || status_soon) && composite_try_add(types, &n, &used, 3))
					last_status_time = now;
				if ((runtime_due || runtime_soon) && composite_try_add(types, &n, &used, 5))
					last_runtime_time = now;
				send_composite(types, n);
			} else {
				connection_write_packet_4();
			}
			mag_update_time = 0;
			last_mag_time = now;
			continue;
		}

		if (info_due) {
			/* Info with optional low-frequency piggyback */
			if (status_due || status_soon || runtime_due || runtime_soon) {
				uint8_t types[3];
				int n = 0, used = 0;
				composite_try_add(types, &n, &used, 0);
				if ((status_due || status_soon) && composite_try_add(types, &n, &used, 3))
					last_status_time = now;
				if ((runtime_due || runtime_soon) && composite_try_add(types, &n, &used, 5))
					last_runtime_time = now;
				send_composite(types, n);
			} else {
				connection_write_packet_0();
			}
			last_info_time = now;
			continue;
		}

		if (status_due) {
			if (runtime_due || runtime_soon) {
				uint8_t types[2];
				int n = 0, used = 0;
				composite_try_add(types, &n, &used, 3);
				last_status_time = now;
				if (composite_try_add(types, &n, &used, 5))
					last_runtime_time = now;
				send_composite(types, n);
			} else {
				last_status_time = now;
				connection_write_packet_3();
			}
			continue;
		}

		if (runtime_due) {
			last_runtime_time = now;
			connection_write_packet_5();
			continue;
		}

		k_usleep(600);
	}
}
