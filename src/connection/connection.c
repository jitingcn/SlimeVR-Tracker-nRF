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
#include "sensor/sensor.h"
#include "sensor/calibration/calibration.h"
#include "sensor/calibration/online_mag.h"
#include "connection.h"
#include "util.h"
#include "esb.h"
#include "sensor_data_snapshot.h"
#include "tracker_events.h"
#include "raw_retx.h"
#include "tdma.h"
#include "build_defines.h"
#include "hid.h"
#include "retained.h"
#include "system/battery_tracker.h"
#include "system/watchdog.h"
#include "system/test_mode.h"
#include "system/esb_ota.h"
#if defined(CONFIG_TDMA_DIAGNOSTICS)
#include "radio_capture.h"
#endif

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

static uint8_t tracker_id, batt, batt_v, sensor_temp, imu_id, mag_id, tracker_status;
static uint8_t tracker_svr_status = SVR_STATUS_OK;
static sensor_data_snapshot_t sensor_data_snapshot;
static float sensor_last_q[4];
static bool sensor_ids_set = false; /* true after connection_update_sensor_ids() first called */
static K_SEM_DEFINE(connection_wake_sem, 0, 1);

static void connection_sensor_snap_q_a(float q_out[4], float a_out[3])
{
	sensor_data_snapshot_read_qa(&sensor_data_snapshot, q_out, a_out);
}

static void connection_sensor_snap_q_m(float q_out[4], float m_out[3])
{
	sensor_data_snapshot_read_qm(&sensor_data_snapshot, q_out, m_out);
}

static bool connection_sensor_get_precise_quat(void)
{
	return sensor_data_snapshot_is_precise(&sensor_data_snapshot);
}

static uint8_t packet_sequence = 0;
static atomic_t next_ping_deadline_ms = ATOMIC_INIT(0);
static atomic_t ping_server_phase_aligned = ATOMIC_INIT(0);
static uint32_t ping_interval_ms = PING_INTERVAL_MS;
static uint32_t ping_aligned_interval_ms;

#define PING_RESYNC_MIN_INTERVAL_MS 500
#define PING_QUEUE_RETRY_MS 50
#define PING_SERVER_PHASE_TOLERANCE_MS 2

static struct {
	atomic_t due;
	atomic_t attempts;
	atomic_t queue_ok;
	atomic_t queue_fail;
	atomic_t retry_deferred;
	atomic_t attempts_in_window;
	atomic_t attempts_rate_max;
	atomic_t window_started_ms;
} ping_sched_stats;
static atomic_t ping_resync_requested;

static uint32_t ping_phase_ms(uint32_t interval_ms)
{
	if (interval_ms == 0) {
		return 0;
	}
	return ((uint32_t)(tracker_id % TDMA_MAX_TRACKERS) * interval_ms) / TDMA_MAX_TRACKERS;
}

static uint32_t ping_next_periodic_deadline(uint32_t previous_deadline, uint32_t now, uint32_t interval_ms)
{
	uint32_t next = previous_deadline + interval_ms;
	if ((int32_t)(next - now) <= 0) {
		next = now + interval_ms;
	}
	return next;
}

static uint32_t ping_server_phase_delay_ms(uint32_t interval_ms)
{
	uint64_t server_ticks = esb_get_server_time_ticks_64();
	if (server_ticks == 0 || interval_ms == 0) {
		return 0;
	}

	uint32_t interval_ticks = (uint32_t)(((uint64_t)interval_ms * 32768U + 500U) / 1000U);
	uint32_t phase_ticks
		= (uint32_t)(((uint64_t)ping_phase_ms(interval_ms) * 32768U + 500U) / 1000U);
	uint32_t server_phase_ticks = (uint32_t)(server_ticks % interval_ticks);
	uint32_t delay_ticks = phase_ticks >= server_phase_ticks ? phase_ticks - server_phase_ticks
											  : interval_ticks - server_phase_ticks + phase_ticks;
	uint32_t delay_ms = (uint32_t)(((uint64_t)delay_ticks * 1000U + 32767U) / 32768U);
	if (delay_ms <= PING_SERVER_PHASE_TOLERANCE_MS) {
		delay_ms += interval_ms;
	}
	return delay_ms;
}

static void ping_stats_attempt(uint32_t now)
{
	uint32_t window_started_ms = (uint32_t)atomic_get(&ping_sched_stats.window_started_ms);
	if (window_started_ms == 0) {
		atomic_set(&ping_sched_stats.window_started_ms, (atomic_val_t)now);
	} else if (now - window_started_ms >= 1000) {
		uint32_t attempts = (uint32_t)atomic_set(&ping_sched_stats.attempts_in_window, 0);
		uint32_t max_rate = (uint32_t)atomic_get(&ping_sched_stats.attempts_rate_max);
		if (attempts > max_rate) {
			atomic_set(&ping_sched_stats.attempts_rate_max, (atomic_val_t)attempts);
		}
		atomic_set(&ping_sched_stats.window_started_ms, (atomic_val_t)now);
	}
	atomic_inc(&ping_sched_stats.attempts_in_window);
}

static void connection_signal_wake(void);

void connection_request_ping_resync(void)
{
	/* Called from ESB event context. The connection thread owns the deadline. */
	atomic_set(&ping_resync_requested, 1);
	atomic_set(&ping_server_phase_aligned, 0);
	ping_aligned_interval_ms = 0;
	atomic_set(&next_ping_deadline_ms, (atomic_val_t)k_uptime_get_32());
	connection_signal_wake();
}

void connection_print_ping_stats(void)
{
	uint32_t attempts_in_window = (uint32_t)atomic_get(&ping_sched_stats.attempts_in_window);
	uint32_t max_rate = (uint32_t)atomic_get(&ping_sched_stats.attempts_rate_max);
	if (attempts_in_window > max_rate) {
		max_rate = attempts_in_window;
	}
	printk(
		"PING SCHED due=%u attempt=%u queue_ok=%u queue_fail=%u retry_deferred=%u rate_max=%u/s next_ms=%u phase_ms=%u aligned=%u resync_pending=%u\n",
		(uint32_t)atomic_get(&ping_sched_stats.due),
		(uint32_t)atomic_get(&ping_sched_stats.attempts),
		(uint32_t)atomic_get(&ping_sched_stats.queue_ok),
		(uint32_t)atomic_get(&ping_sched_stats.queue_fail),
		(uint32_t)atomic_get(&ping_sched_stats.retry_deferred),
		max_rate,
		(uint32_t)atomic_get(&next_ping_deadline_ms),
		ping_phase_ms(get_ping_interval_ms()),
		(uint32_t)atomic_get(&ping_server_phase_aligned),
		(uint32_t)atomic_get(&ping_resync_requested)
	);
}

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
K_THREAD_DEFINE(connection_thread_id, 2048, connection_thread, NULL, NULL, NULL, CONNECTION_THREAD_PRIORITY, K_FP_REGS, 0);

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
	atomic_set(&ping_server_phase_aligned, 0);
	ping_aligned_interval_ms = 0;
	atomic_set(
		&next_ping_deadline_ms,
		(atomic_val_t)(k_uptime_get_32() + ping_phase_ms(PING_INTERVAL_MS))
	);
	tdma_init(id);
}

uint8_t connection_get_packet_sequence(void)
{
	return packet_sequence;
}

enum sub_packet_type {
	SUB_PACKET_INFO = 0,
	SUB_PACKET_QUAT_ACCEL = 1,
	SUB_PACKET_COMPACT_QUAT = 2,
	SUB_PACKET_STATUS = 3,
	SUB_PACKET_QUAT_MAG = 4,
	SUB_PACKET_RUNTIME = 5,
};

/*
 * Sub-packet data sizes (payload only, excluding type byte).
 * These define how much data each sub-packet type contributes
 * when embedded inside a composite packet.
 */
#define SUB_DATA_LEN_INFO 13    /* type 0: batt..patch (no rssi) */
#define SUB_DATA_LEN_QUAT 14    /* type 1: q0-q3 + a0-a2 */
#define SUB_DATA_LEN_COMPACT 13 /* type 2: batt+temp+q_buf+a (no rssi) */
#define SUB_DATA_LEN_STATUS 2   /* type 3: svr_stat + status */
#define SUB_DATA_LEN_MAG 14     /* type 4: q0-q3 + m0-m2 */
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
	float q[4], a[3];
	connection_sensor_snap_q_a(q, a);
	uint16_t *b = (uint16_t *)buf;
	b[0] = TO_FIXED_15(q[1]);
	b[1] = TO_FIXED_15(q[2]);
	b[2] = TO_FIXED_15(q[3]);
	b[3] = TO_FIXED_15(q[0]);
	b[4] = TO_FIXED_7(a[0]);
	b[5] = TO_FIXED_7(a[1]);
	b[6] = TO_FIXED_7(a[2]);
	return SUB_DATA_LEN_QUAT;
}

static int fill_sub_compact_quat(uint8_t *buf)
{
	float q[4], a[3];
	connection_sensor_snap_q_a(q, a);
	buf[0] = batt;
	buf[1] = batt_v;
	buf[2] = sensor_temp;
	float v[3] = {0};
	q_fem(q, v);
	for (int i = 0; i < 3; i++) {
		v[i] = (v[i] + 1) / 2;
	}
	uint16_t v_buf[3]
		= {SATURATE_UINT10((1 << 10) * v[0]), SATURATE_UINT11((1 << 11) * v[1]), SATURATE_UINT11((1 << 11) * v[2])};
	uint32_t *q_buf = (uint32_t *)&buf[3];
	*q_buf = v_buf[0] | (v_buf[1] << 10) | (v_buf[2] << 21);
	uint16_t *ab = (uint16_t *)&buf[7];
	ab[0] = TO_FIXED_7(a[0]);
	ab[1] = TO_FIXED_7(a[1]);
	ab[2] = TO_FIXED_7(a[2]);
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
	float q[4], m[3];
	connection_sensor_snap_q_m(q, m);
	uint16_t *b = (uint16_t *)buf;
	b[0] = TO_FIXED_15(q[1]);
	b[1] = TO_FIXED_15(q[2]);
	b[2] = TO_FIXED_15(q[3]);
	b[3] = TO_FIXED_15(q[0]);
	b[4] = TO_FIXED_10(m[0]);
	b[5] = TO_FIXED_10(m[1]);
	b[6] = TO_FIXED_10(m[2]);
	return SUB_DATA_LEN_MAG;
}

static int fill_sub_runtime(uint8_t *buf)
{
	uint64_t runtime_us = k_ticks_to_us_floor64(sys_get_battery_remaining_time_estimate());
	memcpy(buf, &runtime_us, sizeof(runtime_us));
	return SUB_DATA_LEN_RUNTIME;
}

struct composite_builder {
	uint8_t types[5];
	int64_t *last_times[5];
	int64_t stamps[5];
	int n;
	int used;
	bool has_event;
	struct tracker_event_tx event;
};

typedef int (*sub_fill_fn)(uint8_t *buf);

struct sub_packet_desc {
	uint8_t data_len;
	bool pad_byte15; /* normal 16-byte packet: force data[15]=0 after fill */
	sub_fill_fn fill;
};

/* Indexed by sub-packet type. One source for len + fill used by normal + composite. */
static const struct sub_packet_desc sub_packet_table[] = {
	[SUB_PACKET_INFO] = {SUB_DATA_LEN_INFO, true, fill_sub_info},
	[SUB_PACKET_QUAT_ACCEL] = {SUB_DATA_LEN_QUAT, false, fill_sub_quat_accel},
	[SUB_PACKET_COMPACT_QUAT] = {SUB_DATA_LEN_COMPACT, true, fill_sub_compact_quat},
	[SUB_PACKET_STATUS] = {SUB_DATA_LEN_STATUS, true, fill_sub_status},
	[SUB_PACKET_QUAT_MAG] = {SUB_DATA_LEN_MAG, false, fill_sub_mag},
	[SUB_PACKET_RUNTIME] = {SUB_DATA_LEN_RUNTIME, true, fill_sub_runtime},
};

static const struct sub_packet_desc *sub_packet_get(uint8_t type)
{
	if (type >= ARRAY_SIZE(sub_packet_table) || sub_packet_table[type].fill == NULL) {
		return NULL;
	}
	return &sub_packet_table[type];
}

static int sub_data_len(uint8_t type)
{
	const struct sub_packet_desc *d = sub_packet_get(type);
	return d ? d->data_len : 0;
}

static bool connection_hid_output_ready(void)
{
#if CONFIG_CONNECTION_OVER_HID
	return get_status(SYS_STATUS_USB_CONNECTED);
#else
	return false;
#endif
}

static void fill_normal_packet(uint8_t type, uint8_t data[16])
{
	memset(data, 0, 16);
	const struct sub_packet_desc *d = sub_packet_get(type);
	if (!d) {
		type = SUB_PACKET_QUAT_ACCEL;
		d = sub_packet_get(SUB_PACKET_QUAT_ACCEL);
	}
	data[0] = type;
	data[1] = tracker_id;
	d->fill(&data[2]);
	if (d->pad_byte15) {
		data[15] = 0;
	}
}

static bool write_normal_packet(const uint8_t data[16])
{
	/* Radio unavailable: HID is the configured output path, so its
	 * enqueue success is the send success. */
	if (!esb_ready()) {
#if CONFIG_CONNECTION_OVER_HID
		if (!connection_hid_output_ready()) {
			return false;
		}
		return hid_write_packet_n(data);
#else
		return false;
#endif
	}

	/* Radio available: only ESB queue success counts, so a failed
	 * admission cannot consume the test-rate slot or mirror to HID;
	 * HID stays a mirror of successful sends. */
	uint8_t esb_pkt[ESB_SENSOR_DATA_LEN];

	memcpy(esb_pkt, data, 16);
	esb_pkt[16] = packet_sequence;
	if (esb_write(esb_pkt, no_ack, ESB_SENSOR_DATA_LEN) != 0) {
		return false;
	}
	packet_sequence++;
#if CONFIG_CONNECTION_OVER_HID
	if (connection_hid_output_ready()) {
		hid_write_packet_n(data);
	}
#endif
	return true;
}

#if CONFIG_CONNECTION_OVER_HID
static bool write_hid_packet_type(uint8_t type)
{
	if (!connection_hid_output_ready()) {
		return false;
	}

	uint8_t data[16];

	fill_normal_packet(type, data);
	return hid_write_packet_n(data);
}

static bool write_hid_composite_as_normal_packets(const struct composite_builder *builder)
{
	bool queued = true;
	for (int i = 0; i < builder->n; i++) {
		queued &= write_hid_packet_type(builder->types[i]);
	}
	return queued;
}
#endif

static bool connection_write_packet_type(uint8_t type)
{
	uint8_t data[16];
	fill_normal_packet(type, data);
	return write_normal_packet(data);
}

void connection_update_sensor_ids(int imu, int mag)
{
	imu_id = get_server_constant_imu_id(imu);
	mag_id = get_server_constant_mag_id(mag);
	sensor_ids_set = true;
}

void connection_update_sensor_data(float *q, float *a, int64_t data_time)
{
	// data_time is in system ticks, nonzero means valid measurement
	// TODO: use data_time to measure latency! the latency should be calculated up to before radio sent data

	// Reject NaN quaternions to prevent sending invalid data to server
	if (isnan(q[0]) || isnan(q[1]) || isnan(q[2]) || isnan(q[3])) {
		LOG_WRN("Rejected NaN quaternion");
		return;
	}

	bool precise = q_epsilon(q, sensor_last_q, 0.005f);
	memcpy(sensor_last_q, q, sizeof(sensor_last_q));
	sensor_data_snapshot_publish_qa(&sensor_data_snapshot, q, a, precise);
	connection_signal_wake();
}

static int64_t last_mag_time = 0;

void connection_update_sensor_mag(float *m)
{
	sensor_data_snapshot_publish_m(&sensor_data_snapshot, m);
	connection_signal_wake();
}

void connection_update_sensor_temp(float temp)
{
	// sensor_temp == zero means no data
	if (sensor_get_mag_available() && sensor_get_mag_enabled()) {
		// temp hack to display fusion mag disturbance detection status
		if (sensor_fusion_get_mag_dist_detected()) {
			if (temp < 38.5f) { // assume normal operating should be below 38.5C
				temp
					= -temp; // invert temp to indicate mag disturbance, negative means disturbed, positive means normal
			}
		}
	}
	if (temp < -38.5f) {
		sensor_temp = 1;
	} else if (temp > 88.5f) {
		sensor_temp = 255;
	} else {
		sensor_temp = (uint8_t)((temp - 25) * 2 + 128.5f); // -38.5 - +88.5 -> 1-255
	}
}

// format for packet send
void connection_update_battery(
	bool battery_available,
	bool plugged,
	bool charged,
	uint32_t battery_pptt,
	int battery_mV
)
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
 * ESB raw meta (ESB_RAW_META_TYPE, RAW_PACKET_SIZE):
 *   [2-5]   gyro_range
 *   [6-9]   accel_range
 *   [10-13] gyro_odr / raw TX Hz (equals fusion INT_merge rate; host gyrTs)
 *   [14-17] accel_odr
 *   [18-21] mag_odr
 *   [22]    imu_id
 *   [23]    mag_id
 *   [24-27] chip_gyro_hz
 *   [28-31] fusion_gyro_hz (0 = omit)
 *
 * ESB raw IMU + gyrQuat (ESB_RAW_IMU_QUAT_TYPE, RAW_PACKET_SIZE):
 *   [0]    packet type
 *   [1]    tracker_id
 *   [2-3]  sequence (16-bit BE)
 *   [4-19] gyr_quat w,x,y,z (float × 4, accumulated raw integration)
 *   [20-31] accel x,y,z (float × 3, g)
 *   [32-43] mag x,y,z (float × 3, or zeros)
 *   [44]   flags (bit0: has_new_mag)
 *   [45-48] T-Cal temperature (float, deg C)
 *   [49-51] reserved
 */
#include <zephyr/sys/byteorder.h>

#define RAW_IMU_QUEUE_SIZE 16

struct raw_imu_queued {
	float gyr_quat[4];
	float accel[3];
	float temp_c;
	uint32_t session;
};

K_MSGQ_DEFINE(raw_imu_msgq, sizeof(struct raw_imu_queued), RAW_IMU_QUEUE_SIZE, 4);

static uint16_t raw_sequence = 0;
static atomic_t data_collection_active;
static atomic_t data_collection_batch_active;
static uint16_t data_collection_batch_rate_hz;
static int64_t dc_conn_error_start;
/* While batch collection runs, fusion data only needs to keep the server
 * side minimally alive; 10 TPS leaves the radio almost fully to raw+TDMA.
 * 0 restores the built-in 100 TPS default on every batch exit path. */
#define DC_BATCH_FUSION_TPS 10U
static volatile bool ota_suppressed = false; /* Reduce poll rate during parallel OTA */
static int64_t ota_suppress_start_time = 0;  /* Timestamp when suppress was enabled */
#define OTA_SUPPRESS_TIMEOUT_MS (10 * 60 * 1000)

/*
 * ARQ ring buffer: stores the last RAW_RING_SIZE sent packets for
 * retransmission. The packet's existing big-endian sequence at bytes 2..3 is
 * authoritative; the bitmap only records whether a slot contains a published
 * packet for the current collection session.
 */
#define RAW_RING_SIZE 128
#define RAW_PACKET_SIZE 52 /* Fixed size for raw meta / gyrQuat / cal payloads */
static uint8_t raw_ring[RAW_RING_SIZE][RAW_PACKET_SIZE];
static uint8_t raw_ring_valid_bits[(RAW_RING_SIZE + 7) / 8];
static uint8_t *raw_ring_get(uint16_t sequence)
{
	uint16_t index = sequence % RAW_RING_SIZE;
	uint8_t mask = (uint8_t)BIT(index & 7U);
	if ((raw_ring_valid_bits[index >> 3] & mask) == 0
		|| sys_get_be16(&raw_ring[index][2]) != sequence) {
		return NULL;
	}
	return raw_ring[index];
}

static void raw_ring_store(const uint8_t packet[RAW_PACKET_SIZE])
{
	uint16_t sequence = sys_get_be16(&packet[2]);
	uint16_t index = sequence % RAW_RING_SIZE;
	uint8_t mask = (uint8_t)BIT(index & 7U);
	uint8_t *valid_byte = &raw_ring_valid_bits[index >> 3];

	/* Invalidate before overwrite, then publish only after the complete
	 * packet (including its BE sequence) is in the slot. */
	*valid_byte &= (uint8_t)~mask;
	memcpy(raw_ring[index], packet, RAW_PACKET_SIZE);
	*valid_byte |= mask;
}

/*
 * Retransmit queue: filled by ESB event handler when ACK payload carries
 * retransmit requests (RAW_ARQ_MARKER). Up to RAW_RETX_MAX entries.
 * Connection thread drains this before sending new data.
 */
static struct raw_retx_requests raw_retx;
static volatile uint32_t raw_retx_total; /* lifetime retransmit count */

int connection_request_raw_retransmit(uint16_t sequence)
{
	unsigned key = irq_lock();
	int err = -EACCES;
	if (connection_get_data_collection() && !connection_get_data_collection_batch()) {
		err = raw_retx_submit(&raw_retx, sequence);
	}
	irq_unlock(key);
	return err;
}
static bool raw_metadata_sent = false;

/* Metadata and calibration are captured once at collection-session start.
 * Subsequent requests select bytes from this immutable snapshot, so a live
 * calibration update cannot mix generations within one recording. */
static bool raw_metadata_pending = false;
static uint8_t raw_metadata_buf[RAW_PACKET_SIZE];
static atomic_t raw_snapshot_ready;
static int64_t raw_meta_cal_last_ms;
static uint8_t raw_cal_pending_mask;
static uint8_t raw_cal_pending_chunk;
static bool raw_cal_pending;
static struct k_spinlock raw_request_lock;
static uint8_t raw_request_mask;
static uint8_t raw_request_chunk;
static uint16_t raw_request_token;
static bool raw_cal_points_all;
static bool raw_request_token_valid;


#define RAW_META_MASK_BASIC      0x01
#define RAW_META_MASK_ACCEL      0x02
#define RAW_META_MASK_MAG        0x04
#define RAW_META_MASK_GYRO       0x08
#define RAW_META_MASK_TCAL_STATE 0x10
#define RAW_META_MASK_TCAL_POINTS 0x20
#define RAW_META_MASK_ALL        0x3f
static int64_t raw_collection_start_ms;
static bool connection_raw_collection_active(void);
static void connection_align_mag_BAinv_body(float out[4][3], const float in[4][3]);

struct raw_cal_snapshot {
	float acc_BAinv[4][3];
	float mag_BAinv[4][3];
	float gyro_bias[3];
	float gyro_scale[3];
	uint8_t tcal_flags;
	uint16_t tcal_count;
	float tcal_temp_min;
	float tcal_temp_max;
	uint8_t tcal_apply_mode;
#if CONFIG_SENSOR_USE_TCAL
	struct TempCalPoint tcal_points[TCAL_BUFFER_SIZE];
#endif
};
static struct raw_cal_snapshot raw_cal_snapshot;

static struct k_spinlock latest_mag_lock;
static float latest_mag[3];
static uint32_t latest_mag_session;
static bool latest_mag_valid;
static atomic_t raw_collection_session;


static void connection_schedule_calibration(uint8_t mask, uint8_t chunk)
{
	if ((mask & RAW_META_MASK_ALL) == 0 || (mask & ~RAW_META_MASK_ALL) != 0) {
		return;
	}
	if (mask & RAW_META_MASK_BASIC) {
		raw_metadata_pending = true;
	}
	raw_cal_pending_mask |= mask & (uint8_t)~RAW_META_MASK_BASIC;
	if (mask & RAW_META_MASK_TCAL_POINTS) {
		raw_cal_points_all = chunk == 255;
		raw_cal_pending_chunk = raw_cal_points_all ? 0 : chunk;
	}
	raw_cal_pending = raw_cal_pending_mask != 0;
}
#if CONFIG_SENSOR_USE_TCAL
static bool connection_tcal_point_valid(const struct TempCalPoint *point)
{
	return point->temp != 0.0f;
}
#endif

static void connection_capture_calibration_snapshot(bool include_tcal)
{
	sensor_imu_calibration_t calibration;
	sensor_calibration_snapshot(&calibration);
	memcpy(raw_cal_snapshot.acc_BAinv, calibration.accel_matrix, sizeof(raw_cal_snapshot.acc_BAinv));
	memcpy(raw_cal_snapshot.gyro_bias, calibration.gyro_bias, sizeof(raw_cal_snapshot.gyro_bias));
	memcpy(raw_cal_snapshot.gyro_scale, retained->gyroSensScale, sizeof(raw_cal_snapshot.gyro_scale));
	float mag_BAinv[4][3];
	float mag_body_BAinv[4][3];
	magneto_online_snapshot_BAinv(mag_BAinv);
	connection_align_mag_BAinv_body(mag_body_BAinv, mag_BAinv);
	memcpy(raw_cal_snapshot.mag_BAinv, mag_body_BAinv, sizeof(raw_cal_snapshot.mag_BAinv));
	raw_cal_snapshot.tcal_flags = 0;
	raw_cal_snapshot.tcal_count = 0;
	raw_cal_snapshot.tcal_temp_min = 0.0f;
	raw_cal_snapshot.tcal_temp_max = 0.0f;
	raw_cal_snapshot.tcal_apply_mode = 0;
#if CONFIG_SENSOR_USE_TCAL
	if (include_tcal) {
		if (retained->tcal_enabled) raw_cal_snapshot.tcal_flags |= 0x01;
		switch (sensor_tcal_get_apply_mode()) {
		case SENSOR_TCAL_APPLY_CURVE: raw_cal_snapshot.tcal_flags |= 0x02; break;
		case SENSOR_TCAL_APPLY_ZRO_FALLBACK: raw_cal_snapshot.tcal_flags |= 0x04; break;
		default: break;
		}
		raw_cal_snapshot.tcal_apply_mode = (uint8_t)sensor_tcal_get_apply_mode();
	raw_cal_snapshot.tcal_temp_min = (float)CONFIG_SENSOR_POLY_TEMP_MIN;
	raw_cal_snapshot.tcal_temp_max = (float)CONFIG_SENSOR_POLY_TEMP_MAX;
		for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
			if (connection_tcal_point_valid(&retained->tempCalPoints[i])) {
				raw_cal_snapshot.tcal_points[raw_cal_snapshot.tcal_count++] = retained->tempCalPoints[i];
			}
		}
	}
#else
	(void)include_tcal;
#endif
}

void connection_request_raw_metadata(uint8_t mask, uint8_t chunk, uint16_t token)
{
	if (!connection_raw_collection_active() || mask == 0 || (mask & ~RAW_META_MASK_ALL) != 0) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&raw_request_lock);
	if (!raw_request_token_valid || raw_request_token != token) {
		raw_request_token = token;
		raw_request_token_valid = true;
		if ((raw_request_mask & mask & RAW_META_MASK_TCAL_POINTS) != 0
		    && raw_request_chunk != chunk) {
			raw_request_chunk = 255;
		} else if (mask & RAW_META_MASK_TCAL_POINTS) {
			raw_request_chunk = chunk;
		}
		raw_request_mask |= mask;
	}
	k_spin_unlock(&raw_request_lock, key);
	connection_signal_wake();
}

static void connection_align_mag_body(const float in[3], float out[3])
{
	float mx = in[0];
	float my = in[1];
	float mz = in[2];
	float aligned[3] = {SENSOR_MAGNETOMETER_AXES_ALIGNMENT};

	memcpy(out, aligned, sizeof(aligned));
}

static void connection_align_mag_BAinv_body(float out[4][3], const float in[4][3])
{
	float basis[3][3] = {
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f},
	};
	float R[3][3];

	for (int col = 0; col < 3; col++) {
		float aligned[3];
		connection_align_mag_body(basis[col], aligned);
		for (int row = 0; row < 3; row++) {
			R[row][col] = aligned[row];
		}
	}

	connection_align_mag_body(in[0], out[0]);

	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++) {
			float v = 0.0f;
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					v += R[row][i] * in[i + 1][j] * R[col][j];
				}
			}
			out[row + 1][col] = v;
		}
	}
}

static bool connection_raw_collection_active(void)
{
	return connection_get_data_collection() || connection_get_data_collection_batch();
}

static void connection_reset_raw_collection(bool reset_arq)
{
	k_msgq_purge(&raw_imu_msgq);
	raw_sequence = 0;
	raw_metadata_sent = false;
	raw_metadata_pending = false;
	raw_meta_cal_last_ms = 0;
	raw_cal_pending_mask = 0;
	raw_cal_pending = false;
	k_spinlock_key_t request_key = k_spin_lock(&raw_request_lock);
	atomic_clear(&raw_snapshot_ready);
	raw_request_mask = 0;
	raw_request_chunk = 0;
	raw_request_token_valid = false;
	k_spin_unlock(&raw_request_lock, request_key);
	raw_collection_start_ms = k_uptime_get();
	atomic_inc(&raw_collection_session);
	k_spinlock_key_t mag_key = k_spin_lock(&latest_mag_lock);
	latest_mag_valid = false;
	k_spin_unlock(&latest_mag_lock, mag_key);
	if (reset_arq) {
		memset(raw_ring_valid_bits, 0, sizeof(raw_ring_valid_bits));
		unsigned key = irq_lock();
		raw_retx_reset(&raw_retx);
		irq_unlock(key);
		raw_retx_total = 0;
	}
}
void connection_set_data_collection(bool enable)
{
	bool was_active = connection_get_data_collection();
	if (enable && connection_get_data_collection_batch()) {
		atomic_set(&data_collection_batch_active, 0);
		data_collection_batch_rate_hz = 0;
		sensor_set_batch_collect(false, 0.0f);
		test_mode_set_target_tps(0);
	}
	if (!enable) atomic_set(&data_collection_active, 0);
	if (enable && !was_active) connection_reset_raw_collection(true);
	if (!enable && was_active) {
		k_spinlock_key_t mag_key = k_spin_lock(&latest_mag_lock);
		latest_mag_valid = false;
		k_spin_unlock(&latest_mag_lock, mag_key);
	}
	if (enable) atomic_set(&data_collection_active, 1);
	LOG_INF("Data collection %s", enable ? "STARTED" : "STOPPED");
}

bool connection_get_data_collection(void)
{
	return atomic_get(&data_collection_active) != 0;
}

int connection_set_data_collection_batch(bool enable, uint16_t rate_hz)
{
	bool was_active = connection_get_data_collection_batch();
	if (enable && was_active) {
		if (data_collection_batch_rate_hz != rate_hz) {
			LOG_WRN("Stop batch collection before changing its rate");
			return -EBUSY;
		}
		/* Same-rate retries must not change the session or sensor accumulator. */
		return 0;
	}
	if (enable && connection_get_data_collection()) {
		connection_set_data_collection(false);
	}
	if (!enable) {
		atomic_set(&data_collection_batch_active, 0);
		data_collection_batch_rate_hz = 0;
		sensor_set_batch_collect(false, 0.0f);
		test_mode_set_target_tps(0);
	} else {
		data_collection_batch_rate_hz = rate_hz;
		if (!was_active) {
			connection_reset_raw_collection(false);
		}
		atomic_set(&data_collection_batch_active, 1);
		sensor_set_batch_collect(true, (float)rate_hz);
		test_mode_set_target_tps(DC_BATCH_FUSION_TPS);
		if (rate_hz == 0) LOG_INF("Batch data collection STARTED at accelerometer ODR");
		else LOG_INF("Batch data collection STARTED at %u Hz", rate_hz);
	}
	if (!enable) {
		LOG_INF("Batch data collection STOPPED");
	}
	return 0;
}

bool connection_get_data_collection_batch(void)
{
	return atomic_get(&data_collection_batch_active) != 0;
}

uint16_t connection_get_data_collection_batch_rate(void)
{
	return data_collection_batch_rate_hz;
}


void connection_set_ota_suppressed(bool suppressed)
{
	ota_suppressed = suppressed;
	if (suppressed) {
		ota_suppress_start_time = k_uptime_get();
	} else {
		ota_suppress_start_time = 0;
	}
	LOG_INF("OTA suppression %s", suppressed ? "ENABLED (slow poll)" : "DISABLED (normal poll)");
}

bool connection_get_ota_suppressed(void)
{
	return ota_suppressed;
}


bool connection_raw_collection_startup_done(void)
{
	return connection_raw_collection_active() && (k_uptime_get() - raw_collection_start_ms >= 5000);
}

void connection_queue_raw_sample(const struct raw_imu_sample *sample)
{
	uint32_t session = (uint32_t)atomic_get(&raw_collection_session);
	/* The raw gyro integrator intentionally continues through warmup; only
	 * producer admission is gated, so the first transmitted sample is seq 0
	 * after five seconds without a backlog burst. */
	if (!connection_raw_collection_active() || !connection_raw_collection_startup_done()) return;
	struct raw_imu_queued entry;
	memcpy(entry.gyr_quat, sample->gyr_quat, sizeof(entry.gyr_quat));
	memcpy(entry.accel, sample->accel, sizeof(entry.accel));
	entry.temp_c = sample->temp_c;
	entry.session = session;
	if (k_msgq_put(&raw_imu_msgq, &entry, K_NO_WAIT) != 0) {
		struct raw_imu_queued discard;
		k_msgq_get(&raw_imu_msgq, &discard, K_NO_WAIT);
		k_msgq_put(&raw_imu_msgq, &entry, K_NO_WAIT);
	}
	connection_signal_wake();
}

void connection_queue_raw_mag(const float mag[3])
{
	if (!connection_raw_collection_active()) {
		return;
	}
	uint32_t session = (uint32_t)atomic_get(&raw_collection_session);

	float aligned[3];
	connection_align_mag_body(mag, aligned);
	k_spinlock_key_t key = k_spin_lock(&latest_mag_lock);
	if (connection_raw_collection_active()
	    && session == (uint32_t)atomic_get(&raw_collection_session)) {
		memcpy(latest_mag, aligned, sizeof(latest_mag));
		latest_mag_session = session;
		latest_mag_valid = true;
	}
	k_spin_unlock(&latest_mag_lock, key);
}

void connection_send_raw_metadata(
	float gyro_range, float accel_range, float gyro_odr, float accel_odr,
	float mag_odr, uint8_t imu, uint8_t mag, float chip_gyro_hz, float fusion_gyro_hz)
{
	/* The sensor thread publishes one immutable snapshot per session. */
	k_spinlock_key_t key = k_spin_lock(&raw_request_lock);
	if (!connection_raw_collection_active() || atomic_get(&raw_snapshot_ready)) {
		k_spin_unlock(&raw_request_lock, key);
		return;
	}
	memset(raw_metadata_buf, 0, sizeof(raw_metadata_buf));
	raw_metadata_buf[0] = ESB_RAW_META_TYPE;
	raw_metadata_buf[1] = tracker_id;
	memcpy(&raw_metadata_buf[2], &gyro_range, 4);
	memcpy(&raw_metadata_buf[6], &accel_range, 4);
	memcpy(&raw_metadata_buf[10], &gyro_odr, 4);
	memcpy(&raw_metadata_buf[14], &accel_odr, 4);
	memcpy(&raw_metadata_buf[18], &mag_odr, 4);
	raw_metadata_buf[22] = imu;
	raw_metadata_buf[23] = mag;
	memcpy(&raw_metadata_buf[24], &chip_gyro_hz, 4);
	memcpy(&raw_metadata_buf[28], &fusion_gyro_hz, 4);
	connection_capture_calibration_snapshot(true);
	raw_request_mask = RAW_META_MASK_ALL;
	raw_request_chunk = 255;
	atomic_set(&raw_snapshot_ready, 1);
	k_spin_unlock(&raw_request_lock, key);
	connection_signal_wake();
}

static bool connection_send_calibration_mask(uint8_t mask, uint8_t chunk)
{
	uint8_t buf[RAW_PACKET_SIZE] = {0};
	buf[0] = ESB_RAW_CAL_TYPE;
	buf[1] = tracker_id;
	uint8_t subtype_mask = mask & RAW_META_MASK_ALL;
	if (subtype_mask & RAW_META_MASK_ACCEL) {
		buf[2] = RAW_CAL_SUB_ACCEL;
		memcpy(&buf[3], raw_cal_snapshot.acc_BAinv, sizeof(raw_cal_snapshot.acc_BAinv));
		subtype_mask = RAW_META_MASK_ACCEL;
	} else if (subtype_mask & RAW_META_MASK_MAG) {
		buf[2] = RAW_CAL_SUB_MAG;
		memcpy(&buf[3], raw_cal_snapshot.mag_BAinv, sizeof(raw_cal_snapshot.mag_BAinv));
		subtype_mask = RAW_META_MASK_MAG;
	} else if (subtype_mask & RAW_META_MASK_GYRO) {
		buf[2] = RAW_CAL_SUB_GYRO;
		memcpy(&buf[3], raw_cal_snapshot.gyro_bias, sizeof(raw_cal_snapshot.gyro_bias));
		memcpy(&buf[15], raw_cal_snapshot.gyro_scale, sizeof(raw_cal_snapshot.gyro_scale));
		subtype_mask = RAW_META_MASK_GYRO;
	} else if (subtype_mask & RAW_META_MASK_TCAL_STATE) {
		buf[2] = RAW_CAL_SUB_TCAL;
		buf[3] = raw_cal_snapshot.tcal_flags;
		memcpy(&buf[4], &raw_cal_snapshot.tcal_count, 2);
		memcpy(&buf[6], &raw_cal_snapshot.tcal_temp_min, 4);
		memcpy(&buf[10], &raw_cal_snapshot.tcal_temp_max, 4);
		buf[14] = raw_cal_snapshot.tcal_apply_mode;
		subtype_mask = RAW_META_MASK_TCAL_STATE;
	} else if (subtype_mask & RAW_META_MASK_TCAL_POINTS) {
#if CONFIG_SENSOR_USE_TCAL
		uint16_t count = raw_cal_snapshot.tcal_count;
		uint16_t start = chunk == 255 ? 0 : (uint16_t)chunk * 2U;
		if (count == 0 || start >= count) return true;
		uint8_t n = (uint8_t)((count - start) > 2 ? 2 : count - start);
		buf[2] = RAW_CAL_SUB_TCAL_POINTS;
		buf[3] = chunk == 255 ? 0 : chunk;
		memcpy(&buf[4], &count, 2);
		buf[6] = n;
		memcpy(&buf[7], &raw_cal_snapshot.tcal_points[start], (size_t)n * sizeof(struct TempCalPoint));
#else
		buf[2] = RAW_CAL_SUB_TCAL_POINTS;
		buf[3] = 0;
		buf[4] = 0;
		buf[5] = 0;
		buf[6] = 0;
#endif
		subtype_mask = RAW_META_MASK_TCAL_POINTS;
	} else {
		return true;
	}
	if (esb_write(buf, false, RAW_PACKET_SIZE) != 0) return false;
	return true;
}

static bool connection_process_calibration_requests(void)
{
	if (!raw_cal_pending) {
		return false;
	}
	uint8_t selected = raw_cal_pending_mask;
	if (selected == 0) {
		raw_cal_pending = false;
		return false;
	}
	/* Ascending bits send state before its point blocks. */
	uint8_t subtype = selected & (uint8_t)(0U - selected);
	if (!connection_send_calibration_mask(subtype, raw_cal_pending_chunk)) {
		return false;
	}
	if (subtype == RAW_META_MASK_TCAL_POINTS && raw_cal_points_all
	    && (uint16_t)(raw_cal_pending_chunk + 1U) * 2U < raw_cal_snapshot.tcal_count) {
		raw_cal_pending_chunk++;
	} else {
		raw_cal_pending_mask &= (uint8_t)~subtype;
		if (subtype == RAW_META_MASK_TCAL_POINTS) {
			raw_cal_points_all = false;
		}
	}
	raw_cal_pending = raw_cal_pending_mask != 0;
	return true;
}

bool connection_process_raw_data(void)
{
	if (!connection_raw_collection_active() || !atomic_get(&raw_snapshot_ready)) {
		return false;
	}
	if (!raw_metadata_pending && !raw_cal_pending) {
		k_spinlock_key_t key = k_spin_lock(&raw_request_lock);
		uint8_t mask = raw_request_mask;
		uint8_t chunk = raw_request_chunk;
		raw_request_mask = 0;
		k_spin_unlock(&raw_request_lock, key);
		if (mask) {
			connection_schedule_calibration(mask, chunk);
		}
	}

	/* Priority 1: Process retransmit requests from ARQ ACK payloads */
	uint16_t retx_seq = 0;
	bool have_retx = false;
	if (!connection_get_data_collection_batch()) {
		unsigned irq_key = irq_lock();
		have_retx = raw_retx_take(&raw_retx, &retx_seq);
		irq_unlock(irq_key);
	}
	if (have_retx) {
		uint8_t *packet = raw_ring_get(retx_seq);
		if (packet != NULL) {
			/* Retransmit from ring buffer */
			int err = esb_write(packet, false, RAW_PACKET_SIZE);
			if (err != 0) {
				k_msleep(1);
			}
			raw_retx_total++;
		}
		return true;
	}

	/* Priority 2: metadata and stable calibration requests. Admission failures
	 * leave the exact packet selected and are retried after the pacing interval. */
	if (raw_metadata_pending || raw_cal_pending) {
		int64_t now = k_uptime_get();
		int64_t drip_ms = connection_raw_collection_startup_done() ? 200 : 50;
		if (now - raw_meta_cal_last_ms >= drip_ms) {
			bool sent;
			if (raw_metadata_pending) {
				sent = esb_write(raw_metadata_buf, false, RAW_PACKET_SIZE) == 0;
				if (sent) { raw_metadata_pending = false; raw_metadata_sent = true; }
			} else {
				sent = connection_process_calibration_requests();
			}
			raw_meta_cal_last_ms = now;
			if (sent) return true;
			k_msleep(1);
		}
	}

	/* Wait for metadata before sending data */
	/* Metadata may be queued immediately, but raw samples are not admitted
	 * until the complete five-second startup window has elapsed. */
	if (!connection_raw_collection_startup_done()) return false;
	if (!raw_metadata_sent) return false;
	/* Priority 3: Send new IMU sample */
	struct raw_imu_queued sample;
	if (k_msgq_get(&raw_imu_msgq, &sample, K_NO_WAIT) == 0) {
		/* A collection reset purges the queue, but a producer can be
		 * preempted between its active check and k_msgq_put(). Reject such
		 * a stale sample instead of leaking it into the new session. */
		if (sample.session != (uint32_t)atomic_get(&raw_collection_session)) {
			return true;
		}

		uint8_t buf[RAW_PACKET_SIZE];
		memset(buf, 0, sizeof(buf));

		buf[0] = ESB_RAW_IMU_QUAT_TYPE;
		buf[1] = tracker_id;
		uint16_t seq = raw_sequence++;
		sys_put_be16(seq, &buf[2]);

		/* GyrQuat float × 4 (w, x, y, z) */
		memcpy(&buf[4], &sample.gyr_quat[0], 4);
		memcpy(&buf[8], &sample.gyr_quat[1], 4);
		memcpy(&buf[12], &sample.gyr_quat[2], 4);
		memcpy(&buf[16], &sample.gyr_quat[3], 4);
		/* Accel float × 3 */
		memcpy(&buf[20], &sample.accel[0], 4);
		memcpy(&buf[24], &sample.accel[1], 4);
		memcpy(&buf[28], &sample.accel[2], 4);
		uint8_t flags = 0;
		float mag_snap[3] = {0};
		bool mag_ok = false;
		{
			k_spinlock_key_t key = k_spin_lock(&latest_mag_lock);
			if (latest_mag_valid && latest_mag_session == (uint32_t)atomic_get(&raw_collection_session)) {
				memcpy(mag_snap, latest_mag, sizeof(mag_snap));
				mag_ok = true;
			}
			latest_mag_valid = false;
			k_spin_unlock(&latest_mag_lock, key);
		}
		if (mag_ok) {
			memcpy(&buf[32], &mag_snap[0], 4);
			memcpy(&buf[36], &mag_snap[1], 4);
			memcpy(&buf[40], &mag_snap[2], 4);
			flags |= 0x01;
		}
		buf[44] = flags;
		memcpy(&buf[45], &sample.temp_c, sizeof(sample.temp_c));

		/* Only reliable single-target collection retains ARQ history. */
		if (!connection_get_data_collection_batch()) {
			raw_ring_store(buf);
		}

		int err = esb_write(buf, false, RAW_PACKET_SIZE);
		if (err != 0) {
			/* Batch mode intentionally drops FIFO/admission failures; all
			 * failed sends still yield before the next queued sample. */
			k_msleep(1);
		}
		return true;
	}

	return false;
}

/* Exact test-mode rate schedule in local monotonic microseconds. TDMA still
 * owns slot admission; this schedule only selects which slot opportunities
 * carry a test packet. Fractional microseconds are accumulated so arbitrary
 * TPS values have no integer-ms bias. */
static struct {
	uint64_t next_due_us;
	uint32_t remainder;
	uint16_t tps;
} test_rate_schedule;
static uint64_t test_wake_delay_us;
static bool test_wake_delay_valid;

static void test_rate_schedule_sync(uint64_t now_us)
{
	uint16_t tps = test_mode_effective_tps();
	if (tps == 0) {
		memset(&test_rate_schedule, 0, sizeof(test_rate_schedule));
		return;
	}
	if (test_rate_schedule.tps != tps || test_rate_schedule.next_due_us == 0
		|| test_rate_schedule.next_due_us + 1000000ULL < now_us) {
		test_rate_schedule.tps = tps;
		test_rate_schedule.next_due_us = now_us;
		test_rate_schedule.remainder = 0;
	}
}

static bool test_rate_due(uint64_t now_us)
{
	test_rate_schedule_sync(now_us);
	return test_rate_schedule.tps != 0 && now_us >= test_rate_schedule.next_due_us;
}

static void test_rate_advance(uint64_t now_us)
{
	uint16_t tps = test_rate_schedule.tps;
	if (tps == 0) {
		return;
	}
	do {
		test_rate_schedule.next_due_us += 1000000U / tps;
		test_rate_schedule.remainder += 1000000U % tps;
		if (test_rate_schedule.remainder >= tps) {
			test_rate_schedule.next_due_us++;
			test_rate_schedule.remainder -= tps;
		}
	} while (test_rate_schedule.next_due_us <= now_us);
}

static uint64_t test_rate_delay_us(uint64_t now_us)
{
	test_rate_schedule_sync(now_us);
	return test_rate_schedule.next_due_us > now_us
		? test_rate_schedule.next_due_us - now_us : 0;
}

/* Lookahead window: if a low-freq packet is within this many ms of being due,
 * piggyback it onto the current transmission as a composite sub-packet. */
#define COMPOSITE_LOOKAHEAD_MS 10

/* Max sub-packet payload in a composite: ESB_MAX_PAYLOAD_LEN - 3 (header) - 1 (sequence) */
#define COMPOSITE_MAX_SUB_DATA (ESB_MAX_PAYLOAD_LEN - 4)

/*
 * Build and send a composite packet identified by ESB_COMPOSITE_TYPE
 * containing multiple sub-packets.
 * types[] / lens[] describe the sub-packets to include; n is the count.
 * Each sub-packet is: [type_byte][data...].
 * Format: [ESB_COMPOSITE_TYPE][tracker_id][sub_count][sub0_type][sub0_data...]
 *         [sub1_type][sub1_data...]...[sequence]
 */
static bool send_composite(const struct composite_builder *builder)
{
	uint8_t buf[ESB_MAX_PAYLOAD_LEN];
	int pos = 0;
	/* An event is only a tail of real business data, never a replacement pose.
	 * Reject an empty builder without completing its event token: the event
	 * remains pending for a later pose (or a normal-mode standalone send). */
	if (builder->n <= 0 || builder->n > ARRAY_SIZE(builder->types)) {
		return false;
	}
	buf[pos++] = ESB_COMPOSITE_TYPE;
	buf[pos++] = tracker_id;
	buf[pos++] = (uint8_t)(builder->n + builder->has_event);
	for (int i = 0; i < builder->n; i++) {
		const struct sub_packet_desc *d = sub_packet_get(builder->types[i]);
		if (!d || pos + 1 + d->data_len + 1 > sizeof(buf)) {
			return false;
		}
		buf[pos++] = builder->types[i];
		pos += d->fill(&buf[pos]);
	}
	if (builder->has_event) {
		if (pos + 16 + 1 > sizeof(buf)) {
			return false;
		}
		buf[pos++] = TRACKER_EVENT_ESB_TYPE;
		memcpy(&buf[pos], &builder->event.packet[2], 15);
		pos += 15;
	}
	buf[pos++] = packet_sequence;
	int err = esb_write(buf, no_ack, pos);
	if (builder->has_event) {
		tracker_events_complete(builder->event.token, err == 0, k_uptime_get_32());
	}
	if (err != 0) {
		return false;
	}
	packet_sequence++;
	return true;
}

static void composite_builder_reset(struct composite_builder *builder)
{
	builder->n = 0;
	builder->used = 0;
	builder->has_event = false;
	memset(builder->last_times, 0, sizeof(builder->last_times));
}

/*
 * Try to append a sub-packet type to the pending composite list.
 * Returns true if it fits within COMPOSITE_MAX_SUB_DATA.
 */
static bool composite_try_add(struct composite_builder *builder, uint8_t type)
{
	int need = 1 + sub_data_len(type); /* 1 for type byte + data */
	if (!sub_packet_get(type) || builder->n >= ARRAY_SIZE(builder->types)
		|| builder->used + need > COMPOSITE_MAX_SUB_DATA) {
		return false;
	}
	builder->types[builder->n] = type;
	builder->n++;
	builder->used += need;
	return true;
}

static bool
composite_try_add_due(struct composite_builder *builder, uint8_t type, bool wanted, int64_t *last_time, int64_t now)
{
	if (!wanted) {
		return false;
	}
	if (!composite_try_add(builder, type)) {
		return false;
	}
	builder->last_times[builder->n - 1] = last_time;
	builder->stamps[builder->n - 1] = now;
	return true;
}

/* Apply deferred low-frequency last-sent timestamps once the packet that
 * carries them was actually queued; a failed send must leave them unchanged
 * so the data is retried. */
static void composite_commit_timestamps(const struct composite_builder *builder)
{
	for (int i = 0; i < builder->n; i++) {
		if (builder->last_times[i]) {
			*builder->last_times[i] = builder->stamps[i];
		}
	}
}

static bool send_composite_or_single(struct composite_builder *builder, uint8_t fallback_type)
{
	/* Private event data is never a normal sub-packet or a direct-HID mirror. */
	if (esb_ready() && builder->n > 0 && builder->used + 16 <= COMPOSITE_MAX_SUB_DATA
		&& !esb_ota_is_active() && !get_status(SYS_STATUS_CONNECTION_ERROR)) {
		builder->has_event = tracker_events_select(k_uptime_get_32(), tracker_id, &builder->event);
	}
	if (builder->n > 1 || builder->has_event) {
		if (!esb_ready()) {
#if CONFIG_CONNECTION_OVER_HID
			/* Radio unavailable: HID is the configured output path;
			 * its enqueue success is the send success. */
			return write_hid_composite_as_normal_packets(builder);
#else
			return false;
#endif
		}

		/* Radio available: only ESB queue success counts, so a failed
		 * admission cannot advance the schedule or mirror to HID. */
		if (!send_composite(builder)) {
			return false;
		}
#if CONFIG_CONNECTION_OVER_HID
		write_hid_composite_as_normal_packets(builder);
#endif
		return true;
	}
	return connection_write_packet_type(fallback_type);
}

static void connection_signal_wake(void)
{
	k_sem_give(&connection_wake_sem);
}

void connection_tracker_event_wake(void)
{
	connection_signal_wake();
}

/* True means an admission was attempted, not that the receiver received it. */
static bool connection_send_tracker_event(void)
{
	struct tracker_event_tx event;
	if (!esb_ready() || test_mode_get() || esb_ota_is_active()
		|| get_status(SYS_STATUS_CONNECTION_ERROR)
		|| !tracker_events_select(k_uptime_get_32(), tracker_id, &event)) {
		return false;
	}
	int err = esb_write(event.packet, true, sizeof(event.packet));
	tracker_events_complete(event.token, err == 0, k_uptime_get_32());
	if (err != 0) {
		/* The core defers another event attempt by at least 10ms. */
		k_msleep(1);
	}
	return true;
}

static int64_t connection_next_deadline_ms(int64_t now)
{
	int64_t deadline = now + 1000; /* bounded fallback */

	uint32_t ping_deadline = (uint32_t)atomic_get(&next_ping_deadline_ms);
	uint32_t ping_window_delay_ms;
	if (esb_ready() && tdma_ping_wake_delay_ms(&ping_window_delay_ms)) {
		int64_t guarded_deadline = now + ping_window_delay_ms;
		if (guarded_deadline < deadline) {
			deadline = guarded_deadline;
		}
	} else if (esb_ready() && (int32_t)(ping_deadline - (uint32_t)deadline) < 0) {
		deadline = ping_deadline;
	}
	test_wake_delay_valid = false;
	if (test_mode_get()) {
		uint64_t now_us = k_ticks_to_us_near64(k_uptime_ticks());
		test_wake_delay_us = test_rate_delay_us(now_us);
		test_wake_delay_valid = true;
		int64_t t_dl = now + (int64_t)((test_wake_delay_us + 999ULL) / 1000ULL);
		if (t_dl < deadline) {
			deadline = t_dl;
		}
	}
	if (!test_mode_get()) {
		/* Test mode only piggybacks events on scheduled poses. An already-due
		 * event cannot shorten that wait or it would keep this thread spinning. */
		if (esb_ready() && !esb_ota_is_active() && !get_status(SYS_STATUS_CONNECTION_ERROR)) {
			uint32_t event_deadline = tracker_events_deadline((uint32_t)now);
			if (event_deadline != UINT32_MAX) {
				int64_t e_dl = now + (int32_t)(event_deadline - (uint32_t)now);
				if (e_dl < deadline) {
					deadline = e_dl;
				}
			}
		}
		if (sensor_data_snapshot_m_pending(&sensor_data_snapshot)) {
			int64_t m_dl = last_mag_time + 100;
			if (m_dl < deadline) {
				deadline = m_dl;
			}
		}
		if (sensor_ids_set) {
			int64_t i_dl = last_info_time + 100;
			if (i_dl < deadline) {
				deadline = i_dl;
			}
		}
		int64_t s_dl = last_status_time + 1000;
		if (s_dl < deadline) {
			deadline = s_dl;
		}
		int64_t r_dl = last_runtime_time + 1000;
		if (r_dl < deadline) {
			deadline = r_dl;
		}
	}
	return deadline;
}

static void connection_idle_wait(int64_t now)
{
	int64_t wait_ms = connection_next_deadline_ms(now) - now;
	if (wait_ms <= 0) {
		(void)k_sem_take(&connection_wake_sem, K_NO_WAIT);
		return;
	}
	if (wait_ms > 1000) {
		wait_ms = 1000;
	}
	if (test_wake_delay_valid) {
		uint64_t wait_us = (uint64_t)wait_ms * 1000ULL;
		if (test_wake_delay_us < wait_us) {
			wait_us = test_wake_delay_us;
		}
		(void)k_sem_take(&connection_wake_sem, K_USEC((uint32_t)wait_us));
		return;
	}
	(void)k_sem_take(&connection_wake_sem, K_MSEC(wait_ms));
}

void connection_thread(void)
{
	/* Register connection thread with watchdog */
	watchdog_register_thread(WDT_CHANNEL_CONNECTION, 0);
	atomic_set(
		&next_ping_deadline_ms,
		(atomic_val_t)(k_uptime_get_32() + ping_phase_ms(PING_INTERVAL_MS))
	);

	while (1) {
		int64_t now = k_uptime_get();

		watchdog_feed(WDT_CHANNEL_CONNECTION);
		bool radio_ready = esb_ready();
		bool hid_ready = connection_hid_output_ready();

		/* Adaptive PING interval based on connection health */
		if (get_status(SYS_STATUS_CONNECTION_ERROR)) {
			ping_interval_ms = 1450;
		} else {
			ping_interval_ms = PING_INTERVAL_MS;
		}

		if (!radio_ready && !hid_ready) {
			k_msleep(100);
			continue;
		}

		esb_process_ota_rx_queue();
#if defined(CONFIG_TDMA_DIAGNOSTICS)
		radio_capture_process();
#endif

		if (radio_ready) {
			/* Synchronized operation derives the next PING directly from server
			 * frame time. The deterministic interval is always <=1 second and the
			 * following physical slot is reserved by every tracker. Startup and
			 * receiver-reset recovery retain the immediate unslotted path. */
			uint32_t effective_ping_interval_ms = get_ping_interval_ms();
			uint32_t ping_deadline = (uint32_t)atomic_get(&next_ping_deadline_ms);
			bool force_resync = atomic_cas(&ping_resync_requested, 1, 0);
			uint32_t ping_window_delay_ms = 0;
			bool guarded_schedule = !force_resync && tdma_ping_wake_delay_ms(&ping_window_delay_ms);
			if (guarded_schedule) {
				ping_deadline = (uint32_t)now + ping_window_delay_ms;
				ping_aligned_interval_ms = effective_ping_interval_ms;
				atomic_set(&ping_server_phase_aligned, 1);
			} else if (!force_resync) {
				if (ping_aligned_interval_ms != effective_ping_interval_ms) {
					atomic_set(&ping_server_phase_aligned, 0);
				}
				if (!atomic_get(&ping_server_phase_aligned)) {
					uint32_t phase_delay_ms = ping_server_phase_delay_ms(effective_ping_interval_ms);
					if (phase_delay_ms > 0) {
						ping_deadline = (uint32_t)now + phase_delay_ms;
						atomic_set(&next_ping_deadline_ms, (atomic_val_t)ping_deadline);
						ping_aligned_interval_ms = effective_ping_interval_ms;
						atomic_set(&ping_server_phase_aligned, 1);
					}
				}
			}

			bool ping_due = force_resync
				|| (guarded_schedule ? ping_window_delay_ms == 0
					: (int32_t)((uint32_t)now - ping_deadline) >= 0);
			if (ping_due) {
				uint8_t ping[ESB_PING_LEN] = {0};
				ping[0] = ESB_PING_TYPE;
				ping[1] = connection_get_id();
				uint8_t ping_ack_flag = esb_get_ping_ack_flag();
				ping[7] = ping_ack_flag;
				if (ping_ack_flag == ESB_PONG_FLAG_TEST_MODE_ON) {
					uint16_t tps = test_mode_get_target_tps();
					ping[8] = (tps >> 8) & 0xFF;
					ping[9] = tps & 0xFF;
				} else if (ping_ack_flag == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON) {
					ping[8] = (uint8_t)connection_get_data_collection_batch_rate();
				} else if (ping_ack_flag == ESB_PONG_FLAG_DATA_COLLECT_METADATA) {
					uint8_t request[4];
					esb_get_ping_request_data(request);
					memcpy(&ping[8], request, sizeof(request));
				}
				int err = esb_write_ping(ping, force_resync);
				if (err == -EAGAIN) {
					uint32_t retry_delay_ms = 0;
					if (tdma_ping_wake_delay_ms(&retry_delay_ms)) {
						atomic_set(&next_ping_deadline_ms, (atomic_val_t)((uint32_t)now + retry_delay_ms));
					}
					continue;
				}
				atomic_inc(&ping_sched_stats.due);
				atomic_inc(&ping_sched_stats.attempts);
				ping_stats_attempt((uint32_t)now);
				if (!guarded_schedule) {
					ping_deadline = ping_next_periodic_deadline(
						ping_deadline, (uint32_t)now, effective_ping_interval_ms
					);
					atomic_set(&next_ping_deadline_ms, (atomic_val_t)ping_deadline);
				}
				if (err == 0) {
					atomic_inc(&ping_sched_stats.queue_ok);
				} else {
					atomic_inc(&ping_sched_stats.queue_fail);
					atomic_inc(&ping_sched_stats.retry_deferred);
					atomic_set(&ping_server_phase_aligned, 0);
					uint32_t retry_delay_ms = PING_QUEUE_RETRY_MS;
					if (guarded_schedule) {
						(void)tdma_ping_wake_delay_ms(&retry_delay_ms);
					}
					atomic_set(&next_ping_deadline_ms, (atomic_val_t)((uint32_t)now + retry_delay_ms));
				}
				continue;
			}

			/*
			 * ESB OTA mode: when active, stop sending sensor data and instead
			 * send frequent OTA status/poll packets. The receiver responds
			 * with OTA data in the ACK payload.
			 */
			if (esb_ota_is_active()) {
				esb_ota_check_timeout();
				esb_ota_periodic_status();
				k_usleep(1500);
				continue;
			}

			/*
			 * OTA suppression: when another tracker is being updated,
			 * this tracker reduces its poll rate to free radio bandwidth.
			 */
			if (ota_suppressed) {
				/* Safety timeout: auto-unsuppress after timeout */
				if (ota_suppress_start_time > 0 && (now - ota_suppress_start_time) > OTA_SUPPRESS_TIMEOUT_MS) {
					LOG_WRN("OTA suppress timeout, auto-unsuppressing");
					connection_set_ota_suppressed(false);
				} else {
					k_msleep(100); /* ~10 Hz poll rate */
				}
			}

			/* Connection-loss shutdown is independent of metadata repair. */
			if (get_status(SYS_STATUS_CONNECTION_ERROR)) {
				if (connection_raw_collection_active()) {
					if (dc_conn_error_start == 0) {
						dc_conn_error_start = now;
					} else if (now - dc_conn_error_start > 60000) {
						connection_set_data_collection(false);
						connection_set_data_collection_batch(false, 0);
						test_mode_set(false);
						dc_conn_error_start = 0;
						LOG_WRN("Data collection auto-stopped (connection error for 60s)");
					}
				}
				k_msleep(100);
				continue;
			}
			dc_conn_error_start = 0;

			/* Raw streams yield one admission to a due event; test mode may
			 * only carry events on its scheduled real pose packet. */
			if (connection_raw_collection_active() && connection_send_tracker_event()) {
				continue;
			}
			/* Otherwise raw data retains its existing priority. */
			if (connection_process_raw_data()) {
				continue;
			}
		} // end of radio_ready block

		/* During data collection, throttle fusion data
		 * to leave radio bandwidth for raw data. */
		if (radio_ready && connection_raw_collection_active()) {
			static int64_t last_fusion_dc_time;
			if (now - last_fusion_dc_time < 9) {
				k_usleep(300);
				continue;
			}
			last_fusion_dc_time = now;
		}

		/* TDMA is the sole frame scheduler. In normal mode a fresh snapshot
		 * enters admission immediately. In test mode the configured interval
		 * is both floor and ceiling: publications only refresh the latest
		 * snapshot; exactly one packet enters admission when the interval is
		 * due. */
		bool pending = sensor_data_snapshot_qa_pending(&sensor_data_snapshot);
		bool in_test_mode = test_mode_get();
		if (!in_test_mode && test_rate_schedule.tps != 0) {
			memset(&test_rate_schedule, 0, sizeof(test_rate_schedule));
		}
		uint64_t now_us = k_ticks_to_us_near64(k_uptime_ticks());
		bool test_send_due = in_test_mode && test_rate_due(now_us);
		bool quat_ready = in_test_mode ? test_send_due : pending;
		/* In test mode all low-frequency data rides the next target-rate
		 * packet. Standalone sends would violate the configured ceiling. */
		bool mag_due = !in_test_mode && sensor_data_snapshot_m_pending(&sensor_data_snapshot)
			&& (now - last_mag_time >= 100);
		bool info_due = !in_test_mode && sensor_ids_set && (now - last_info_time >= 100);
		bool status_due = !in_test_mode && (now - last_status_time >= 1000);
		bool runtime_due = !in_test_mode && (now - last_runtime_time >= 1000);

		/* Low-frequency fields may always piggyback on a quat/test packet. */
		bool info_soon = sensor_ids_set && (now - last_info_time > 100 - COMPOSITE_LOOKAHEAD_MS);
		bool status_soon = (now - last_status_time > 1000 - COMPOSITE_LOOKAHEAD_MS);
		bool runtime_soon = (now - last_runtime_time > 1000 - COMPOSITE_LOOKAHEAD_MS);
		bool mag_soon = sensor_data_snapshot_m_pending(&sensor_data_snapshot)
			&& (now - last_mag_time > 100 - COMPOSITE_LOOKAHEAD_MS);
		bool status_wanted = status_due || status_soon;
		bool runtime_wanted = runtime_due || runtime_soon;
		bool info_wanted = info_due || info_soon;
		bool mag_wanted = mag_due || mag_soon;

		if (quat_ready) {
			struct composite_builder builder;
			uint8_t fallback_type = SUB_PACKET_QUAT_ACCEL;
			composite_builder_reset(&builder);

			/* Primary: quat sub-packet */
			if (mag_wanted) {
				/* mag includes full quat, use type 4 instead of separate quat+mag */
				composite_try_add_due(&builder, SUB_PACKET_QUAT_MAG, true, &last_mag_time, now);
				fallback_type = SUB_PACKET_QUAT_MAG;
			} else if (!connection_sensor_get_precise_quat() && info_wanted) {
				/* compact quat (type 2) contains batt/temp but NOT imu_id/mag_id.
				 * Don't update last_info_time here so that a real type 0 info
				 * sub-packet is still piggybacked to keep IMU model visible. */
				composite_try_add(&builder, SUB_PACKET_COMPACT_QUAT);
				fallback_type = SUB_PACKET_COMPACT_QUAT;
			} else {
				composite_try_add(&builder, SUB_PACKET_QUAT_ACCEL);
			}

			/* Piggyback low-freq sub-packets if they fit */
			composite_try_add_due(&builder, SUB_PACKET_STATUS, status_wanted, &last_status_time, now);
			composite_try_add_due(&builder, SUB_PACKET_RUNTIME, runtime_wanted, &last_runtime_time, now);
			composite_try_add_due(&builder, SUB_PACKET_INFO, info_wanted, &last_info_time, now);

			if (send_composite_or_single(&builder, fallback_type)) {
				/* Only a successfully queued packet consumes the test-rate
				 * slot; missed ESB admission is retried without advancing
				 * the schedule. */
				if (in_test_mode) {
					test_rate_advance(k_ticks_to_us_near64(k_uptime_ticks()));
				}
				composite_commit_timestamps(&builder);
			} else {
				k_msleep(1);
			}
			continue;
		}

		/* No quat ready: all low-frequency packets may carry an event tail. */
		if (mag_due || info_due || status_due || runtime_due) {
			struct composite_builder builder;
			composite_builder_reset(&builder);
			uint8_t primary = mag_due ? SUB_PACKET_QUAT_MAG
				: info_due ? SUB_PACKET_INFO
				: status_due ? SUB_PACKET_STATUS : SUB_PACKET_RUNTIME;
			if (mag_due) {
				composite_try_add_due(&builder, SUB_PACKET_QUAT_MAG, true, &last_mag_time, now);
			} else if (info_due) {
				composite_try_add_due(&builder, SUB_PACKET_INFO, true, &last_info_time, now);
			}
			composite_try_add_due(&builder, SUB_PACKET_STATUS, status_wanted, &last_status_time, now);
			composite_try_add_due(&builder, SUB_PACKET_RUNTIME, runtime_wanted, &last_runtime_time, now);
			if (send_composite_or_single(&builder, primary)) {
				composite_commit_timestamps(&builder);
			} else {
				k_msleep(1);
			}
			continue;
		}
		if (connection_send_tracker_event()) {
			continue;
		}

		connection_idle_wait(now);
	}
}
