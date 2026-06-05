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
#ifndef SLIMENRF_ESB
#define SLIMENRF_ESB

#include <esb.h>
#include <nrfx_timer.h>

// TODO: timer?
#define LAST_RESET_LIMIT 10
extern uint8_t last_reset;
// TODO: move to esb/timer
// extern const nrfx_timer_t m_timer;
extern bool esb_state;
extern bool timer_state;

// TODO: esb/sensor?
extern uint16_t led_clock;
extern uint32_t led_clock_offset;

void esb_write_ack(uint8_t type);
void event_handler(struct esb_evt const *event);
int clocks_start(void);
void clocks_stop(void);
void clocks_request_start(uint32_t delay_us);
void clocks_request_stop(uint32_t delay_us);
int esb_initialize(bool);
void esb_deinitialize(void);

void esb_set_addr_discovery(void);
void esb_set_addr_paired(void);

void esb_set_pair(uint64_t addr);

void esb_pair(void);
void esb_reset_pair(void);
void esb_clear_pair(void);

void esb_process_ota_rx_queue(void);
void esb_write(uint8_t *data, bool no_ack, size_t data_length); // TODO: give packets some names

#define PING_INTERVAL_MS 997
// Ping/Pong types for ACK payload validation
#define ESB_PING_TYPE 0xF0
#define ESB_PONG_TYPE 0xF1

// Ping/Pong packet sizes
#define ESB_PING_LEN 13 // with CRC-8
#define ESB_PONG_LEN 13 // with CRC-8
#define ESB_SENSOR_DATA_LEN 17
#define ESB_MAX_PAYLOAD_LEN CONFIG_ESB_MAX_PAYLOAD_LENGTH
#define ESB_COMPOSITE_TYPE 0xFE // Composite packet containing multiple sub-packets

// Remote command flags for PONG data[7] (shared with receiver)
#define ESB_PONG_FLAG_NORMAL 0x00
#define ESB_PONG_FLAG_SHUTDOWN 0x01
#define ESB_PONG_FLAG_CALIBRATE 0x02     // Trigger gyro/accel ZRO calibration
#define ESB_PONG_FLAG_SIX_SIDE_CAL 0x03  // Trigger 6-point accelerometer calibration
#define ESB_PONG_FLAG_MEOW 0x04          // Trigger meow output
#define ESB_PONG_FLAG_SCAN 0x05          // Trigger sensor scan
#define ESB_PONG_FLAG_MAG_CLEAR 0x06     // Clear magnetometer calibration
#define ESB_PONG_FLAG_REBOOT 0x07        // Reboot tracker
#define ESB_PONG_FLAG_CLEAR 0x08         // Clear pairing data
#define ESB_PONG_FLAG_DFU 0x09           // Enter DFU bootloader
#define ESB_PONG_FLAG_SET_CHANNEL 0x0A   // Set RF channel (data[8-11] contains channel value)
#define ESB_PONG_FLAG_CLEAR_CHANNEL 0x0B // Clear RF channel setting (restore default)
#define ESB_PONG_FLAG_SENS_SET 0x0C
#define ESB_PONG_FLAG_SENS_RESET 0x0D
#define ESB_PONG_FLAG_RESET_ZRO 0x0E
#define ESB_PONG_FLAG_RESET_ACC 0x0F
#define ESB_PONG_FLAG_RESET_BAT 0x10
#define ESB_PONG_FLAG_PING 0x11
#define ESB_PONG_FLAG_RESET_TCAL 0x12
#define ESB_PONG_FLAG_TCAL_AUTO_ON 0x13  // Enable T-Cal auto-calibration
#define ESB_PONG_FLAG_TCAL_AUTO_OFF 0x14 // Disable T-Cal auto-calibration
#define ESB_PONG_FLAG_FUSION_RESET 0x15  // Reset fusion (invalidate quaternion)
#define ESB_PONG_FLAG_TCAL_BOOT_ON 0x16  // Enable T-Cal boot calibration
#define ESB_PONG_FLAG_TCAL_BOOT_OFF 0x17 // Disable T-Cal boot calibration
#define ESB_PONG_FLAG_MAG_CAL 0x18       // Trigger magnetometer calibration
#define ESB_PONG_FLAG_MAG_ON 0x19        // Enable magnetometer
#define ESB_PONG_FLAG_MAG_OFF 0x1A       // Disable magnetometer
#define ESB_PONG_FLAG_TCAL_ON 0x1B       // Enable T-Cal (temperature calibration)
#define ESB_PONG_FLAG_TCAL_OFF 0x1C      // Disable T-Cal (temperature calibration)
#define ESB_PONG_FLAG_TDMA_ON 0x1D       // Enable TDMA scheduling
#define ESB_PONG_FLAG_TDMA_OFF 0x1E      // Disable TDMA scheduling
#define ESB_PONG_FLAG_TEST_MODE_ON 0x1F  // Enable battery drain test mode
#define ESB_PONG_FLAG_TEST_MODE_OFF 0x20 // Disable battery drain test mode
#define ESB_PONG_FLAG_DFU_OTA 0x21       // Enter OTA DFU bootloader
#define ESB_PONG_FLAG_DATA_COLLECT_ON 0x22  // Start raw data collection
#define ESB_PONG_FLAG_DATA_COLLECT_OFF 0x23 // Stop raw data collection
#define ESB_PONG_FLAG_SENS_AUTO 0x24        // Auto-calibrate gyro sensitivity
#define ESB_PONG_FLAG_OTA_QUERY_INFO 0x30   // Request firmware info for ESB OTA
#define ESB_PONG_FLAG_OTA_ABORT 0x31        // Abort ESB OTA update
#define ESB_PONG_FLAG_OTA_SUPPRESS 0x32     // Suppress tracker during OTA (reduce poll rate)
#define ESB_PONG_FLAG_OTA_UNSUPPRESS 0x33   // Resume normal poll rate after OTA

// Raw data collection packet types
#define ESB_RAW_IMU_TYPE    0x10  // Raw IMU data (float, with piggybacked mag)
#define ESB_RAW_MAG_TYPE    0x11  // Raw magnetometer data (float, reserved)
#define ESB_RAW_META_TYPE   0x12  // Metadata (ODR, range, sensor IDs - sent once)
#define ESB_RAW_IMU_QUAT_TYPE 0x13  // Raw IMU with gyrQuat (52 bytes, packet-loss resistant)
#define ESB_RAW_CAL_TYPE    0x14  // Extended calibration metadata (sub-typed)

// ESB_RAW_CAL_TYPE sub-types (byte[2] of 0x14 packet)
#define RAW_CAL_SUB_ACCEL   0x01  // Accel calibration: accBAinv[4][3] (48 bytes)
#define RAW_CAL_SUB_MAG     0x02  // Mag calibration: magBAinv[4][3] (48 bytes)
#define RAW_CAL_SUB_GYRO    0x03  // Gyro cal: gyroBias[3] + gyroSensScale[3] (24 bytes)
#define RAW_CAL_SUB_TCAL    0x04  // T-Cal state: enabled, count, temp range, correction offset
#define RAW_CAL_SUB_TCAL_POINTS 0x05  // T-Cal raw points (chunked, 2 per packet)

// ESB OTA packet types (used during firmware update over ESB)
#define ESB_OTA_DATA_TYPE       0x20  // OTA firmware data (receiver → tracker)
#define ESB_OTA_STATUS_TYPE     0x21  // OTA status report (tracker → receiver)
#define ESB_OTA_FW_INFO_TYPE    0x22  // Firmware info report (tracker → receiver)
#define ESB_OTA_BEGIN_TYPE      0x23  // Begin OTA session (receiver → tracker)
#define ESB_OTA_VERIFY_TYPE     0x24  // Request CRC verification (receiver → tracker)
#define ESB_OTA_ACTIVATE_TYPE   0x25  // Activate new firmware (receiver → tracker)

bool esb_ready(void);

// Get remote command flag to echo back in PING
uint8_t esb_get_ping_ack_flag(void);

// Additional delay applied to the base ping interval after repeated failures.
uint32_t esb_get_ping_backoff_ms(void);

// Get estimated current server time in ticks (0 if not synced) - high precision
uint64_t esb_get_server_time_ticks_64(void);

// Get estimated current server time in microseconds (0 if not synced)
uint64_t esb_get_server_time_us_64(void);

// Get estimated current server time in milliseconds (0 if not synced)
uint32_t esb_get_server_time(void);

// Get time since last successful PONG sync in milliseconds (-1 if never synced)
int64_t esb_get_sync_age_ms(void);

// Helper: log esb_write call frequency
void esb_write_rate_tick(void);

#endif
