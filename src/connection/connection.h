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
#ifndef SLIMENRF_CONNECTION
#define SLIMENRF_CONNECTION
#include <stdbool.h>
#include <stdint.h>

uint32_t get_ping_interval_ms(void);
void connection_clocks_request_start(void);
void connection_clocks_request_start_delay_us(uint32_t delay_us);
void connection_clocks_request_stop(void);
void connection_clocks_request_stop_delay_us(uint32_t delay_us);

uint8_t connection_get_id(void);
void connection_set_id(uint8_t id);
uint8_t connection_get_packet_sequence(void);

void connection_update_sensor_ids(int imu_id, int mag_id);
void connection_update_sensor_data(float *q, float *a, int64_t data_time); // ticks
void connection_update_sensor_mag(float *m);
void connection_update_sensor_temp(float temp);
void connection_update_battery(bool battery_available, bool plugged, bool charged, uint32_t battery_pptt, int battery_mV);
void connection_update_status(int status);

void connection_write_packet_0();
void connection_write_packet_1();
void connection_write_packet_2();
void connection_write_packet_3();
void connection_write_packet_4();
void connection_write_packet_5();

// Raw sensor data collection (runtime controlled via PONG command)

struct raw_imu_sample {
	float gyr_quat[4];  // accumulated raw gyro quaternion (w,x,y,z)
	float accel[3];     // g from fifo_process
	float temp_c;       // T-Cal temperature in deg C for raw data collection
};

// Enable/disable data collection (called from PONG command handler)
void connection_set_data_collection(bool enable);
bool connection_get_data_collection(void);

// OTA suppression: reduce poll rate when another tracker is being updated
void connection_set_ota_suppressed(bool suppressed);
bool connection_get_ota_suppressed(void);

// Queue a raw IMU sample for transmission (called from sensor thread)
void connection_queue_raw_sample(const struct raw_imu_sample *sample);

// Queue uncalibrated magnetometer data for body-frame raw transport
void connection_queue_raw_mag(const float mag[3]);

// Send metadata packet with ODR/range info (called once when data collection starts)
void connection_send_raw_metadata(float gyro_range, float accel_range,
				  float gyro_odr, float accel_odr,
				  float mag_odr, uint8_t imu_id, uint8_t mag_id);

// Send calibration data packets (type 0x14, called after metadata)
void connection_send_raw_calibration(void);

// Check if metadata needs periodic re-send (returns true if due)
bool connection_raw_metadata_resend_due(void);

// Drain queued raw data and transmit (called from connection thread)
// Returns true if a packet was sent
bool connection_process_raw_data(void);

#endif
