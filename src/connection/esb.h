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
extern bool send_data;
// TODO: esb/sensor?
extern uint16_t led_clock;
extern uint32_t led_clock_offset;

void esb_write_ack(uint8_t type);
void event_handler(struct esb_evt const* event);
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

void esb_write(uint8_t* data, bool no_ack, size_t data_length);  // TODO: give packets some names

#define PING_INTERVAL_MS 1000
// Ping/Pong types for ACK payload validation
#define ESB_PING_TYPE 0xF0
#define ESB_PONG_TYPE 0xF1

// Ping/Pong packet sizes
#define ESB_PING_LEN 13  // with CRC-8
#define ESB_PONG_LEN 13  // with CRC-8

// Remote command flags for PONG data[7] (shared with receiver)
#define ESB_PONG_FLAG_NORMAL 0x00
#define ESB_PONG_FLAG_SHUTDOWN 0x01
#define ESB_PONG_FLAG_CALIBRATE 0x02        // Trigger gyro/accel ZRO calibration
#define ESB_PONG_FLAG_SIX_SIDE_CAL 0x03     // Trigger 6-point accelerometer calibration
#define ESB_PONG_FLAG_MEOW 0x04             // Trigger meow output
#define ESB_PONG_FLAG_SCAN 0x05             // Trigger sensor scan
#define ESB_PONG_FLAG_MAG_CLEAR 0x06        // Clear magnetometer calibration
#define ESB_PONG_FLAG_REBOOT 0x07           // Reboot tracker
#define ESB_PONG_FLAG_CLEAR 0x08            // Clear pairing data
#define ESB_PONG_FLAG_DFU 0x09              // Enter DFU bootloader
// Reserved for future use: 0x0A-0xFF

bool esb_ready(void);

// Get remote command flag to echo back in PING
uint8_t esb_get_ping_ack_flag(void);

// Helper: log esb_write call frequency
void esb_write_rate_tick(void);

#endif
