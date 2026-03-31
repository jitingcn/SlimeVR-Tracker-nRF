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
#ifndef SLIMENRF_TDMA
#define SLIMENRF_TDMA

#include <stdbool.h>
#include <stdint.h>

/*
 * TDMA (Time Division Multiple Access) scheduling for ESB radio.
 *
 * Uses a short repeating frame to give each tracker a micro-slot for
 * high-frequency continuous packet transmission.
 *
 * Time base: server time ticks (32768 Hz) from PING/PONG synchronization.
 *
 * Frame structure (repeats every frame_ticks):
 *   [Slot 0][Slot 1][Slot 2]...[Slot N-1]   (N = total_slots, dynamic)
 *
 * Parameters are dynamically assigned by the receiver via PONG bytes 8-11:
 *   byte 8:  assigned_slot_index (0-15, or 0xFF = unassigned)
 *   byte 9:  total_slots (1-16)
 *   byte 10: slot_ticks (14-163)
 *   byte 11: config_epoch (wrapping uint8_t)
 *
 * NoACK sensor data TX at 2Mbps ≈ 200-250μs air time, fits in minimum 427μs slot.
 *
 * Architecture:
 *   - Connection thread prepares packets and calls esb_write()
 *   - esb_write() queues payload (ESB MANUAL TX mode)
 *   - For noack data: tdma_wait_for_slot() blocks until assigned slot
 *   - For PING/ACK packets: esb_start_tx() is called immediately (bypasses TDMA)
 */

/* Compile-time defaults / fallbacks (used if no dynamic config received) */
#define TDMA_NUM_TRACKERS  10
#define TDMA_SLOT_TICKS    18  /* ~550μs at 32768Hz */
#define TDMA_FRAME_TICKS   (TDMA_SLOT_TICKS * TDMA_NUM_TRACKERS)  /* 180 ticks ≈ 5.5ms */

/*
 * Grace window (ticks) past the nominal slot end.
 * If the connection thread overshoots the slot boundary (e.g. preempted
 * by the higher-priority sensor thread), allow TX anyway rather than
 * waiting a full frame.  Half a slot covers minor scheduler jitter
 * while leaving room before the neighbour's
 * target point (neighbour aims for their slot_start + 4 ticks).
 */
#define TDMA_OVERSHOOT_GRACE 3

/**
 * Initialize the TDMA module with this tracker's ID.
 */
void tdma_init(uint8_t tracker_id);

/**
 * Block the calling thread until this tracker's TDMA slot begins.
 *
 * Must be called from esb_write() AFTER esb_write_payload() but BEFORE
 * esb_start_tx().  This is the ONLY safe placement: the packet is already
 * in the FIFO, the caller is suspended (no second esb_write() call can race
 * in and flush the FIFO), and esb_start_tx() is called immediately on wakeup.
 *
 * Returns immediately if:
 *   - CONFIG_CONNECTION_TDMA is disabled
 *   - runtime TDMA is disabled
 *   - server time is not yet synced (fallback: transmit now)
 *   - already inside our slot window
 *
 * Uses k_sleep(K_TICKS(n)) which on nRF52 is backed by the 32768 Hz RTC,
 * giving ±1 tick (~30 µs) accuracy — sufficient for 610 µs slots.
 */
void tdma_wait_for_slot(void);

/**
 * Enable or disable TDMA at runtime.
 */
void tdma_set_enabled(bool enabled);

/**
 * Check if TDMA is currently active (compiled in AND runtime enabled).
 */
bool tdma_is_enabled(void);

/**
 * Update TDMA parameters from receiver's dynamic config (PONG bytes 8-11).
 * Automatically enables TDMA when valid config is received.
 *
 * @param slot_index   Assigned slot index for this tracker (0-15)
 * @param total_slots  Total number of active slots in the frame (1-16)
 * @param slot_ticks   Width of each slot in ticks (14-163)
 * @param epoch        Config epoch counter (for change detection)
 */
void tdma_update_config(uint8_t slot_index, uint8_t total_slots, uint8_t slot_ticks, uint8_t epoch);

/**
 * Get the current config epoch (for change detection in PONG processing).
 */
uint8_t tdma_get_config_epoch(void);

#endif
