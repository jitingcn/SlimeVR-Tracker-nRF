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
 * Every slot is 22 network ticks. Normal NoACK traffic may use the owner's
 * slot once per frame. Approximately once per second, each tracker has a
 * deterministic guarded PING window spanning its slot and the following slot;
 * the following owner remains silent for that frame. Empty opportunities do
 * not cause transmissions.
 *
 * Parameters are dynamically assigned by the receiver via PONG bytes 8-11:
 *   byte 8:  assigned_slot_index (0-15, or 0xFF = unassigned)
 *   byte 9:  total_slots (1-16)
 *   byte 10: slot_ticks (22 for the measured-safe nRF52 guarded mode)
 *   byte 11: config_epoch (wrapping uint8_t)
 *
 * Normal NoACK traffic is limited to one packet per tracker frame. Before
 * queueing, payload-aware admission reserves measured READY->END airtime plus
 * queue/ramp margin; late packets wait for the next unguarded own slot.
 *
 * Architecture:
 *   - Connection thread prepares packets and calls esb_write()
 *   - Every tracker derives the same PING/guard schedule from server frame time
 *   - A synchronized PING consumes its own frame and the next physical slot
 *   - Unsynced, stale-sync, or unconfigured PINGs transmit immediately for recovery
 */
/* Compile-time defaults / fallbacks (used if no dynamic config received) */
#define TDMA_NUM_TRACKERS  10
#define TDMA_SLOT_TICKS    22  /* ~671μs at 32768Hz */
#define TDMA_FRAME_TICKS   (TDMA_SLOT_TICKS * TDMA_NUM_TRACKERS)

/** Number of receiver-supported tracker IDs used to partition PING phase. */
#define TDMA_MAX_TRACKERS 16

/**
 * Initialize the TDMA module with this tracker's ID.
 */
void tdma_init(uint8_t tracker_id);

/**
 * Block until this tracker can safely launch one normal NoACK packet.
 *
 * Admission allows at most one normal NoACK packet per TDMA frame and reserves
 * a payload-size-specific call-to-END budget. A late packet, guarded frame, or
 * second packet in one frame waits for the next usable own slot. Waiting occurs
 * before esb_write_payload(), so the ESB FIFO cannot auto-drain while sleeping.
 * Returns immediately when TDMA or synchronized server time is unavailable.
 */
bool tdma_wait_for_slot(uint8_t payload_length);

/** Result of attempting the current deterministic guarded PING window. */
enum tdma_ping_admission {
	TDMA_PING_UNAVAILABLE,
	TDMA_PING_DEFERRED,
	TDMA_PING_ADMITTED,
};

/** Coarse delay until the next deterministic PING window; wakes early. */
bool tdma_ping_wake_delay_ms(uint32_t *delay_ms);

/** Admit the current two-slot PING window without waiting a full period. */
enum tdma_ping_admission tdma_wait_for_ping_window(void);

/** Record that an admitted frame was skipped because another ESB TX was active. */
void tdma_note_radio_busy(void);

/** Print cumulative payload-admission counters for a field canary. */
void tdma_print_stats(void);

/**
 * Enable or disable TDMA at runtime.
 */
void tdma_set_enabled(bool enabled);

/**
 * Check if TDMA is currently active (compiled in AND runtime enabled).
 */
bool tdma_is_enabled(void);

/**
 * Current TDMA frame width in 32768 Hz server ticks (0 = no dynamic config yet).
 */
uint16_t tdma_frame_ticks_get(void);

/** Snapshot the active TDMA slot configuration for diagnostics. */
bool tdma_config_get(uint8_t *slot_index, uint8_t *slot_ticks, uint16_t *frame_ticks);

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
