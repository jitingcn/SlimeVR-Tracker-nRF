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
#ifndef SLIMENRF_TIMER
#define SLIMENRF_TIMER

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize TDMA hardware timer
 *
 * Sets up TIMER1 @ 1MHz for TDMA slot scheduling.
 * Must be called after tracker ID is known (after pairing).
 */
void timer_init(void);

/**
 * @brief Set tracker ID for TDMA slot calculation
 *
 * @param id Tracker ID (0-9 for 10 tracker TDMA)
 */
void timer_set_tracker_id(uint8_t id);

/**
 * @brief Notify timer of skew convergence status
 *
 * Called from esb.c when clock skew is updated.
 * TDMA is enabled when abs_error stays below threshold for consecutive updates.
 *
 * @param abs_error Absolute value of skew error (fixed point)
 */
void timer_on_skew_update(int32_t abs_error);

/**
 * @brief Check if TDMA mode is currently active
 *
 * @return true if TDMA timing control is active
 * @return false if using immediate TX (fallback mode)
 */
bool timer_is_tdma_active(void);

/**
 * @brief Signal that a packet is pending transmission
 *
 * In TDMA mode, the timer will call esb_start_tx() at the correct slot.
 * In non-TDMA mode, this should trigger immediate transmission.
 */
void timer_signal_tx_pending(void);

#endif // SLIMENRF_TIMER
