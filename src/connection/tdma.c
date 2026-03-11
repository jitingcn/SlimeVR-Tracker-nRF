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
#include "tdma.h"
#include "esb.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tdma, LOG_LEVEL_INF);

static uint8_t tdma_slot_index = 0;
static bool tdma_runtime_enabled = true;

void tdma_init(uint8_t tracker_id)
{
	tdma_slot_index = tracker_id % TDMA_NUM_TRACKERS;
#if CONFIG_CONNECTION_TDMA
	LOG_INF("TDMA init: slot=%u/%u, frame=%u ticks (~%u.%01u ms), ~%u TPS/tracker",
		tdma_slot_index, TDMA_NUM_TRACKERS,
		TDMA_FRAME_TICKS,
		(TDMA_FRAME_TICKS * 1000) / 32768,
		((TDMA_FRAME_TICKS * 10000) / 32768) % 10,
		32768 / TDMA_FRAME_TICKS);
#else
	LOG_INF("TDMA compiled out, all transmissions immediate");
#endif
}

void tdma_wait_for_slot(void)
{
#if !CONFIG_CONNECTION_TDMA
	return;
#else
	if (!tdma_runtime_enabled) {
		return;
	}

	uint64_t server_ticks = esb_get_server_time_ticks_64();
	if (server_ticks == 0) {
		return; /* not synced — fallback to immediate TX */
	}

	uint32_t frame_phase = (uint32_t)(server_ticks % TDMA_FRAME_TICKS);
	uint32_t current_slot = frame_phase / TDMA_SLOT_TICKS;
	uint32_t ticks_into_slot = frame_phase % TDMA_SLOT_TICKS;

	if (current_slot == tdma_slot_index) {
		return; /* already inside our slot, start TX now */
	}

	uint32_t slots_ahead;
	if (tdma_slot_index > current_slot) {
		slots_ahead = tdma_slot_index - current_slot;
	} else {
		slots_ahead = TDMA_NUM_TRACKERS - current_slot + tdma_slot_index;
	}

	/*
	 * Blocking wait until our slot boundary.
	 *
	 * During this sleep the connection thread is suspended, which prevents
	 * a second esb_write() call from racing in and flushing the FIFO before
	 * esb_start_tx() fires.
	 */
	int32_t ticks = (int32_t)(slots_ahead * TDMA_SLOT_TICKS - ticks_into_slot);
	if (ticks > 0) {
		k_sleep(K_TICKS(ticks));
	}
#endif
}

void tdma_set_enabled(bool enabled)
{
#if CONFIG_CONNECTION_TDMA
	if (tdma_runtime_enabled != enabled) {
		tdma_runtime_enabled = enabled;
		LOG_INF("TDMA %s at runtime", enabled ? "enabled" : "disabled");
	}
#else
	ARG_UNUSED(enabled);
	LOG_WRN("TDMA not compiled in, cannot change at runtime");
#endif
}

bool tdma_is_enabled(void)
{
#if CONFIG_CONNECTION_TDMA
	return tdma_runtime_enabled;
#else
	return false;
#endif
}
