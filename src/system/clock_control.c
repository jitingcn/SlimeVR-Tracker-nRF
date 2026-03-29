/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 SlimeVR Contributors
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
#include "clock_control.h"

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <hal/nrf_clock.h>
#include <zephyr/logging/log.h>

// clock_control already has a log module defined in nrf_clock_control, so we define our own for this file
LOG_MODULE_REGISTER(clock_switch, LOG_LEVEL_INF);

#define LFCLK_WAIT_STEP_US 300

/* Helper to normalize XTAL variants for comparison */
static inline nrf_clock_lfclk_t normalize_source(nrf_clock_lfclk_t source)
{
	/* XTAL_FULL_SWING and XTAL_LOW_SWING report as XTAL in actual source */
	if (
		source == NRF_CLOCK_LFCLK_XTAL_FULL_SWING
#ifdef NRF_CLOCK_LFCLK_XTAL_LOW_SWING
		|| source == NRF_CLOCK_LFCLK_XTAL_LOW_SWING
#endif
	) {
		return NRF_CLOCK_LFCLK_XTAL;
	}
	return source;
}

// Safely switch LF clock source
void clock_switch(nrf_clock_lfclk_t source)
{
	LOG_INF("clock_switch: requesting source=%d", source);

#if defined(NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES) || defined(__NRFX_DOXYGEN__)
	/*
	 * Avoid switching to XTAL when the board does not have an external LFXO.
	 * Note: switching to RC is always safe.
	 */
	if (!IS_ENABLED(CONFIG_CLOCK_USE_LFXO)
		&& (source == NRF_CLOCK_LFCLK_XTAL || source == NRF_CLOCK_LFCLK_XTAL_FULL_SWING)) {
		LOG_INF("clock_switch: skipping XTAL, CONFIG_CLOCK_USE_LFXO disabled");
		return;
	}
#endif

	/* Check if already running with the requested source */
	nrf_clock_lfclk_t current_source = nrf_clock_lf_actv_src_get(NRF_CLOCK);
	nrf_clock_lfclk_t normalized_requested = normalize_source(source);

	if (current_source == normalized_requested) {
		LOG_INF("clock_switch: already running with source=%d", current_source);
		return;
	}

	LOG_INF("clock_switch: %d -> %d", current_source, normalized_requested);

	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTOP);

	while (nrf_clock_lf_is_running(NRF_CLOCK) && nrf_clock_lf_actv_src_get(NRF_CLOCK) != NRF_CLOCK_LFCLK_RC) {}

	/*
	 * Start and wait for LFCLKSTARTED event, as used in sdk-nrf board init hooks.
	 * This avoids returning early before the LF clock has actually started.
	 */
	nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
	nrf_clock_lf_src_set(NRF_CLOCK, source);
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);

	if (source == NRF_CLOCK_LFCLK_RC) {
		// RC starts very quickly, just wait for the event without sleeping
		while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED)) {}
	} else {
	uint32_t waited_us = 0;
		while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED)) {
			k_usleep(LFCLK_WAIT_STEP_US);
			waited_us += LFCLK_WAIT_STEP_US;
		}
		if (waited_us > 1000) {
			LOG_INF("clock_switch: LFCLK start waited %u us", waited_us);
		}
	}

	/* Verify the actual clock source matches what we requested */
	nrf_clock_lfclk_t actual_source = nrf_clock_lf_actv_src_get(NRF_CLOCK);
	if (actual_source != normalized_requested) {
		LOG_ERR("clock_switch: source mismatch! requested=%d, actual=%d", normalized_requested, actual_source);
	} else {
		LOG_INF("clock_switch: switched to source=%d successfully", actual_source);
	}
}

// Switch to RC clock before shut down to avoid any problems with the bootloader
void clock_pre_shutdown(void)
{
	nrf_clock_lfclk_t current_source = nrf_clock_lf_actv_src_get(NRF_CLOCK);

	if (current_source != NRF_CLOCK_LFCLK_RC) {
		clock_switch(NRF_CLOCK_LFCLK_RC);
	}
}

// Switch to external oscillator for LF clock for good TDMA precision
void clock_init_external(void)
{
#if defined(NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES) || defined(__NRFX_DOXYGEN__)
	if (IS_ENABLED(CONFIG_CLOCK_USE_LFXO)) {
		clock_switch(NRF_CLOCK_LFCLK_XTAL);
	} else if (IS_ENABLED(CONFIG_CLOCK_USE_LF_SYNTH)) {
		/* Use LF synthesizer (derived from HFXO) for TDMA timing precision
		 * when LFXO is not available on the board */
#ifdef NRF_CLOCK_LFCLK_SYNTH
		clock_switch(NRF_CLOCK_LFCLK_SYNTH);
#else
		LOG_WRN("clock_init_external: LF_SYNTH requested but not supported");
#endif
	}
#endif
}

// Async version of clock_init_external
static struct k_thread clock_init_thread_id;
static K_THREAD_STACK_DEFINE(clock_init_thread_stack, 512);

static void clock_init_external_async_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	clock_init_external();
}

void clock_init_external_async(void)
{
	k_thread_create(
		&clock_init_thread_id,
		clock_init_thread_stack,
		K_THREAD_STACK_SIZEOF(clock_init_thread_stack),
		clock_init_external_async_thread,
		NULL,
		NULL,
		NULL,
		8,
		0,
		K_NO_WAIT
	);
}
