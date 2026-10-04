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
#include "globals.h"

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <hal/nrf_clock.h>
#include <zephyr/logging/log.h>
#include <lib/nrfx_coredep.h>
#include <zephyr/sys/reboot.h>
#include <errno.h>

// clock_control already has a log module defined in nrf_clock_control, so we define our own for this file
LOG_MODULE_REGISTER(clock_switch, LOG_LEVEL_INF);

#define LFCLK_WAIT_STEP_US 300
#define LFCLK_STOP_TIMEOUT_US 10000
#define LFCLK_START_TIMEOUT_US 1000000

/* Helper to normalize XTAL variants for comparison.
 * XTAL_FULL_SWING and XTAL_LOW_SWING report as XTAL in the actual source. */
static inline nrf_clock_lfclk_t normalize_source(nrf_clock_lfclk_t source)
{
#if defined(NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES) || defined(__NRFX_DOXYGEN__)
	if (source == NRF_CLOCK_LFCLK_XTAL_FULL_SWING || source == NRF_CLOCK_LFCLK_XTAL_LOW_SWING) {
		return NRF_CLOCK_LFCLK_XTAL;
	}
#endif
	return source;
}

static bool lfclk_running_source_get(nrf_clock_lfclk_t *source)
{
	nrf_clock_lfclk_t active_source = NRF_CLOCK_LFCLK_RC;
	bool running = nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_LFCLK, &active_source);

	if (source != NULL) {
		*source = normalize_source(active_source);
	}

	return running;
}
/* CPU-loop delays remain usable while the LFCLK-backed system timer is stopped. */
static int lfclk_start_source(nrf_clock_lfclk_t source)
{
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTOP);
	for (uint32_t waited = 0; ; waited += LFCLK_WAIT_STEP_US) {
		nrf_clock_lfclk_t actual;
		bool running = lfclk_running_source_get(&actual);
		/* WDT can keep LFRC running after STOP clears the software START request.
		 * Wait for that request to clear and any old non-RC source to settle. */
		if (!nrf_clock_start_task_check(NRF_CLOCK, NRF_CLOCK_DOMAIN_LFCLK)
			&& (!running || actual == NRF_CLOCK_LFCLK_RC)) {
			break;
		}
		if (waited >= LFCLK_STOP_TIMEOUT_US) {
			return -ETIMEDOUT;
		}
		nrfx_coredep_delay_us(LFCLK_WAIT_STEP_US);
	}
	nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
	nrf_clock_lf_src_set(NRF_CLOCK, source);
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);
	for (uint32_t waited = 0; waited < LFCLK_START_TIMEOUT_US; waited += LFCLK_WAIT_STEP_US) {
		nrf_clock_lfclk_t actual;
		if (nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED)
			&& lfclk_running_source_get(&actual) && actual == normalize_source(source)) {
			return 0;
		}
		nrfx_coredep_delay_us(LFCLK_WAIT_STEP_US);
	}
	return -ETIMEDOUT;
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
	bool xtal_source = source == NRF_CLOCK_LFCLK_XTAL;
#ifdef NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES
	xtal_source = xtal_source || source == NRF_CLOCK_LFCLK_XTAL_FULL_SWING
		|| source == NRF_CLOCK_LFCLK_XTAL_LOW_SWING;
#endif
	if (!IS_ENABLED(CONFIG_CLOCK_USE_LFXO) && xtal_source) {
		LOG_INF("clock_switch: skipping XTAL, CONFIG_CLOCK_USE_LFXO disabled");
		return;
	}
#endif

	/* Check if already running with the requested source */
	nrf_clock_lfclk_t current_source;
	bool running = lfclk_running_source_get(&current_source);
	nrf_clock_lfclk_t normalized_requested = normalize_source(source);

	if (running && current_source == normalized_requested) {
		LOG_INF("clock_switch: already running with source=%d", current_source);
		return;
	}

	LOG_INF("clock_switch: %d -> %d", current_source, normalized_requested);

	/* Keep the stop-to-start transition atomic while the system timer is stopped. */
	unsigned int key = irq_lock();
	int err = lfclk_start_source(source);
	if (err && normalized_requested != NRF_CLOCK_LFCLK_RC) {
		err = lfclk_start_source(NRF_CLOCK_LFCLK_RC);
		if (!err) {
			irq_unlock(key);
			LOG_ERR("LFCLK source %d failed; using RC fallback", source);
			return;
		}
	}
	irq_unlock(key);
	if (err) {
		LOG_ERR("LFCLK unavailable: %d", err);
		sys_reboot(SYS_REBOOT_COLD);
		return;
	}
	LOG_INF("clock_switch: switched to source=%d successfully", normalized_requested);
}

// Switch to RC clock before shut down to avoid any problems with the bootloader
void clock_pre_shutdown(void)
{
	nrf_clock_lfclk_t current_source;
	bool running = lfclk_running_source_get(&current_source);

	if (running && current_source != NRF_CLOCK_LFCLK_RC) {
		clock_switch(NRF_CLOCK_LFCLK_RC);
	}
}

// Switch to external oscillator for LF clock for good TDMA precision
void clock_init_external(void)
{
	if (IS_ENABLED(CONFIG_CLOCK_USE_LFXO)) {
#if defined(NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES) || defined(__NRFX_DOXYGEN__)
		if (IS_ENABLED(CONFIG_CLOCK_USE_LFXO_MODE_FULL_SWING)) {
			clock_switch(NRF_CLOCK_LFCLK_XTAL_FULL_SWING);
		} else if (IS_ENABLED(CONFIG_CLOCK_USE_LFXO_MODE_LOW_SWING)) {
			clock_switch(NRF_CLOCK_LFCLK_XTAL_LOW_SWING);
		} else
#endif
		{
			clock_switch(NRF_CLOCK_LFCLK_XTAL);
		}
	} else if (IS_ENABLED(CONFIG_CLOCK_USE_LF_SYNTH)) {
		/* Use LF synthesizer (derived from HFXO) for TDMA timing precision
		 * when LFXO is not available on the board */
#if defined(CLOCK_LFCLKSRC_SRC_Synth) || defined(CLOCK_LFCLK_SRC_SRC_LFSYNT) || \
	defined(CLOCK_LFCLKSRC_SRC_LFSYNT)
		clock_switch(NRF_CLOCK_LFCLK_SYNTH);
#else
		LOG_WRN("clock_init_external: LF_SYNTH requested but not supported");
#endif
	}
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
		CLOCK_INIT_THREAD_PRIORITY,
		0,
		K_NO_WAIT
	);
}
