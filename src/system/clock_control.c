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

#define LFCLK_WAIT_STEP_US 100
#define LFCLK_WAIT_TIMEOUT_US 500000

/*
 * Track whether we have actually switched to an external LFCLK source.
 * This avoids doing redundant RC->RC switching in shutdown paths.
 */
static bool lfclk_using_xtal = false;

// Safely switch LF clock source
void clock_switch(nrf_clock_lfclk_t source)
{
#if defined(NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES) || defined(__NRFX_DOXYGEN__)
	/*
	 * Avoid switching to XTAL when the board does not have an external LFXO.
	 * Note: switching to RC is always safe.
	 */
	if (!IS_ENABLED(CONFIG_CLOCK_USE_LFXO) &&
	    (source == NRF_CLOCK_LFCLK_XTAL || source == NRF_CLOCK_LFCLK_XTAL_FULL_SWING)) {
		return;
	}
#endif

	/*
	 * Keep the register update sequence atomic with respect to interrupts,
	 * but do not keep interrupts disabled while waiting for the clock to stop/start.
	 */
	unsigned int key = irq_lock();
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTOP);
	irq_unlock(key);

	uint32_t waited_us = 0;
	while (nrf_clock_lf_is_running(NRF_CLOCK) && (waited_us < LFCLK_WAIT_TIMEOUT_US)) {
		k_busy_wait(LFCLK_WAIT_STEP_US);
		waited_us += LFCLK_WAIT_STEP_US;
	}

	/*
	 * Start and wait for LFCLKSTARTED event, as used in sdk-nrf board init hooks.
	 * This avoids returning early before the LF clock has actually started.
	 */
	key = irq_lock();
	nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
	nrf_clock_lf_src_set(NRF_CLOCK, source);
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);
	irq_unlock(key);

	waited_us = 0;
	while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED) &&
	       (waited_us < LFCLK_WAIT_TIMEOUT_US)) {
		k_busy_wait(LFCLK_WAIT_STEP_US);
		waited_us += LFCLK_WAIT_STEP_US;
	}

	const bool started = nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
	if (started) {
		lfclk_using_xtal = (source == NRF_CLOCK_LFCLK_XTAL) ||
				  (source == NRF_CLOCK_LFCLK_XTAL_FULL_SWING);
	}

	key = irq_lock();
	nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
	irq_unlock(key);
}

// Switch to RC clock before shut down to avoid any problems with the bootloader
void clock_pre_shutdown()
{
	if (!lfclk_using_xtal) {
		return;
	}

	clock_switch(NRF_CLOCK_LFCLK_RC);
}

// Switch to external oscillator for LF clock for good TDMA precision
void clock_init_external()
{
#if defined(NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES) || defined(__NRFX_DOXYGEN__)
	if (IS_ENABLED(CONFIG_CLOCK_USE_LFXO)) {
		clock_switch(NRF_CLOCK_LFCLK_XTAL_FULL_SWING);
	}
#endif
}
