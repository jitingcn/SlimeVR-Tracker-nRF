/* CPU-cycle diagnostics. DWT is independent of the 32768 Hz kernel clock. */
#ifndef SLIMENRF_RELAY_TIMING_H
#define SLIMENRF_RELAY_TIMING_H
#include <stdint.h>
#if defined(CONFIG_RELAY_CYCLE_DIAGNOSTICS) || defined(CONFIG_RELAY_LEDGER_TIMING) || defined(CONFIG_RELAY_IRQ_DIAGNOSTICS)
#include <cmsis_core.h>
static inline void relay_cycle_init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t relay_cycle_now(void) { return DWT->CYCCNT; }
static inline void relay_cycle_max(uint32_t *maximum, uint32_t start)
{
	uint32_t elapsed = relay_cycle_now() - start;
	if (elapsed > *maximum) {
		*maximum = elapsed;
	}
}
#endif
#endif
