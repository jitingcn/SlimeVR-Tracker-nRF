#ifndef CONNECTION_NETWORK_TICKS_H
#define CONNECTION_NETWORK_TICKS_H

#include <stdint.h>
#include <zephyr/sys/time_units.h>

/* Convert the full kernel timeline before truncating to a wire timestamp.
 * nRF54L kernel ticks run at 31250 Hz; network ticks always run at 32768 Hz. */
static inline uint64_t net_ticks_from_kernel64(uint64_t kernel_ticks)
{
#if defined(CONFIG_SOC_SERIES_NRF54L) || defined(CONFIG_SOC_COMPATIBLE_NRF54L)
	return (k_ticks_to_us_near64(kernel_ticks) * 32768ULL) / 1000000ULL;
#else
	return kernel_ticks;
#endif
}

static inline uint32_t net_ticks_from_ms_near32(uint32_t ms)
{
	return (uint32_t)(((uint64_t)ms * 32768U + 500U) / 1000U);
}

static inline uint32_t net_ticks_to_ms_floor32(uint32_t ticks)
{
	return (uint32_t)(((uint64_t)ticks * 1000U) / 32768U);
}

static inline uint32_t net_ticks_to_ms_ceil32(uint32_t ticks)
{
	return (uint32_t)(((uint64_t)ticks * 1000U + 32767U) / 32768U);
}

static inline uint64_t net_ticks_to_us_ceil64(uint32_t ticks)
{
	return ((uint64_t)ticks * 1000000U + 32767U) / 32768U;
}

#endif
