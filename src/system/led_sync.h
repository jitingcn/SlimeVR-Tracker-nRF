#ifndef SLIMENRF_LED_SYNC_H
#define SLIMENRF_LED_SYNC_H
#include <stdint.h>
#include <limits.h>
#define LED_SYNC_HZ 32768U

/* Readonly conversion; kernel tick frequency is not the radio clock frequency.
 * Split before multiplication to preserve precision and avoid large products. */
static inline uint32_t led_sync_kernel_ticks(uint64_t ticks, uint32_t kernel_hz)
{
	return (uint32_t)((ticks / kernel_hz) * LED_SYNC_HZ + (ticks % kernel_hz) * LED_SYNC_HZ / kernel_hz);
}
/* Raw receiver low32 intentionally has a single phase discontinuity at wrap.
 * No private epoch or changes to the network clock protocol are introduced. */
static inline uint32_t led_sync_phase_ms(uint32_t ticks, uint32_t period_ms)
{
	uint32_t period_ticks = (uint64_t)period_ms * LED_SYNC_HZ / 1000;
	return (uint64_t)(ticks % period_ticks) * 1000 / LED_SYNC_HZ;
}
static inline uint32_t led_sync_wrap_ms(uint32_t ticks)
{
	return ((uint64_t)UINT32_MAX - ticks + 1) * 1000 / LED_SYNC_HZ + 1;
}
#endif
