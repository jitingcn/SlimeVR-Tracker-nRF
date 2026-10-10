#ifndef HOST_ZEPHYR_KERNEL_H
#define HOST_ZEPHYR_KERNEL_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../../harness/include/driver_util.h"
#define BIT(n) (1UL << (n))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static inline int64_t k_uptime_get(void) { return 0; }
/* The host shim has a stationary clock; use microseconds as its tick unit. */
static inline int64_t k_uptime_ticks(void) { return 0; }
static inline uint64_t k_ticks_to_us_near64(uint64_t ticks) { return ticks; }
static inline void k_busy_wait(uint32_t usec) { (void)usec; }
static inline void k_msleep(int32_t msec) { (void)msec; }
static inline void k_usleep(int32_t usec) { (void)usec; }

#endif
