#ifndef HOST_ZEPHYR_KERNEL_H
#define HOST_ZEPHYR_KERNEL_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#define MHZ(value) ((value) * 1000000U)
#define ARG_UNUSED(value) ((void)(value))

static inline int64_t k_uptime_get(void) { return 0; }
static inline void k_busy_wait(uint32_t usec) { (void)usec; }
static inline void k_msleep(int32_t msec) { (void)msec; }
static inline void k_usleep(int32_t usec) { (void)usec; }

#endif
