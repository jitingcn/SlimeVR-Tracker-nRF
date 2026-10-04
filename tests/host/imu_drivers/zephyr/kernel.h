#ifndef HOST_ZEPHYR_KERNEL_H
#define HOST_ZEPHYR_KERNEL_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../../harness/include/driver_util.h"

static inline int64_t k_uptime_get(void) { return 0; }
static inline void k_busy_wait(uint32_t usec) { (void)usec; }
static inline void k_msleep(int32_t msec) { (void)msec; }
static inline void k_usleep(int32_t usec) { (void)usec; }

#endif
