#ifndef HOST_ZEPHYR_KERNEL_H
#define HOST_ZEPHYR_KERNEL_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#define MHZ(value) ((value) * 1000000U)
#define ARG_UNUSED(value) ((void)(value))

/* Host SPI model has no devicetree-selected bitbang controller. */
#define DT_NODE_HAS_COMPAT(node_id, compat) 0

int64_t host_k_uptime_get(void);
void host_k_busy_wait(uint32_t usec);
void host_k_msleep(int32_t msec);
void host_k_usleep(int32_t usec);

static inline int64_t k_uptime_get(void)
{
	return host_k_uptime_get();
}

static inline void k_busy_wait(uint32_t usec)
{
	host_k_busy_wait(usec);
}

static inline void k_msleep(int32_t msec)
{
	host_k_msleep(msec);
}

static inline void k_usleep(int32_t usec)
{
	host_k_usleep(usec);
}

#endif
