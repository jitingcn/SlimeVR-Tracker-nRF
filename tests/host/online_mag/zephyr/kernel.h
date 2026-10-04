#ifndef ONLINE_TEST_KERNEL_H
#define ONLINE_TEST_KERNEL_H
#include <stdint.h>
#include "../../harness/include/heap.h"
#include "../../harness/include/spinlock.h"
uint32_t k_uptime_get_32(void);
void k_msleep(unsigned ms);
#endif
