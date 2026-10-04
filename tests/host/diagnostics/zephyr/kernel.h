#ifndef TEST_DIAGNOSTICS_KERNEL_H
#define TEST_DIAGNOSTICS_KERNEL_H

#include <stdint.h>

#define ARG_UNUSED(x) (void)(x)
#define IS_ENABLED(x) (x)
#include "../../harness/include/spinlock.h"
int64_t k_uptime_get(void);

#endif
