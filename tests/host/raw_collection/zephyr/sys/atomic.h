#ifndef TEST_RAW_COLLECTION_ZEPHYR_SYS_ATOMIC_H
#define TEST_RAW_COLLECTION_ZEPHYR_SYS_ATOMIC_H

#include "../../../harness/include/atomic.h"

static inline atomic_val_t atomic_inc(atomic_t *target)
{
	return __atomic_fetch_add(target, 1, __ATOMIC_SEQ_CST);
}

#endif
