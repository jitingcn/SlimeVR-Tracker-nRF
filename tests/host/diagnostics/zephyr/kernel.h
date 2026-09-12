#ifndef TEST_DIAGNOSTICS_KERNEL_H
#define TEST_DIAGNOSTICS_KERNEL_H

#include <stdint.h>

#define ARG_UNUSED(x) (void)(x)
#define IS_ENABLED(x) (x)
struct k_spinlock { unsigned int held; };
typedef unsigned int k_spinlock_key_t;
k_spinlock_key_t k_spin_lock(struct k_spinlock *lock);
void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key);
int64_t k_uptime_get(void);

#endif
