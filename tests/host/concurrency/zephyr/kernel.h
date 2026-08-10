#ifndef TEST_ZEPHYR_KERNEL_H
#define TEST_ZEPHYR_KERNEL_H

#include <stdbool.h>

typedef unsigned int k_spinlock_key_t;

struct k_spinlock {
	bool locked;
};

static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *lock)
{
	while (__atomic_test_and_set(&lock->locked, __ATOMIC_ACQUIRE)) {
	}
	return 0;
}

static inline void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
	(void)key;
	__atomic_clear(&lock->locked, __ATOMIC_RELEASE);
}

#endif
