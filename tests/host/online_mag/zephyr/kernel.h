#ifndef ONLINE_TEST_KERNEL_H
#define ONLINE_TEST_KERNEL_H
#include <stdint.h>
#include <stddef.h>
struct k_spinlock { unsigned held; };
typedef unsigned k_spinlock_key_t;
k_spinlock_key_t k_spin_lock(struct k_spinlock *lock);
void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key);
uint32_t k_uptime_get_32(void);
void k_msleep(unsigned ms);
void *k_malloc(size_t size);
void k_free(void *ptr);
#endif
