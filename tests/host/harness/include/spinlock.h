#ifndef HOST_HARNESS_SPINLOCK_H
#define HOST_HARNESS_SPINLOCK_H

/* Fixture-owned lock operations; no default locking or IRQ behavior. */
struct k_spinlock { unsigned int held; };
typedef unsigned int k_spinlock_key_t;
k_spinlock_key_t k_spin_lock(struct k_spinlock *lock);
void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key);

#endif
