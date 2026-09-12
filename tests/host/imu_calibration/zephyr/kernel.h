#ifndef TEST_IMU_CALIBRATION_KERNEL_H
#define TEST_IMU_CALIBRATION_KERNEL_H

#include <assert.h>
#include <errno.h>
#include <pthread.h>

struct k_spinlock {
	pthread_mutex_t mutex;
};
typedef int k_spinlock_key_t;

static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *lock)
{
	assert(pthread_mutex_lock(&lock->mutex) == 0);
	return 0;
}

static inline void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
	(void)key;
	assert(pthread_mutex_unlock(&lock->mutex) == 0);
}

struct k_mutex {
	pthread_mutex_t mutex;
};
#define K_MUTEX_DEFINE(name) struct k_mutex name = {PTHREAD_MUTEX_INITIALIZER}
#define K_FOREVER (-1)
#ifdef TEST_LIFECYCLE
void host_mutex_waiting(struct k_mutex *mutex);
#endif

static inline int k_mutex_lock(struct k_mutex *mutex, int timeout)
{
	(void)timeout;
	int err = pthread_mutex_trylock(&mutex->mutex);
	if (err == EBUSY) {
#ifdef TEST_LIFECYCLE
		host_mutex_waiting(mutex);
#endif
		err = pthread_mutex_lock(&mutex->mutex);
	}
	assert(err == 0);
	return 0;
}

static inline int k_mutex_unlock(struct k_mutex *mutex)
{
	assert(pthread_mutex_unlock(&mutex->mutex) == 0);
	return 0;
}

#endif
