#ifndef CONTROL_TEST_KERNEL_H
#define CONTROL_TEST_KERNEL_H
#include <pthread.h>
#define K_FOREVER (-1)
#define K_MUTEX_DEFINE(name) pthread_mutex_t name = PTHREAD_MUTEX_INITIALIZER
static inline int k_mutex_lock(pthread_mutex_t *mutex, int timeout)
{
	(void)timeout;
	return pthread_mutex_lock(mutex);
}
static inline int k_mutex_unlock(pthread_mutex_t *mutex)
{
	return pthread_mutex_unlock(mutex);
}
#endif
