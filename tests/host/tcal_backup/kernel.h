#pragma once
#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#define K_NO_WAIT 0
#define K_MSEC(ms) (ms)
#define K_SECONDS(s) ((s)*1000)
static bool in_irq, allocation_fail;
static unsigned allocations, frees, live_allocations;
static size_t live_bytes;
static int64_t now_ms;
static int64_t k_uptime_get(void) { return now_ms; }
static void (*during_allocation)(void);
static void *k_malloc(size_t size) {
    assert(!in_irq);
    if (allocation_fail) return NULL;
    void *p = malloc(size);
    assert(p && !live_allocations);
    allocations++; live_allocations++; live_bytes = size;
    if (during_allocation) {
        void (*callback)(void) = during_allocation;
        during_allocation = NULL;
        callback();
    }
    return p;
}
static void k_free(void *p) {
    assert(!in_irq && p && live_allocations == 1);
    frees++; live_allocations--; live_bytes = 0; free(p);
}
struct k_spinlock { bool held; };
typedef int k_spinlock_key_t;
static k_spinlock_key_t k_spin_lock(struct k_spinlock *s) {
    assert(!s->held); s->held = true; return 0;
}
static void k_spin_unlock(struct k_spinlock *s, k_spinlock_key_t key) {
    (void)key; assert(s->held); s->held = false;
}
#define K_FOREVER (-1)
#define K_MUTEX_DEFINE(name) static pthread_mutex_t name = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP
static unsigned lock_depth;
static int k_mutex_lock(pthread_mutex_t *mutex, int timeout) {
    assert(!in_irq);
    (void)timeout;
    int result = pthread_mutex_lock(mutex);
    assert(result == 0);
    lock_depth++;
    return result;
}
static int k_mutex_unlock(pthread_mutex_t *mutex) {
    assert(lock_depth);
    lock_depth--;
    int result = pthread_mutex_unlock(mutex);
    assert(result == 0);
    return result;
}
static void (*during_sleep)(int);
static void k_msleep(int ms) {
    assert(lock_depth == 0 && !in_irq);
    if (during_sleep) during_sleep(ms);
}
