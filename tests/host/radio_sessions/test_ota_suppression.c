#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>

#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)

static int64_t clock_ms;
static int64_t k_uptime_get(void) { return __atomic_load_n(&clock_ms, __ATOMIC_SEQ_CST); }
static void set_clock(int64_t value) { __atomic_store_n(&clock_ms, value, __ATOMIC_SEQ_CST); }

#include "suppression.inc"

static pthread_barrier_t start_barrier, finish_barrier;
#define RACE_ITERATIONS 2000

static void wait_barrier(pthread_barrier_t *barrier)
{
    int result = pthread_barrier_wait(barrier);
    assert(result == 0 || result == PTHREAD_BARRIER_SERIAL_THREAD);
}

static void *refresh_worker(void *unused)
{
    (void)unused;
    for (int i = 0; i < RACE_ITERATIONS; ++i) {
        wait_barrier(&start_barrier);
        set_clock(600002);
        connection_set_ota_suppressed(true);
        wait_barrier(&finish_barrier);
    }
    return NULL;
}

static void *timeout_worker(void *unused)
{
    (void)unused;
    for (int i = 0; i < RACE_ITERATIONS; ++i) {
        wait_barrier(&start_barrier);
        (void)connection_ota_suppression_active(600002);
        wait_barrier(&finish_barrier);
    }
    return NULL;
}

int main(void)
{
    set_clock(100);
    connection_set_ota_suppressed(true);
    assert(connection_ota_suppression_active(600100));
    assert(!connection_ota_suppression_active(600101));
    assert(!connection_get_ota_suppressed());

    set_clock(100);
    connection_set_ota_suppressed(true);
    assert(connection_get_ota_suppressed());
    set_clock(600101);
    connection_set_ota_suppressed(true);
    assert(connection_ota_suppression_active(600101));
    assert(connection_get_ota_suppressed());
    assert(!connection_ota_suppression_active(1200102));

    set_clock(0);
    connection_set_ota_suppressed(true);
    assert(connection_ota_suppression_active(700000));
    connection_set_ota_suppressed(false);
    assert(!connection_get_ota_suppressed());

    assert(pthread_barrier_init(&start_barrier, NULL, 3) == 0);
    assert(pthread_barrier_init(&finish_barrier, NULL, 3) == 0);
    pthread_t writer, timeout;
    assert(pthread_create(&writer, NULL, refresh_worker, NULL) == 0);
    assert(pthread_create(&timeout, NULL, timeout_worker, NULL) == 0);
    for (int i = 0; i < RACE_ITERATIONS; ++i) {
        set_clock(1);
        connection_set_ota_suppressed(true);
        wait_barrier(&start_barrier);
        wait_barrier(&finish_barrier);
        assert(connection_get_ota_suppressed());
    }
    assert(pthread_join(writer, NULL) == 0);
    assert(pthread_join(timeout, NULL) == 0);
    assert(pthread_barrier_destroy(&start_barrier) == 0);
    assert(pthread_barrier_destroy(&finish_barrier) == 0);
    puts("OTA suppression timeout and concurrent refresh: PASS");
    return 0;
}
