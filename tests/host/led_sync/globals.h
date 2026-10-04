#ifndef LED_HOST_GLOBALS_H
#define LED_HOST_GLOBALS_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <limits.h>
#define CONFIG_LED_THREAD_STACK_SIZE 1024
#ifndef CONFIG_SYS_CLOCK_TICKS_PER_SEC
#define CONFIG_SYS_CLOCK_TICKS_PER_SEC 32768
#endif
#ifndef CONFIG_LED_NETWORK_SYNC
#define CONFIG_LED_NETWORK_SYNC 0
#endif
#define LED_THREAD_PRIORITY 0
#define LOG_MODULE_REGISTER(...)
#define LOG_LEVEL_INF 0
#define LOG_ERR(...) ((void)0)
struct k_spinlock { int unused; };
typedef int k_spinlock_key_t;
typedef int64_t k_timeout_t;
struct k_sem { unsigned count; };
#define K_SEM_DEFINE(n,initial,max) struct k_sem n={initial}
#define K_THREAD_DEFINE(n,stack,fn,...) static void (*const n)(void) __attribute__((unused))=fn
#define K_FOREVER INT64_MAX
#define K_MSEC(n) ((k_timeout_t)(n))
static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *s) { (void)s; return 0; }
static inline void k_spin_unlock(struct k_spinlock *s,k_spinlock_key_t k) { (void)s; (void)k; }
static inline void k_sem_give(struct k_sem *s) { s->count=1; }
int64_t k_uptime_ticks(void);
int64_t k_uptime_get(void);
int k_sem_take(struct k_sem *,k_timeout_t);
int printk(const char *format,...);
#endif
