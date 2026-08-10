#ifndef TEST_ZEPHYR_SYS_ATOMIC_H
#define TEST_ZEPHYR_SYS_ATOMIC_H

typedef int atomic_val_t;
typedef volatile atomic_val_t atomic_t;

#define ATOMIC_INIT(value) (value)

static inline atomic_val_t atomic_get(const atomic_t *target)
{
	return __atomic_load_n(target, __ATOMIC_SEQ_CST);
}

static inline atomic_val_t atomic_set(atomic_t *target, atomic_val_t value)
{
	return __atomic_exchange_n(target, value, __ATOMIC_SEQ_CST);
}

#endif
