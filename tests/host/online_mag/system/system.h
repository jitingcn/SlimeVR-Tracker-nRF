#ifndef ONLINE_TEST_SYSTEM_H
#define ONLINE_TEST_SYSTEM_H
#include <stddef.h>
#include <stdbool.h>
void sys_warm_transaction_begin(void);
void sys_warm_transaction_end(bool changed);
void sys_warm_transaction_mark(int id, void *data, size_t size);
void sys_write(int id, void *dst, const void *src, size_t size);
#endif
