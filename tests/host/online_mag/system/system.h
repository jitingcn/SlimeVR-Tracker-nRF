#ifndef ONLINE_TEST_SYSTEM_H
#define ONLINE_TEST_SYSTEM_H
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
void sys_warm_transaction_begin(void);
void sys_warm_transaction_end(bool changed);
void sys_warm_transaction_mark(uint16_t id, const void *data, size_t size);
int sys_write(uint16_t id, void *dst, const void *src, size_t size);
#endif
