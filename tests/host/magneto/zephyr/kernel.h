#ifndef TEST_MAGNETO_KERNEL_H
#define TEST_MAGNETO_KERNEL_H

#include <stddef.h>

void *k_malloc(size_t size);
void k_free(void *ptr);

#endif
