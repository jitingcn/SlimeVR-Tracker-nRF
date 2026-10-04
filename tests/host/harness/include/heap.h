#ifndef HOST_HARNESS_HEAP_H
#define HOST_HARNESS_HEAP_H

#include <stddef.h>

void *k_malloc(size_t size);
void k_free(void *ptr);

#endif
