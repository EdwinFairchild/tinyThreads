#ifndef TINYTHREADS_MEMORY_H
#define TINYTHREADS_MEMORY_H

#include "tinyThreads_core.h"
#include "tinyThreads_error.h"

void *allocate_memory(size_t size);
void  free_memory(void *ptr);
#endif // TINYTHREADS_MEMORY_H
