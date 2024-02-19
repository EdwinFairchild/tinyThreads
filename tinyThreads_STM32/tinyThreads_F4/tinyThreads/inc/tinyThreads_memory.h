#ifndef TINYTHREADS_MEMORY_H
#define TINYTHREADS_MEMORY_H

#include "tinyThreads_core.h"
#include "tinyThreads_error.h"

void *tt_MemoryAllocBuf(size_t size);
void  tt_MemoryFreeBuf(void *ptr);
#endif // TINYTHREADS_MEMORY_H
