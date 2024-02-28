#ifndef TINYTHREADS_MEMORY_H
#define TINYTHREADS_MEMORY_H

#include "tinyThreads_core.h"
#include "tinyThreads_error.h"

/**************************************************************************
 * TODO :
 *
 *
 **************************************************************************/
void *tt_MemoryAllocBuf(size_t size);

/**************************************************************************
 * TODO :
 *
 *
 **************************************************************************/
void tt_MemoryFreeBuf(void *ptr);

/**************************************************************************
 * TODO :
 *
 *
 **************************************************************************/
TinyThreadsStatus tt_MemoryInit(void);
#endif // TINYTHREADS_MEMORY_H
