#ifndef TINYTHREADS_SEMAPHORE_H
#define TINYTHREADS_SEMAPHORE_H

#include "tinyThreads_core.h"

bool tt_SemaphoreTake(tinyThread_binary_semaphore_t *sem, tinyThreadsTime_ms_t timeout_ms);
void tt_SemaphoreGive(tinyThread_binary_semaphore_t *sem);

#endif // TINYTHREADS_SEMAPHORE_H
