#ifndef TINYTHREADS_TASK_H
#define TINYTHREADS_TASK_H
// clang-format off
#include "tinyThreads_types.h"
#include "tinyThreads_error.h"

TinyThreadsStatus tt_ThreadAdd(void (*thread)(void), tinyThreadsTime_ms_t period, tinyThreadPriority_t priority);
TinyThreadsStatus tt_ThreadStackInit(uint32_t threadIDX);
TinyThreadsStatus tt_ThreadSleep(uint32_t time_ms);
tinyThreadsTime_ms_t tt_ThreadGetLastRunTime();
tinyThreadsTime_ms_t tt_ThreadGetSleepCount(tinyThread_tcb_idx id);
tinyThread_tcb *tt_ThreadGetCurrentTcb(void);
#endif // TINYTHREADS_TASK_H
