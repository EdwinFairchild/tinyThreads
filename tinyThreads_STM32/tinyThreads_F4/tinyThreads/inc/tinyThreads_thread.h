#ifndef TINYTHREADS_TASK_H
#define TINYTHREADS_TASK_H
// clang-format off
#include "tinyThreads_types.h"
#include "tinyThreads_error.h"

tinyThreadsTime_ms_t tinyThreads_getThreadLastRunTime(void);
TinyThreadsStatus tt_ThreadAdd(void (*thread)(void), tinyThreadsTime_ms_t period, tinyThreadPriority_t priority);
tinyThreadsTime_ms_t tinyKernel_getThreadLastRunTime();
TinyThreadsStatus tt_ThreadSleep(uint32_t time_ms);
tinyThreadsTime_ms_t getSleepCount(tinyThread_tcb_idx id);
#endif // TINYTHREADS_TASK_H
