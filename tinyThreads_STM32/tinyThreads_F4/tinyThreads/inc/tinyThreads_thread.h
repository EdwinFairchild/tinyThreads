#ifndef TINYTHREADS_TASK_H
#define TINYTHREADS_TASK_H
// clang-format off
#include "tinyThreads_types.h"
#include "tinyThreads_error.h"


/**************************************************************************
 * TODO :
 * 
 * 
 **************************************************************************/
tinyThread_tcb_idx tt_ThreadAdd(void (*thread)(void), tinyThreadsTime_ms_t period, tinyThreadPriority_t priority);

/**************************************************************************
 * TODO :
 * 
 * 
 **************************************************************************/
TinyThreadsStatus tt_ThreadStackInit(uint32_t threadIDX);

/**************************************************************************
 * TODO :
 * 
 * 
 **************************************************************************/
TinyThreadsStatus tt_ThreadSleep(uint32_t time_ms);

/**************************************************************************
 * tt_ThreadWake
 * Use to wake a thread that is in sleeping via tt_ThreadSleep
 *  returns TINYTHREADS_OK if successful
 **************************************************************************/
TinyThreadsStatus tt_ThreadWake(tinyThread_tcb_idx id);

/**************************************************************************
 * TODO :
 * 
 * 
 **************************************************************************/
tinyThreadsTime_ms_t tt_ThreadGetLastRunTime();

/**************************************************************************
 * TODO :
 * 
 * 
 **************************************************************************/
tinyThreadsTime_ms_t tt_ThreadGetSleepCount(tinyThread_tcb_idx id);

/**************************************************************************
 * TODO :
 * 
 * 
 **************************************************************************/
tinyThread_tcb *tt_ThreadGetCurrentTcb(void);
void tt_ThreadYield(void);
#endif // TINYTHREADS_TASK_H
