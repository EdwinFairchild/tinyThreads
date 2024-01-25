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
TinyThreadsStatus tt_ThreadSleep(uint32_t time_ms);

/**************************************************************************
 * tt_ThreadWake
 * Used to wake a thread that is in sleeping via tt_ThreadSleep
 * returns TINYTHREADS_OK if successful
 **************************************************************************/
TinyThreadsStatus tt_ThreadWake(tinyThread_tcb_idx id);

/**************************************************************************
 * tt_ThreadPause
 * Used to pause a thread that is running
 * returns TINYTHREADS_OK if successful
 **************************************************************************/
TinyThreadsStatus tt_ThreadPause(tinyThread_tcb_idx id);

/**************************************************************************
 * tt_ThreadUnPause
 * Used to resume a thread thatthat is paused via tt_ThreadPause
 * returns TINYTHREADS_OK if successful
 **************************************************************************/
TinyThreadsStatus tt_ThreadUnPause(tinyThread_tcb_idx id);
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

void tt_ThreadUpdateNextThreadPtr(void);
TinyThreadsStatus tt_ThreadUpdateInactive(void);
void tt_ThreadYield(void);
#endif // TINYTHREADS_TASK_H
