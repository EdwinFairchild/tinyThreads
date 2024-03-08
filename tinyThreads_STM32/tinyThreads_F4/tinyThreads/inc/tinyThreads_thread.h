#ifndef TINYTHREADS_TASK_H
#define TINYTHREADS_TASK_H

#include "tinyThreads_error.h"
#include "tinyThreads_types.h"

/**************************************************************************
 * Add a thread to the scheduler
 * @param thread - function pointer to the thread
 * @param stackPtr - pointer to the stack
 * @param stackSize - size of the stack
 * @param period - period of the thread
 * @param priority - priority of the thread
 * @param name - name of the thread
 * @param ready - if the thread is ready to run
 * @return tinyThread_tcb_idx - index of the thread
 * @return -1 - if the thread could not be added
 * @return -2 - if the priority is invalid
 *
 *
 **************************************************************************/
tinyThread_tcb_idx tt_ThreadAdd(void (*thread)(uint32_t), uint32_t *stackPtr, uint32_t stackSize,
                                tinyThreadsTime_ms_t period, tinyThreadPriority_t priority, uint8_t *name, bool ready);

/**************************************************************************
 * TODO :
 *
 *
 **************************************************************************/
void tt_ThreadSleep(uint32_t time_ms);

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
tinyThreadsTime_ms_t tt_ThreadGetNotifyTimeoutCount(tinyThread_tcb_idx id);

/**************************************************************************
 * TODO :
 *
 *
 **************************************************************************/
tinyThread_tcb *tt_ThreadGetCurrentTcb(void);

/**************************************************************************
 * TODO :
 *
 *
 **************************************************************************/
void tt_ThreadUpdateNextThreadPtr(void);

/**************************************************************************
 * TODO :
 *
 *
 **************************************************************************/
TinyThreadsStatus tt_ThreadUpdateInactive(void);

/**************************************************************************
 * TODO :
 *
 *
 **************************************************************************/
void tt_ThreadYield(bool updateNext);

/**************************************************************************
 * TODO :
 *
 *
 **************************************************************************/
tinyThread_tcb_idx tt_ThreadGetCurrentID(void);

/**************************************************************************
 * TODO :
 *
 *
 **************************************************************************/
TinyThreadsStatus tt_SetCurrentTcb(tinyThread_tcb *tcb);

/**************************************************************************
 * TODO :
 *
 *
 **************************************************************************/
tinyThread_tcb *tt_ThreadGetTcbByID(tinyThread_tcb_idx id);
/**************************************************************************
 * TODO :
 *
 *
 **************************************************************************/
TinyThreadsStatus tt_ThreadNotifyWait(tinyThreadsTime_ms_t timeout, uint32_t *val);

/**************************************************************************
 * TODO :
 *
 *
 **************************************************************************/
TinyThreadsStatus tt_ThreadNotify(tinyThread_tcb_idx taskID, uint32_t newVal, bool overwrite);

/**************************************************************************
 * TODO :
 *
 *
 **************************************************************************/
uint32_t tt_ThreadGetInactiveThreadCount(void);

/**************************************************************************
 * TODO :
 *
 *
 **************************************************************************/
void tt_ThreadSleepState(uint32_t time_ms, tinyThread_state state);

#endif // TINYTHREADS_TASK_H
