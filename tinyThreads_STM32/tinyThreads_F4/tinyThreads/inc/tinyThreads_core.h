#ifndef TINYKERNEL_H
#define TINYKERNEL_H

// clang-format off
#include "stdarg.h"
#include "stdbool.h"
#include "stddef.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"

#include "tinyThreads_types.h"
#include "tinyThreads_config.h"
#include "tinyThreads_debug.h"
#include "tinyThreads_error.h"
#include "tinyThreads_memory.h"
#include "tinyThreads_port.h"
#include "tinyThreads_semaphore.h"
#include "tinyThreads_system.h"
#include "tinyThreads_thread.h"
#include "tinyThreads_time.h"
#include "tinyThreads_timers.h"

#include "stdlib.h"
// clang-format on
/******************************************************************************
************************| TintyThread thread count |***************************
******************************************************************************/
#define TT_SYSTEM_THREADS 1
#define TT_MAX_THREADS    (CFG_TINYTHREADS_NUMBER_OF_THREADS + TT_SYSTEM_THREADS)
/***************************************************************************
**************| Exception frame |*******************************************
****************************************************************************
These are the registers that are pushed onto the stack
when an exception occurs. The stack grows from high
addresses to low addresses. The stack pointer points to
the top of the stack. The exception frame is pushed onto
the stack in the following order:
xPSR - top of stack (highest address value) Top of stack
PC (R15)
LR (R14)
R12
R3
R2
R1
R0 (lowest address value) */

#define TT_TOTAL_STACK_SIZE               (TT_MAX_THREADS * CFG_TINYTHREADS_STACK_SIZE)
#define TT_TOP_OF_STACK(stacksize)        (stacksize - 1U)
#define TT_EXCEPTION_FRAME_PSR(stacksize) TT_TOP_OF_STACK(stacksize)
#define TT_EXCEPTION_FRAME_PC(stacksize)  (TT_TOP_OF_STACK(stacksize) - 1U)
#define TT_EXCEPTION_FRAME_LR(stacksize)  (TT_TOP_OF_STACK(stacksize) - 2U)
#define TT_EXCEPTION_FRAME_R12(stacksize) (TT_TOP_OF_STACK(stacksize) - 3U)
#define TT_EXCEPTION_FRAME_R3(stacksize)  (TT_TOP_OF_STACK(stacksize) - 4U)
#define TT_EXCEPTION_FRAME_R2(stacksize)  (TT_TOP_OF_STACK(stacksize) - 5U)
#define TT_EXCEPTION_FRAME_R1(stacksize)  (TT_TOP_OF_STACK(stacksize) - 6U)
#define TT_EXCEPTION_FRAME_R0(stacksize)  (TT_TOP_OF_STACK(stacksize) - 7U)

/******************************************************************************
*****************| TintyThread control block member offsets |******************
******************************************************************************/
// This is to not use magic numbers in the assembly code
#define TINYTASK_TCB_SP_OFFSET              ((size_t)offsetof(tinyThread_tcb, stackPointer))
#define TINYTASK_TCB_NEXT_PTR_OFFSET        ((size_t)offsetof(tinyThread_tcb, next))
#define TINYTASK_TCB_PREV_PTR_OFFSET        ((size_t)offsetof(tinyThread_tcb, prev))
#define TINYTASK_TCB_PERIOD_OFFSET          ((size_t)offsetof(tinyThread_tcb, period_ms))
#define TINYTASK_TCB_LAST_RUNTIME_OFFSET    ((size_t)offsetof(tinyThread_tcb, lastRunTime))
#define TINYTASK_TCB_TASK_PRIORITY_OFFSET   ((size_t)offsetof(tinyThread_tcb, priority))
#define TINYTASK_TCB_TASK_STATE_OFFSET      ((size_t)offsetof(tinyThread_tcb, state))
#define TINYTASK_TCB_NOTIFY_VAL_OFFSET      ((size_t)offsetof(tinyThread_tcb, notifyVal))
#define TINYTASK_TCB_NOTIFY_CONSUMED_OFFSET ((size_t)offsetof(tinyThread_tcb, notifyConsumed))
#define TINYTASK_TCB_SLEEP_COUNT_OFFSET     ((size_t)offsetof(tinyThread_tcb, sleep_count_ms))
#define TINYTASK_TCB_NOTIFY_TIMEOUT_COUNT   ((size_t)offsetof(tinyThread_tcb, notify_timeout_count))
#define TINYTASK_TCB_ID_OFFSET              ((size_t)offsetof(tinyThread_tcb, id))
#define TINYTASK_TCB_NAME_OFFSET            ((size_t)offsetof(tinyThread_tcb, name))
/*****************************************************************************
******************************************************************************/

/*************************************************
 * Critical sections should be nested
 * and only the outermost critical section
 * should enable/disable interrupts
 *************************************************/
void tt_CoreCsEnter(void);
void tt_CoreCsExit(void);

TinyThreadsStatus tt_CoreInit(void);
TinyThreadsStatus tt_CoreRun(void);

void tt_CoreSystemTickHandler(void);

#endif // TINYKERNEL_H
