#ifndef TINYKERNEL_H
#define TINYKERNEL_H

#include "stdbool.h"
#include "stdio.h"
#include "stdint.h"
#include "stdarg.h"
#include "string.h"

#include "tinyTasksError.h"
#include "tinyTasksConfig.h"
#include "tinyTasksPort.h"
#include "tinyTasks_debug.h"
#include "tinyTasks_system.h"

/*************| Exception frame |********************
These are the registers that are pushed onto the stack
when an exception occurs. The stack grows from high
addresses to low addresses. The stack pointer points to
the top of the stack. The exception frame is pushed onto
the stack in the following order:
R0 (lowest address value)
R1
R2
R3
R12
LR (R14)
PC (R15)
xPSR - top of stack (highest address value)
*******************************************************/
#define TT_TOTAL_STACK_SIZE (TINYTASKS_MAX_TASKS * TINYTASKS_STACK_SIZE)
#define TT_TOP_OF_STACK (TINYTASKS_STACK_SIZE - 1U)
#define TT_EXCEPTION_FRAME_PSR TT_TOP_OF_STACK
#define TT_EXCEPTION_FRAME_PC (TT_TOP_OF_STACK - 1U)
#define TT_EXCEPTION_FRAME_LR (TT_TOP_OF_STACK - 2U)
#define TT_EXCEPTION_FRAME_R12 (TT_TOP_OF_STACK - 3U)
#define TT_EXCEPTION_FRAME_R3 (TT_TOP_OF_STACK - 4U)
#define TT_EXCEPTION_FRAME_R2 (TT_TOP_OF_STACK - 5U)
#define TT_EXCEPTION_FRAME_R1 (TT_TOP_OF_STACK - 6U)
#define TT_EXCEPTION_FRAME_R0 (TT_TOP_OF_STACK - 7U)

#define TINYTASKS_NUM_OF_SYS_TASKS 1
#define TINYTASKS_MAX_TASKS (TINYTASKS_NUMBER_OF_TASKS+TINYTASKS_NUM_OF_SYS_TASKS)

/***************| system tick macros |**********************/
extern uint32_t tinyTask_tick;
#define tinyTask_tick_inc() (++tinyTask_tick)
#define tinyTask_tick_reset() (tinyTask_tick = 0)

TinyTasksStatus tinyKernel_init(void);
TinyTasksStatus tinyKernel_task_stack_init(uint32_t taskIDX);
TinyTasksStatus tinyKernel_run(void);
TinyTasksStatus tinyKernel_addTask(void (*task)(void), uint32_t period);
void tinyTask_isr_task_switch(uint32_t tick);
// uint32_t tinyTask_tick_inc();
#endif // TINYKERNEL_H
