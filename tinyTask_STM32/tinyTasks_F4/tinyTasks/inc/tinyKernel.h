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

/* Macros for readability */
/***************| Exception frame |**********************
xPSR - top of stack
PC (R15) - top of stack -1
LR (R14)
R12
R3
R2
R1
R0
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
