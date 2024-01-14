#ifndef TINYKERNEL_H
#define TINYKERNEL_H

#include "stdbool.h"
#include "stdio.h"
#include "stdint.h"
#include "stdarg.h"
#include "string.h"

#include "tinyTasksError.h"

TinyTasksStatus tinyKernel_init(void);
TinyTasksStatus tinyKernel_task_stack_init(uint32_t taskIDX);
TinyTasksStatus tinyKernel_run(void);
TinyTasksStatus tinyKetnell_addTask(void (*task)(void), uint32_t period);

#endif // TINYKERNEL_H
