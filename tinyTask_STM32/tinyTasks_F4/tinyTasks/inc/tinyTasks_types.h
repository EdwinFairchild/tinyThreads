#ifndef TINYTASKS_TYPES_H
#define TINYTASKS_TYPES_H

#include "stdint.h"

typedef uint32_t tinyTasksTime_t;
typedef uint32_t tinyTaskPeriod_t;
typedef uint32_t tinyTaskPriority_t;
typedef enum {
    TASK_STATE_READY, 
    TASK_STATE_RUNNING, 
    TASK_STATE_BLOCKED,
    TASK_STATE_PAUSED
} tinyTasksState_t;


#endif // TINYTASKS_TYPES_H
