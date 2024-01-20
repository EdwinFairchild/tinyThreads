#ifndef TINYTASKS_TYPES_H
#define TINYTASKS_TYPES_H

#include "stdint.h"

typedef uint32_t tinyThreadsTime_t;
typedef uint32_t tinyThreadPeriod_t;
typedef uint32_t tinyThreadPriority_t;
typedef enum {
    TASK_STATE_READY, 
    TASK_STATE_RUNNING, 
    TASK_STATE_BLOCKED,
    TASK_STATE_PAUSED
} tinyThreadsState_t;


#endif // TINYTASKS_TYPES_H
