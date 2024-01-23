#ifndef TINYTHREADS_TYPES_H
#define TINYTHREADS_TYPES_H

#include "stdint.h"

typedef uint32_t tinyThreadsTime_ms_t;
typedef uint32_t tinyThreadPeriod_t;
typedef uint32_t tinyThreadPriority_t;
typedef enum {
    THREAD_STATE_READY, 
    THREAD_STATE_BLOCKED,
    THREAD_STATE_PAUSED
} tinyThreadsState_t;


#endif // TINYTHREADS_TYPES_H
