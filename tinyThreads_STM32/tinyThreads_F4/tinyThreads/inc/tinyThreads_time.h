#ifndef TINYTHREADS_TIME_H
#define TINYTHREADS_TIME_H

#include "tinyThreads_config.h"
#include "tinyThreads_debug.h"
#include "tinyThreads_error.h"
#include "tinyThreads_port.h"

/***************| system tick macros |**********************/
extern uint32_t tinyThread_tick;
#define tt_tick_inc()    (++tinyThread_tick)
#define tt_TimeGetTick() (tinyThread_tick)

#define tt_TimeGetTickElapsedMs(previousTick) ((uint32_t)(tt_TimeGetTick() - (uint32_t)previousTick))
// This can be wrong is called more than once per tick
#define tt_TimeGetTickApproxJitterMs(tick, expected) ((uint32_t)(tt_TimeGetTickElapsedMs(tick) - (uint32_t)expected))

#endif // TINYTHREADS_TIME_H
