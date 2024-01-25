#ifndef TINYTHREADS_CONFIG_H
#define TINYTHREADS_CONFIG_H

#include <stdint.h>
#define CFG_SYSTEM_CLOCK                  80
#define CFG_TINYTHREADS_NUMBER_OF_THREADS 3
#define CFG_TINYTHREADS_STACK_SIZE        1000

/* ---------- Scheduling ----------*/
#define CFG_TINYTASK_ROUND_ROBIN 1

/* ---------- Debugging ----------*/
#define CFG_TINYTHREADS_DEBUG 1

#endif // TINYTHREADS_CONFIG_H
