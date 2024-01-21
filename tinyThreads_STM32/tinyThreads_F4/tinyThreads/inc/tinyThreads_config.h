#ifndef TINYTHREADS_CONFIG_H
#define TINYTHREADS_CONFIG_H

#include <stdint.h>
#define SYSTEM_CLCOCK_FREQ_MHZ 80
#define TINYTHREADS_NUMBER_OF_TASKS  2
#define TINYTHREADS_STACK_SIZE 1000

/* ---------- Scheduling ----------*/
#define TINYTASK_ROUND_ROBIN 1

/* ---------- Debugging ----------*/
#define TINYTHREADS_DEBUG 1


#endif // TINYTHREADS_CONFIG_H
