#ifndef TINYTASKS_CONFIG_H
#define TINYTASKS_CONFIG_H

#include <stdint.h>

#define TT_MAX_TASKS 10
#define TT_STACK_SIZE 0x1000
#define TOTAL_STACK_SIZE (TT_MAX_TASKS * TT_STACK_SIZE)

/* Reserve stack space */
uint32_t tinyTask_stack[TT_MAX_TASKS][TOTAL_STACK_SIZE];


#endif // TINYTASKS_CONFIG_H
