#ifndef TINYTHREADS_CONFIG_H
#define TINYTHREADS_CONFIG_H

#include <stdint.h>
#define CFG_SYSTEM_CLOCK                  80
#define CFG_TINYTHREADS_NUMBER_OF_THREADS 4
#define CFG_TINYTHREADS_STACK_SIZE        1000
#define CFG_TINYTHREADS_MAX_NAME_LENGTH   16
#define CFG_TINYTHREADS_STACK_INIT_VALUE  0xDEADBEEF

// Memory pool configuration - Application specific as needed
#define CFG_MEM_POOL_LARGE_SIZE_BYTES  256
#define CFG_MEM_POOL_MEDIUM_SIZE_BYTES 64
#define CFG_MEM_POOL_SMALL_SIZE_BYTES  32

// Define the value to use for unused memory
#define CFG_MEM_POOL_UNUSED_VALUE 0xFFFFFFFF

// Define the number of blocks for each size
#define CFG_MEM_POOLNUM_SMALL_BLOCKS  10
#define CFG_MEM_POOLNUM_MEDIUM_BLOCKS 10
#define CFG_MEM_POOLNUM_LARGE_BLOCKS  10

/* ---------- Scheduling ----------*/
#define CFG_TINYTASK_ROUND_ROBIN 1

/* ---------- Debugging ----------*/
#define CFG_TINYTHREADS_DEBUG               1
#define CFG_TINYTHREADS_DEBUG_HALT_ON_ERROR 1
#endif // TINYTHREADS_CONFIG_H
