#ifndef TINYTHREADS_CONFIG_H
#define TINYTHREADS_CONFIG_H

#include <stdint.h>
#define CFG_SYSTEM_CLOCK                  80
#define CFG_TINYTHREADS_NUMBER_OF_THREADS 4
#define CFG_TINYTHREADS_STACK_SIZE        1000

// Memory pool configuration - Application specific as needed
#define CFG_MEM_POOL_LARGE_SIZE_BYTES  256
#define CFG_MEM_POOL_MEDIUM_SIZE_BYTES 64
#define CFG_MEM_POOL_SMALL_SIZE_BYTES  32
// Define the number of blocks for each size
#define CFG_MEM_POOLNUM_SMALL_BLOCKS  10
#define CFG_MEM_POOLNUM_MEDIUM_BLOCKS 10
#define CFG_MEM_POOLNUM_LARGE_BLOCKS  10

/* ---------- Scheduling ----------*/
#define CFG_TINYTASK_ROUND_ROBIN 1

/* ---------- Debugging ----------*/
#define CFG_TINYTHREADS_DEBUG 1

#endif // TINYTHREADS_CONFIG_H
