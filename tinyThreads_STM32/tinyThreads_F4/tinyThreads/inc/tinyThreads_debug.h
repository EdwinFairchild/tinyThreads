#ifndef CFG_TINYTHREADS_DEBUG_H
#define CFG_TINYTHREADS_DEBUG_H

#include "tinyThreads_error.h"
#include "tinyThreads_config.h"

// TODO: Makes the assumption user has retargeted printf to UART
// Debug macro definition
#if defined(CFG_TINYTHREADS_DEBUG) && (CFG_TINYTHREADS_DEBUG  == 1)
    #define debug(err) if(err != TINYTHREADS_OK){ \
    printf("!!! err @ %s: %s !!!\r\n", __func__, errorToString(err)); \
    } 

#else
    #define debug(err) ((void)0)
#endif

#endif // CFG_TINYTHREADS_DEBUG_H
