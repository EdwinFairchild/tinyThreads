#ifndef CFG_TINYTHREADS_DEBUG_H
#define CFG_TINYTHREADS_DEBUG_H
// clang-format off
#include "tinyThreads_error.h"
#include "tinyThreads_config.h"
#include "tinyThreads_port.h"

// TODO: Makes the assumption user has retargeted printf to UART
// Debug macro definition
#if defined(CFG_TINYTHREADS_DEBUG) && (CFG_TINYTHREADS_DEBUG  == 1)
    #define debug(err) if(err != TINYTHREADS_OK){ \
    printf("!!! err @ %s: %s !!!\r\n", __func__, errorToString(err)); \
    } 

#else
    #define debug(err) ((void)0)
#endif

__STATIC_FORCEINLINE void port_dbg_signal_1_assert(void)
{
   
}
__STATIC_FORCEINLINE void port_dbg_signal_1_deassert(void)
{
 
}
#endif // CFG_TINYTHREADS_DEBUG_H
