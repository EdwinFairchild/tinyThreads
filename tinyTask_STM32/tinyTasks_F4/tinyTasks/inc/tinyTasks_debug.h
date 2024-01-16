#ifndef TINYTASKS_DEBUG_H
#define TINYTASKS_DEBUG_H

#include "tinyTasksError.h"
#include "tinyTasksConfig.h"

// TODO: Makes the assumption user has retargeted printf to UART
// Debug macro definition
#if defined(TINYTASKS_DEBUG) && (TINYTASKS_DEBUG  == 1)
    #define debug(err) if(err != TINYTASKS_OK){ \
    printf("!!! err @ %s: %s !!!\r\n", __func__, errorToString(err)); \
    } 

#else
    #define debug(err) ((void)0)
#endif

#endif // TINYTASKS_DEBUG_H
