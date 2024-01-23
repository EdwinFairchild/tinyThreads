#include "tinyKernel.h"

tinyThreadsTime_ms_t tinyThreads_getThreadLastRunTime(void)
{
    return tinyKernel_getThreadLastRunTime();
}