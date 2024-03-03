
#include "tinyThreads_semaphore.h"
#include "tinyThreads_core.h"
bool tt_SemaphoreTake(tinyThread_binary_semaphore_t *sem, tinyThreadsTime_ms_t timeout_ms)
{
    bool retVal = false;
    // if semaphore is locked, wait for it to be unlocked
    if (sem->locked)
    {
        // hijacking sleep function to wait for semaphore to be unlocked
        tt_ThreadSleepState(timeout_ms, THREAD_STATE_SEMAPHORE_WAIT);
        retVal = false;
    }
    else
    {
        sem->locked = true;
        retVal = true;
    }

    return retVal;
}

void tt_SemaphoreGive(tinyThread_binary_semaphore_t *sem)
{
    sem->locked = false;
}