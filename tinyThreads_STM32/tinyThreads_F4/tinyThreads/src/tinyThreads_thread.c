#include "tinyThreads_thread.h"
#include "tinyThreads_core.h"

static tinyThread_tcb_t tinyThread_thread_ctl[TT_MAX_THREADS];
// used to keep track of current tcb
tinyThread_tcb *tinyThread_current_tcb;
// used to keep track of next tcb
tinyThread_tcb *tinyThread_next_tcb = NULL;

static tinyThread_ready_threads_list_t tinyThread_ready_threads_list;

// make a list of suspended threads
static tinyThread_suspended_threads_list_t tinyThread_suspended_threads_list;

// keep track of number of threads, also used as id
static tinyThread_tcb_idx tinyThreads_thread_Count = 0;

static uint32_t tinyThread_inactive_thread_count = 0;
// system tick
tinyThreadsTime_ms_t tinyThread_tick = 0;
//**************************************************************************
static bool tt_isValidTcb(tinyThread_tcb_idx id)
{
    bool valid = false;
    if (id < tinyThreads_thread_Count)
    {
        valid = true;
    }
    return valid;
}
//**************************************************************************
static TinyThreadsStatus tinyThread_ready_thread_add_ll(tinyThread_tcb_idx id)
{
    TinyThreadsStatus err = TINYTHREADS_OK;

    if (tinyThread_ready_threads_list.tcb_ll_head == NULL && tinyThread_ready_threads_list.tcb_ll_tail == NULL)
    {
        // this is first task (system task)
        tinyThread_ready_threads_list.tcb_ll_tail = &tinyThread_thread_ctl[tinyThreads_thread_Count];
        tinyThread_ready_threads_list.tcb_ll_head = tinyThread_ready_threads_list.tcb_ll_tail;
    }
    else
    {
        tinyThread_ready_threads_list.tcb_ll_tail->next = &tinyThread_thread_ctl[tinyThreads_thread_Count];
        tinyThread_ready_threads_list.tcb_ll_tail = &tinyThread_thread_ctl[tinyThreads_thread_Count];
        tinyThread_ready_threads_list.tcb_ll_tail->next = tinyThread_ready_threads_list.tcb_ll_head;
    }

    debug(err);
    return err;
}
//**************************************************************************
static TinyThreadsStatus tinyThread_addThreadToNonReadyList(tinyThread_tcb_idx id)
{
    TinyThreadsStatus err = TINYTHREADS_OK;
    tt_CoreCsEnter();
    // make new node
    tinyThread_suspended_threads_node_t *temp =
        (tinyThread_suspended_threads_node_t *)tt_MemoryAllocBuf(sizeof(tinyThread_suspended_threads_node_t));
    if (temp != NULL)
    {

        temp->tcb = &tinyThread_thread_ctl[id];
        // STATE should be set before calling this function, since task can be
        // non ready for multiple reasons
        // thus im not setting it here
        temp->next = NULL;
        temp->prev = NULL;
        // check if head is null
        if (tinyThread_suspended_threads_list.head == NULL)
        {
            tinyThread_suspended_threads_list.head = temp;
            tinyThread_suspended_threads_list.tail = NULL;
        }
        else
        {
            if (tinyThread_suspended_threads_list.tail == NULL)
            {
                tinyThread_suspended_threads_list.tail = temp;
                tinyThread_suspended_threads_list.head->next = tinyThread_suspended_threads_list.tail;
                tinyThread_suspended_threads_list.tail->prev = tinyThread_suspended_threads_list.head;
            }
            else
            {
                tinyThread_suspended_threads_list.tail->next = temp;
                temp->prev = tinyThread_suspended_threads_list.tail;
                tinyThread_suspended_threads_list.tail = temp;
            }
        }
        tinyThread_inactive_thread_count++;
    }
    else
    {
        err = TINYTHREADS_ERROR;
    }
    tt_CoreCsExit();
    return err;
}
//**************************************************************************
static TinyThreadsStatus tinyThread_removeThreadFromNonReadyList(tinyThread_tcb_idx id)
{
    TinyThreadsStatus err = TINYTHREADS_ERROR;
    // make new node
    tinyThread_suspended_threads_node_t *temp = tinyThread_suspended_threads_list.head;
    tinyThread_suspended_threads_node_t *nodeToremove = NULL;
    if (temp != NULL && tinyThread_inactive_thread_count > 0)
    {

        // handle if node to remove is head
        if (temp->tcb->id == id)
        {
            // save the head pointer so we can free it
            nodeToremove = tinyThread_suspended_threads_list.head;
            // new head is the next node
            tinyThread_suspended_threads_list.head = tinyThread_suspended_threads_list.head->next;
        }
        // handle if node to remove is tail
        else if (tinyThread_suspended_threads_list.tail->tcb->id == id)
        {
            // save the tail pointer
            nodeToremove = tinyThread_suspended_threads_list.tail;
            // new tail is the prev node
            tinyThread_suspended_threads_list.tail = tinyThread_suspended_threads_list.tail->prev;
        }
        // handle if node to remove is in the middle
        else
        {
            // Traverse the list to find the node
            while (temp != NULL && temp->tcb->id != id)
            {
                temp = temp->next;
            }
            if (temp != NULL)
            {
                // remove the node by bypassing self
                temp->prev->next = temp->next;
                temp->next->prev = temp->prev;
                nodeToremove = temp;
            }
            else
            {
                err = TINYTHREADS_ERROR;
            }
        }
        if (nodeToremove != NULL)
        {
            tt_MemoryFreeBuf(nodeToremove);
            tinyThread_inactive_thread_count--;
            err = TINYTHREADS_OK;
        }
    }
    debug(err);
    return err;
}
//**************************************************************************
static TinyThreadsStatus tinyThread_canAddThread(void)
{
    TinyThreadsStatus err = TINYTHREADS_OK;
    if (!(tinyThreads_thread_Count <= (TT_MAX_THREADS)))
    {
        err = TINYTHREADS_MAX_THREADS_REACHED;
    }
    debug(err);
    return err;
}

//**************************************************************************
static TinyThreadsStatus tt_ThreadStackInit(uint32_t threadIDX, uint32_t *stackPtr, uint32_t stacksize,
                                            void (*thread)(uint32_t))
{
    TinyThreadsStatus err = TINYTHREADS_OK;
    uint8_t numberOffRegistersToSave = 0;
    #if CFG_TINYTHREADS_SAVE_OPTIONAL_REGISTERS == 1
    numberOffRegistersToSave = 16;;
    #else
    numberOffRegistersToSave = 8;
    #endif
    if (stackPtr != NULL && stacksize != 0 && thread != NULL && threadIDX < TT_MAX_THREADS)
    {
        // initialize the stack pointer
        // R13 is stack pointer, we manage the register manually using the stack pointer below
        tinyThread_thread_ctl[threadIDX].stackPointer = (uint32_t *)&stackPtr[stacksize - numberOffRegistersToSave];
        tinyThread_thread_ctl[threadIDX].lastRunTime = tt_TimeGetTick();
        // initialize PC (program crounter) to the thread function entry point
        // However during context switching  we will push the PC (which will vary) to the stack
        // and pop it off when we want to return to the thread at its proper place
        stackPtr[TT_EXCEPTION_FRAME_PC(stacksize)] = (uint32_t)thread;
        //set T-bit to 1 to make sure we run in thumb mode : see core_cm4.h > xPSR_Type
        stackPtr[TT_EXCEPTION_FRAME_PSR(stacksize)] |= (1 << 24); // xPSR
        stackPtr[TT_EXCEPTION_FRAME_LR(stacksize)] = CFG_TINYTHREADS_STACK_INIT_VALUE;  // R14 (LR)
        stackPtr[TT_EXCEPTION_FRAME_R12(stacksize)] = CFG_TINYTHREADS_STACK_INIT_VALUE; // R12
        stackPtr[TT_EXCEPTION_FRAME_R3(stacksize)] = CFG_TINYTHREADS_STACK_INIT_VALUE;  // R3
        stackPtr[TT_EXCEPTION_FRAME_R2(stacksize)] = CFG_TINYTHREADS_STACK_INIT_VALUE;  // R2
        stackPtr[TT_EXCEPTION_FRAME_R1(stacksize)] = CFG_TINYTHREADS_STACK_INIT_VALUE;  // R1
        stackPtr[TT_EXCEPTION_FRAME_R0(stacksize)] = CFG_TINYTHREADS_STACK_INIT_VALUE;  // R0

        // R4-R11 are general purpose registers that are optional to save
        #if CFG_TINYTHREADS_SAVE_OPTIONAL_REGISTERS == 1
        stackPtr[stacksize - 9] = CFG_TINYTHREADS_STACK_INIT_VALUE;  // R11
        stackPtr[stacksize - 10] = CFG_TINYTHREADS_STACK_INIT_VALUE; // R10
        stackPtr[stacksize - 11] = CFG_TINYTHREADS_STACK_INIT_VALUE; // R9
        stackPtr[stacksize - 12] = CFG_TINYTHREADS_STACK_INIT_VALUE; // R8
        stackPtr[stacksize - 13] = CFG_TINYTHREADS_STACK_INIT_VALUE; // R7
        stackPtr[stacksize - 14] = CFG_TINYTHREADS_STACK_INIT_VALUE; // R6
        stackPtr[stacksize - 15] = CFG_TINYTHREADS_STACK_INIT_VALUE; // R5
        stackPtr[stacksize - 16] = CFG_TINYTHREADS_STACK_INIT_VALUE; // R4
        #endif
    }
    else
    {
        err = TINYTHREADS_ERROR;
    }
    debug(err);
    return err;
}
//**************************************************************************
tinyThread_tcb_idx tt_ThreadAdd(void (*thread)(uint32_t), uint32_t *stackPtr, uint32_t stackSize,
                                tinyThreadsTime_ms_t period, tinyThreadPriority_t priority, uint8_t *name, bool ready)
{
    TinyThreadsStatus err = TINYTHREADS_OK;
    // var to store id of thread
    tinyThread_tcb_idx id = 0;
    /* disable interrupts */
    tt_CoreCsEnter();
    // TODO : check all params are within configured limts and return propper errors
    if (tinyThread_canAddThread() == TINYTHREADS_OK && thread != NULL)
    {
        // intiialize thread control block
        tinyThread_thread_ctl[tinyThreads_thread_Count].period_ms = period;
        tinyThread_thread_ctl[tinyThreads_thread_Count].priority = priority;
        tinyThread_thread_ctl[tinyThreads_thread_Count].state = THREAD_STATE_READY;
        tinyThread_thread_ctl[tinyThreads_thread_Count].prev = NULL;
        tinyThread_thread_ctl[tinyThreads_thread_Count].lastRunTime = (tinyThreadsTime_ms_t)0;
        tinyThread_thread_ctl[tinyThreads_thread_Count].id = tinyThreads_thread_Count;
        tinyThread_thread_ctl[tinyThreads_thread_Count].notifyVal = NULL;
        tinyThread_thread_ctl[tinyThreads_thread_Count].sleep_count_ms = 0;
        tinyThread_thread_ctl[tinyThreads_thread_Count].notify_timeout_count = 0;
        // copy name using memcpy
        memcpy(tinyThread_thread_ctl[tinyThreads_thread_Count].name, name,
               sizeof(tinyThread_thread_ctl[tinyThreads_thread_Count].name));

        if (ready)
        {
            tinyThread_ready_thread_add_ll(tinyThreads_thread_Count);
        }
        else
        {
            tinyThread_addThreadToNonReadyList(tinyThreads_thread_Count);
        }

        tt_ThreadStackInit(tinyThreads_thread_Count, stackPtr, stackSize, thread);
        
        id = tinyThreads_thread_Count;
        tinyThreads_thread_Count++;
    }
    else
    {
        err = TINYTHREADS_MAX_THREADS_REACHED;
    }
    /* enable interrupts */
    tt_CoreCsExit();
    if (err != TINYTHREADS_OK)
    {
        id = err;
    }
    debug(err);
    return id;
}
//**************************************************************************
tinyThreadsTime_ms_t tt_ThreadGetLastRunTime()
{
    return tinyThread_current_tcb->lastRunTime;
}
//**************************************************************************
void tt_ThreadYield(bool updateNext)
{
    // in some cases we will have already update the next tcb
    if (updateNext)
    {
        tt_ThreadUpdateNextThreadPtr();
    }
    FORCE_YEILD_INTERRUPT();
}
//**************************************************************************
void tt_ThreadUpdateNextThreadPtr(void)
{
    // TODO need to make this more efficient, and priority aware
    // travese the tcbs to find the next ready thread
    tinyThread_next_tcb = tinyThread_current_tcb->next;
    while (tinyThread_next_tcb->state != THREAD_STATE_READY)
    {
        // we didnt break out of loop so keep looking for next ready thread
        tinyThread_next_tcb = tinyThread_next_tcb->next;
    }
}
//**************************************************************************
TinyThreadsStatus tt_ThreadUpdateInactive(void)
{
    TinyThreadsStatus err = TINYTHREADS_OK;
    // travese the non ready threads list and update the sleep counter
    tinyThread_suspended_threads_node_t *temp = tinyThread_suspended_threads_list.head;
    while (temp != NULL)
    {
        switch (temp->tcb->state)
        {
        case THREAD_STATE_SLEEPING:
            if (temp->tcb->sleep_count_ms > 0)
            {
                temp->tcb->sleep_count_ms--;
            }
            else
            {
                // set state to ready
                temp->tcb->state &= ~THREAD_STATE_SLEEPING;
                // remove from non ready list
                tinyThread_removeThreadFromNonReadyList(temp->tcb->id);
            }
            break;
        case THREAD_STATE_SEMAPHORE_WAIT:
            // semaphore_timeout_count
            if (temp->tcb->semaphore_timeout_count > 0)
            {
                temp->tcb->semaphore_timeout_count--;
            }
            else
            {
                // set state to ready
                temp->tcb->state &= ~THREAD_STATE_SEMAPHORE_WAIT;
                // remove from non ready list
                tinyThread_removeThreadFromNonReadyList(temp->tcb->id);
            }
            // do nothing
            break;
        case THREAD_STATE_PAUSED:
            // do nothing,
            // thread is paused directly from tt_ThreadPause
            break;
        case THREAD_STATE_READY:
            // do nothing
            break;

        case THREAD_STATE_BLOCKED:
            // do nothing
            break;
        case THREAD_STATE_PENDING_NOTIFY:
            if (temp->tcb->notify_timeout_count)
            {
                temp->tcb->notify_timeout_count--;
            }
            else
            {
                // set state to ready
                temp->tcb->state &= ~THREAD_STATE_PENDING_NOTIFY;
                // remove from non ready list
                tinyThread_removeThreadFromNonReadyList(temp->tcb->id);
            }
            break;
        }
        temp = temp->next;
    }
    debug(err);
    return err;
}
//**************************************************************************
void tt_ThreadSleep(uint32_t time_ms)
{
    tt_ThreadSleepState(time_ms, THREAD_STATE_SLEEPING);
}
//**************************************************************************
void tt_ThreadSleepState(uint32_t time_ms, tinyThread_state state)
{
    TinyThreadsStatus err = TINYTHREADS_OK;
    tt_CoreCsEnter();
    // set state to sleeping
    tinyThread_current_tcb->state |= state;
    if (state == THREAD_STATE_SLEEPING)
    {

        // set sleep counter
        tinyThread_current_tcb->sleep_count_ms = time_ms;
    }
    else if (state == THREAD_STATE_SEMAPHORE_WAIT)
    {
        tinyThread_current_tcb->semaphore_timeout_count = time_ms;
    }
    err = tinyThread_addThreadToNonReadyList(tinyThread_current_tcb->id);
    tt_CoreCsExit();
    debug(err);
    tt_ThreadYield(true);
}
//**************************************************************************
TinyThreadsStatus tt_ThreadWake(tinyThread_tcb_idx id)
{
    TinyThreadsStatus err = TINYTHREADS_ERROR;
    if (tt_isValidTcb(id) && (tinyThread_thread_ctl[id].state & THREAD_STATE_SLEEPING))
    {
        tt_CoreCsEnter();
        // clear sleeping state
        tinyThread_thread_ctl[id].state &= ~THREAD_STATE_SLEEPING;
        // set sleep time to 0
        // tinyThread_thread_ctl[id].sleep_count_ms = 0;
        // remove from non ready list
        err = tinyThread_removeThreadFromNonReadyList(id);
        tt_CoreCsExit();
    }
    debug(err);
    return err;
}
//**************************************************************************
TinyThreadsStatus tt_ThreadPause(tinyThread_tcb_idx id)
{
    TinyThreadsStatus err = TINYTHREADS_ERROR;
    if (tt_isValidTcb(id))
    {
        tt_CoreCsEnter();
        // set state to paused
        tinyThread_thread_ctl[id].state |= THREAD_STATE_PAUSED;
        err = TINYTHREADS_OK;
        tt_CoreCsExit();
    }
    debug(err);
    return err;
}
//**************************************************************************
TinyThreadsStatus tt_ThreadUnPause(tinyThread_tcb_idx id)
{
    // TODO : this will put any thread in the ready state, even if it was blocked
    // or sleeping, should I check for that? do I want a global resume?
    TinyThreadsStatus err = TINYTHREADS_ERROR;
    if (tt_isValidTcb(id) && (tinyThread_thread_ctl[id].state & THREAD_STATE_PAUSED))
    {
        tt_CoreCsEnter();
        // clear paused bit
        tinyThread_thread_ctl[id].state &= ~THREAD_STATE_PAUSED;
        err = TINYTHREADS_OK;
        tt_CoreCsExit();
    }
    debug(err);
    return err;
}
//**************************************************************************
// TODO: do i want to keep this? internal use?
tinyThreadsTime_ms_t tt_ThreadGetSleepCount(tinyThread_tcb_idx id)
{
    // TODO: implement
    if (tt_isValidTcb(id))
    {
        return tinyThread_thread_ctl[id].sleep_count_ms;
    }
    // ID not valid
    debug(TINYTHREADS_INVALID_TASK);
    return 0;
}
//**************************************************************************
tinyThreadsTime_ms_t tt_ThreadGetNotifyTimeoutCount(tinyThread_tcb_idx id)
{
    if (tt_isValidTcb(id))
    {
        return tinyThread_thread_ctl[id].notify_timeout_count;
    }
    // ID not valid
    debug(TINYTHREADS_INVALID_TASK);
    return 0;
}
//**************************************************************************
tinyThread_tcb *tt_ThreadGetCurrentTcb(void)
{
    return tinyThread_current_tcb;
}
//**************************************************************************
tinyThread_tcb *tt_ThreadGetTcbByID(tinyThread_tcb_idx id)
{
    if (tt_isValidTcb(id))
    {
        return &tinyThread_thread_ctl[id];
    }
    // ID not valid
    debug(TINYTHREADS_INVALID_TASK);
    return NULL;
}
//**************************************************************************
TinyThreadsStatus tt_SetCurrentTcb(tinyThread_tcb *tcb)
{
    TinyThreadsStatus err = TINYTHREADS_OK;
    if (tcb != NULL)
    {
        if (tt_isValidTcb(tcb->id))
        {
            tinyThread_current_tcb = tcb;
        }
        else
        {
            err = TINYTHREADS_ERROR;
        }
    }
    debug(err);
    return err;
}
//**************************************************************************
TinyThreadsStatus tt_ThreadNotifyWait(tinyThreadsTime_ms_t timeout, uint32_t *val)
{

    TinyThreadsStatus err = TINYTHREADS_OK;
    tinyThread_tcb_t *tempTcb = tinyThread_current_tcb;

    // at this point we have not received anything so set notifyConsumed to false
    tempTcb->notifyConsumed = false;
    // if there is a timeout and there is no data then put the thread in non ready list
    if (timeout || tempTcb->notifyConsumed == false)
    {
        // set notify value to point to the value passed in
        tempTcb->notifyVal = val;
        // set state to pending notify
        tempTcb->state |= THREAD_STATE_PENDING_NOTIFY;
        // set timeout counter
        tempTcb->notify_timeout_count = timeout;
        // add to non ready list
        tinyThread_addThreadToNonReadyList(tempTcb->id);
        // yield
        tt_ThreadYield(true);
        // thread will return here after timeout or notification
    }

    // We get to this point if there is a notification or we timedout
    // or if timeout is 0 or if there is a notification
    // TODO : we should check if we timedout or not could be useful
    // do we need err status here?
    if (tempTcb->notifyVal != NULL)
    {
        // read the value
        *val = *(tempTcb->notifyVal);
        tempTcb->notifyConsumed = true;
    }
    tt_CoreCsExit();
    debug(err);
    return err;
}
//**************************************************************************
TinyThreadsStatus tt_ThreadNotify(tinyThread_tcb_idx taskID, uint32_t newVal, bool overwrite)
{
    tt_CoreCsEnter();
    TinyThreadsStatus err = TINYTHREADS_OK;
    if (tt_isValidTcb(taskID))
    {

        tinyThread_tcb_t *tempTcb = &tinyThread_thread_ctl[taskID];
        // write the data if overwrite is true or if data has been consumed
        if (overwrite || tempTcb->notifyConsumed == true)
        {
            *(tempTcb->notifyVal) = newVal;

            // check if task is still waiting for notification, otherwise it may have timedout
            // in hich case do nothing
            if (tempTcb->state & THREAD_STATE_PENDING_NOTIFY)
            {
                // clear the state
                tempTcb->state &= ~(THREAD_STATE_PENDING_NOTIFY);
                // reset its counter whether its being used or not
                tempTcb->notify_timeout_count = 0;

                // if clearing THREAD_STATE_PENDING_NOTIFY bit resulted in state being
                // THREAD_STATE_READY remove from non-ready list
                if (tempTcb->state == THREAD_STATE_READY)
                {
                    // remove from non ready list
                    tinyThread_removeThreadFromNonReadyList(tempTcb->id);
                    // TODO : im preemptive scheduling we need to see if this new ready
                    //  task is higher priority and if so for a context switch. or maybe
                    //  let user decide of context switch should happen immediately or not?
                }
            }
        }
        else
        {
            err = TINYTHREADS_NOTIFY_FAILED;
        }
    }
    tt_CoreCsExit();
    debug(err);
    return err;
}
//**************************************************************************
tinyThread_tcb_idx tt_ThreadGetCurrentID(void)
{
    return tinyThread_current_tcb->id;
}
//**************************************************************************
uint32_t tt_ThreadGetInactiveThreadCount(void)
{
    return tinyThread_inactive_thread_count;
}
//**************************************************************************
// TODO : remove debnugging helpers
void printNoneReadyList()
{
    tinyThread_suspended_threads_node_t *temp = tinyThread_suspended_threads_list.head;

    uint8_t count = 0;

    printf("Non ready threads------\r\n");
    while (temp != NULL)
    {
        printf("Thread[%d][%s]\r\n", count++, temp->tcb->name);
        temp = temp->next;
    }
}

void removeAllFromNoneReadyList()
{
    tinyThread_suspended_threads_node_t *temp = tinyThread_suspended_threads_list.head;
    while (temp != NULL)
    {
        tinyThread_removeThreadFromNonReadyList(temp->tcb->id);
        temp = temp->next;
    }
}