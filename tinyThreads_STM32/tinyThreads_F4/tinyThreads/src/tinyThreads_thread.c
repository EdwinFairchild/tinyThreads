#include "tinyThreads_thread.h"
#include "tinyThreads_core.h"

// TODO: remove Debugging-----------

//---------------------

/* Reserve stack space, include user thread space and system thread space */
// TODO : shoould I make a seprate system stack space ? If not , user musct know system will use some of
// their stack , so do not calcualte stack on their threads alone.
uint32_t tinyThread_stack[TT_MAX_THREADS][TT_TOTAL_STACK_SIZE];

/* Static allocation of Thread Control Block Array*/
// TODO : make user allocate their TCBs in RAM
// it will be laste wasteful than me allocating TT_MAX_THREADS amount
// this will also allow them to decide stack size
// list of all thread control blocks
// however this means task list now should be a linked list instead of this array
// since i wont know array size before hand
static tinyThread_tcb_t tinyThread_thread_ctl[TT_MAX_THREADS];
// used to keep track of current tcb
// TODO : should I make a set curtrent tcb function?
tinyThread_tcb *tinyThread_current_tcb; //= &tinyThread_thread_ctl[0];
// used to keep track of next tcb
tinyThread_tcb *tinyThread_next_tcb = NULL;

static tinyThread_ready_threads_list_t tinyThread_ready_threads_list;

// make a list of suspended threads
static tinyThread_suspended_threads_list_t tinyThread_suspended_threads_list;

// keep track of number of threads, also used as id
static tinyThread_tcb_idx tinyThreads_thread_Count = 0;

static uint32_t tinyThread_inactive_thread_count = 0;
/***************| system tick |**********************/
tinyThreadsTime_ms_t tinyThread_tick = 0;

static bool tt_isValidTcb(tinyThread_tcb_idx id)
{
    bool valid = false;
    if (id < tinyThreads_thread_Count)
    {
        valid = true;
    }
    return valid;
}
/**************************************************************************
 * linked list functions for : tcb linked list and non ready threads linked list
 ***************************************************************************/

/**************************************************************************
 * Attempt to add a new thread to the thread control block linked list
 * Can only get here through tt_ThreadAdd
 * which veryfies tinyThread_canAddThread
 * this linked list contains all threads in all states
 * return : TinyThreadsStatus
 ***************************************************************************/
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

    return err;
}

/**************************************************************************
 * Add a tcb to the non ready list array
 * This list only contains threads that are not ready to run
 * return : TinyThreadsStatus
 **************************************************************************/
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
        // temp->tcb->state |= THREAD_STATE_SLEEPING;
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
/**************************************************************************
 * Remove a tcb from the non ready list
 * This list only contains threads that are not ready to run
 * return : TinyThreadsStatus
 **************************************************************************/
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

    return err;
}

/**************************************************************************
 * Check if max num of threads have been added:
 * return : TinyThreadsStatus
 **************************************************************************/
static TinyThreadsStatus tinyThread_canAddThread(void)
{
    TinyThreadsStatus err = TINYTHREADS_OK;
    if (!(tinyThreads_thread_Count <= (TT_MAX_THREADS)))
    {
        err = TINYTHREADS_MAX_THREADS_REACHED;
    }
    return err;
}

/**************************************************************************
 * - Initializes the stack for a thread to mostly dummy values
 *  -set T-bit to 1 to make sure we run in thumb mode : see core_cm4.h > xPSR_Type
 * stacks are in decending order
 * this is why we set the stack pointer to the last element of the stack
 ***************************************************************************/
static TinyThreadsStatus tt_ThreadStackInit(uint32_t threadIDX)
{
    TinyThreadsStatus err = TINYTHREADS_OK;

    /*  initialize the stack pointer
        R13 is stack pointer, we manages the register manually using the stack pointer blow */
    tinyThread_thread_ctl[threadIDX].stackPointer = &tinyThread_stack[threadIDX][CFG_TINYTHREADS_STACK_SIZE - 16];
    tinyThread_thread_ctl[threadIDX].lastRunTime = tinyThread_tick_get();

    tinyThread_stack[threadIDX][TT_EXCEPTION_FRAME_PSR] |= (1 << 24); // xPSR --------
    /*  The PC get initialized in tt_ThreadAdd                               |
    tinyThread_stack[threadIDX][CFG_TINYTHREADS_STACK_SIZE - 2] = 0x12345678; // PC */ //    |
    tinyThread_stack[threadIDX][TT_EXCEPTION_FRAME_LR] = 0xe2345678;  // R14 (LR)    |
    tinyThread_stack[threadIDX][TT_EXCEPTION_FRAME_R12] = 0x12345678; // R12         |
    tinyThread_stack[threadIDX][TT_EXCEPTION_FRAME_R3] = 0x22345678;  // R3          ---- Exception frame
    tinyThread_stack[threadIDX][TT_EXCEPTION_FRAME_R2] = 0x32345678;  // R2          |
    tinyThread_stack[threadIDX][TT_EXCEPTION_FRAME_R1] = 0x42345678;  // R1          |
    tinyThread_stack[threadIDX][TT_EXCEPTION_FRAME_R0] = 0x52345678;  // R0 ----------

    // R4-R11 are general purpose registers that are optional to save
    tinyThread_stack[threadIDX][CFG_TINYTHREADS_STACK_SIZE - 9] = 0x62345678;  // R11
    tinyThread_stack[threadIDX][CFG_TINYTHREADS_STACK_SIZE - 10] = 0x72345678; // R10
    tinyThread_stack[threadIDX][CFG_TINYTHREADS_STACK_SIZE - 11] = 0x82345678; // R9
    tinyThread_stack[threadIDX][CFG_TINYTHREADS_STACK_SIZE - 12] = 0x92345678; // R8
    tinyThread_stack[threadIDX][CFG_TINYTHREADS_STACK_SIZE - 13] = 0xa2345678; // R7
    tinyThread_stack[threadIDX][CFG_TINYTHREADS_STACK_SIZE - 14] = 0xb2345678; // R6
    tinyThread_stack[threadIDX][CFG_TINYTHREADS_STACK_SIZE - 15] = 0xc2345678; // R5
    tinyThread_stack[threadIDX][CFG_TINYTHREADS_STACK_SIZE - 16] = 0xd2345678; // R4

    return err;
}

/**************************************************************************
 * Add a thread to the linked list
 * - Add the thread to the ready linked list or non ready linked list
 * - Initialize the stack for the thread
 * - Initialize the PC for the thread to the thread function
 *   this is only valid on initilization, during execution the PC can be
 *   anywhere in the thread function
 * - Increment the tinyThreads_thread_Count
 *
 *  TODO: make a thread type with period and other things , also make thread type
 *  accept a 32bit argument that can be used to pass messages in the form of
 * a pointer to a struct or literal number
 **************************************************************************/
tinyThread_tcb_idx tt_ThreadAdd(void (*thread)(uint32_t), tinyThreadsTime_ms_t period, tinyThreadPriority_t priority,
                                uint8_t *name, bool ready)
{

    TinyThreadsStatus err = TINYTHREADS_OK;
    // var to store id of thread
    tinyThread_tcb_idx id = 0;
    /* disable interrupts */
    tt_CoreCsEnter();
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
        // TODO : this assumes static allocation of tcb
        tinyThread_stack[tinyThreads_thread_Count][TT_EXCEPTION_FRAME_PC] = (uint32_t)thread;
        if (ready)
        {
            tinyThread_ready_thread_add_ll(tinyThreads_thread_Count);
        }
        else
        {
            tinyThread_addThreadToNonReadyList(tinyThreads_thread_Count);
        }

        tt_ThreadStackInit(tinyThreads_thread_Count);
        // initialize PC , initial program counter just points to the thread.
        // However during context switching  we will push the PC (which will vary) to the stack
        // and pop it off when we want to return to the thread at its proper place
        id = tinyThreads_thread_Count;
        tinyThreads_thread_Count++;
    }
    else
    {
        err = TINYTHREADS_MAX_THREADS_REACHED;
    }
    /* enable interrupts */
    tt_CoreCsExit();
    debug(err);
    if (err != TINYTHREADS_OK)
    {
        id = err;
    }
    return id;
}

tinyThreadsTime_ms_t tt_ThreadGetLastRunTime()
{
    return tinyThread_current_tcb->lastRunTime;
}

void tt_ThreadYield(bool updateNext)
{
    // in some cases we will have already update the next tcb
    if (updateNext)
    {
        tt_ThreadUpdateNextThreadPtr();
    }
    FORCE_YEILD_INTERRUPT();
}

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
    return err; // TODO : error check
}

TinyThreadsStatus tt_ThreadSleep(uint32_t time_ms)
{
    TinyThreadsStatus err = TINYTHREADS_OK;
    tt_CoreCsEnter();
    // set state to sleeping
    tinyThread_current_tcb->state |= THREAD_STATE_SLEEPING;
    // set sleep counter
    tinyThread_current_tcb->sleep_count_ms = time_ms;
    tinyThread_addThreadToNonReadyList(tinyThread_current_tcb->id);
    tt_CoreCsExit();
    tt_ThreadYield(true);
    return err;
}

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
    return err;
}

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
    return err;
}

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
    return err;
}

// TODO: do i want to keep this? internal use?
tinyThreadsTime_ms_t tt_ThreadGetSleepCount(tinyThread_tcb_idx id)
{
    // TODO: implement
    if (tt_isValidTcb(id))
    {
        return tinyThread_thread_ctl[id].sleep_count_ms;
    }
    return 0;
}

tinyThreadsTime_ms_t tt_ThreadGetNotifyToCount(tinyThread_tcb_idx id)
{
    // TODO: implement
    if (tt_isValidTcb(id))
    {
        return tinyThread_thread_ctl[id].notify_timeout_count;
    }
    return 0;
}

tinyThread_tcb *tt_ThreadGetCurrentTcb(void)
{
    return tinyThread_current_tcb;
}

tinyThread_tcb *tt_ThreadGetTcbByID(tinyThread_tcb_idx id)
{
    // TOOD this assumes static allocation of tcb
    if (tt_isValidTcb(id))
    {
        return &tinyThread_thread_ctl[id];
    }
    return NULL;
}

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
    return err;
}

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
    if (tempTcb->notifyVal != NULL)
    {
        // read the value
        *val = *(tempTcb->notifyVal);
        tempTcb->notifyConsumed = true;
    }
    tt_CoreCsExit();
    return err;
}

TinyThreadsStatus tt_ThreadNotify(tinyThread_tcb_idx taskID, uint32_t newVal, bool overwrite)
{
    tt_CoreCsEnter();
    TinyThreadsStatus err = TINYTHREADS_OK;
    if (tt_isValidTcb(taskID))
    {

        tinyThread_tcb_t *tempTcb = &tinyThread_thread_ctl[taskID];
        // write the data if overwrite is true or if data has been consumed
        if (overwrite || tempTcb->notifyVal == true)
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
    return err;
}

tinyThread_tcb_idx tt_ThreadGetCurrentID(void)
{
    return tinyThread_current_tcb->id;
}

uint32_t tt_ThreadGetInactiveThreadCount(void)
{
    return tinyThread_inactive_thread_count;
}

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