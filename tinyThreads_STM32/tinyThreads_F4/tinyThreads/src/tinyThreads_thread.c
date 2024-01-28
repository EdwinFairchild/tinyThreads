#include "tinyThreads_thread.h"
#include "tinyThreads_config.h"
#include "tinyThreads_core.h"
#include "tinyThreads_debug.h"
#include "tinyThreads_error.h"
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
tinyThread_tcb *tinyThread_current_tcb = &tinyThread_thread_ctl[0];
// used to keep track of next tcb
static tinyThread_tcb *tinyThread_next_tcb = NULL;
static tinyThread_tcb *tcb_ll_head = NULL;
static tinyThread_tcb *tcb_ll_tail = NULL;

// make a list of suspended threads
static tinyThread_suspended_threads_list_t tinyThread_suspended_threads_list;

// keep track of number of threads, also used as id
static tinyThread_tcb_idx tinyThreads_thread_Count = 0;

static uint32_t tinyThread_inactive_thread_count = 0;
/***************| system tick |**********************/
tinyThreadsTime_ms_t tinyThread_tick = 0;

/***************| function prototypes |**********************/

static bool tt_isValidTcb(tinyThread_tcb_idx id);
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
static TinyThreadsStatus tinyThread_tcb_ll_add(uint32_t period)
{
    TinyThreadsStatus err = TINYTHREADS_OK;

    if (tcb_ll_head == NULL && tcb_ll_tail == NULL)
    {
        // this is first task (system task)
        tcb_ll_tail = &tinyThread_thread_ctl[tinyThreads_thread_Count];
        tcb_ll_head = tcb_ll_tail;
    }
    else
    {
        tcb_ll_tail->next = &tinyThread_thread_ctl[tinyThreads_thread_Count];
        tcb_ll_tail = &tinyThread_thread_ctl[tinyThreads_thread_Count];
        tcb_ll_tail->next = tcb_ll_head;
    }

    return err;
}

/**************************************************************************
 * Add a tcb to the non ready list array
 * This list only contains threads that are not ready to run
 * return : TinyThreadsStatus
 **************************************************************************/
static TinyThreadsStatus tinyThread_non_ready_thread_add_ll(tinyThread_tcb_idx id)
{
    TinyThreadsStatus err = TINYTHREADS_OK;

    // make new node
    tinyThread_suspended_threads_node_t *temp =
        (tinyThread_suspended_threads_node_t *)malloc(sizeof(tinyThread_suspended_threads_node_t));
    temp->tcb = &tinyThread_thread_ctl[id];
    // STATE should be set before calling this function, since task can be
    // non ready for multiple reasons
    // temp->tcb->state |= THREAD_STATE_SLEEPING;
    temp->next = NULL;
    temp->prev = NULL;
    // check if head is null
    if (tinyThread_suspended_threads_list.head == NULL)
    {
        tinyThread_suspended_threads_list.head = temp;
        tinyThread_suspended_threads_list.tail = tinyThread_suspended_threads_list.head;
    }
    else
    {
        tinyThread_suspended_threads_list.tail->next = temp;
        temp->prev = tinyThread_suspended_threads_list.tail;
        tinyThread_suspended_threads_list.tail = temp;
    }
    tinyThread_inactive_thread_count++;
    return err;
}
/**************************************************************************
 * Remove a tcb from the non ready list
 * This list only contains threads that are not ready to run
 * return : TinyThreadsStatus
 **************************************************************************/
static TinyThreadsStatus tinyThread_non_ready_thread_remove_ll(tinyThread_tcb_idx id)
{
    TinyThreadsStatus err = TINYTHREADS_ERROR;
    // make new node
    tinyThread_suspended_threads_node_t *temp =
        (tinyThread_suspended_threads_node_t *)malloc(sizeof(tinyThread_suspended_threads_node_t));
    if (temp != NULL && tinyThread_inactive_thread_count > 0)
    {

        // check if this node is head and then remove and free
        if (tinyThread_suspended_threads_list.head->tcb->id == id)
        {
            // save the head pointer so we can free it
            temp = tinyThread_suspended_threads_list.head;
            // new head is the next node
            tinyThread_suspended_threads_list.head = tinyThread_suspended_threads_list.head->next;
            // free the old head
            free(temp);
            err = TINYTHREADS_OK;
        }
        else if (tinyThread_suspended_threads_list.tail->tcb->id == id)
        {
            // save the tail pointer
            temp = tinyThread_suspended_threads_list.tail;
            // new tail is the prev node
            tinyThread_suspended_threads_list.tail = tinyThread_suspended_threads_list.tail->prev;
            // free the old tail
            free(temp);
            err = TINYTHREADS_OK;
        }
        else
        {
            // find the node
            temp = tinyThread_suspended_threads_list.head;
            while (temp->tcb->id != id)
            {
                temp = temp->next;
            }
            // remove the node
            temp->prev->next = temp->next;
            temp->next->prev = temp->prev;
            free(temp);
            err = TINYTHREADS_OK;
        }
        tinyThread_inactive_thread_count--;
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
 * - Add the thread to the linked list
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
tinyThread_tcb_idx tt_ThreadAdd(void (*thread)(void), tinyThreadsTime_ms_t period, tinyThreadPriority_t priority)
{

    TinyThreadsStatus err = TINYTHREADS_OK;
    // var to store id of thread
    tinyThread_tcb_idx id = 0;
    /* disable interrupts */
    __disable_irq();
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

        tinyThread_tcb_ll_add(period);
        tt_ThreadStackInit(tinyThreads_thread_Count);
        // initialize PC , initial program counter just points to the thread.
        // However during context switching  we will push the PC (which will vary) to the stack
        // and pop it off when we want to return to the thread at its proper place
        tinyThread_stack[tinyThreads_thread_Count][TT_EXCEPTION_FRAME_PC] = (uint32_t)thread;
        id = tinyThreads_thread_Count;
        tinyThreads_thread_Count++;
    }
    else
    {
        err = TINYTHREADS_MAX_THREADS_REACHED;
    }
    /* enable interrupts */
    __enable_irq();
    debug(err);
    if (err != TINYTHREADS_OK)
    {
        id = err;
    }
    return id;
}

/* enter exception
    this will push the exception frame onto the stack
    and set the stack pointer to the top of the stack, knowing this
    information we save the stack pointer for the current thread
    and load the stack pointer for the next thread
*/
__attribute__((naked)) void PendSV_Handler(void)
{
    port_dbg_signal_1_assert();
    // disble interrupts
    __disable_irq();
    // clear pendsv interrupt
    SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;
    // this is defined to be an inline function so we are not altering the stack
    NVIC_EnableIRQ(PendSV_IRQn);

    /************************ Suspend and save current thread ************************/
    // load address of current tinyThread_tcb into r0
    __asm("LDR r0, =tinyThread_current_tcb");
    /*  derefrence the address at r0 (get the value at that address)
        the address found at the first element of the tinyThread_tcb struct is the stack pointer
        now R1 will contain the address of the stack pointer for the current tinyThread_tcb
    */
    __asm("LDR r1, [r0]");
    /*  save the current stack pointer register
        into the address pointed to by r1 which is the stack pointer for the current tinyThread_tcb
    */

    // save r4-r11 on the stack
    __asm("PUSH {r4-r11}");

    __asm("STR sp, [r1]");

    /************************ Restore next thread ************************
     * tt_CoreSystemTickHandler function will determine which thread comes next
     * and set tinyThread_next_tcb */
    __asm("LDR r1, =tinyThread_next_tcb");
    /*  derefrence the address at r1 (get the value at that address)
        the address found at the first element of the tinyThread_tcb struct is the stack pointer
        now R1 will contain the address of the stack pointer for the next tinyThread_tcb
    */
    __asm("LDR r1, [r1]");
    /*  Since R1 now points to the next tinyThread_tcb and the first element of any tinyThread_tcb struct
        is the stack pointer for that given taks stack, now load the value at the address pointed to by r1
        into the stack pointer register
    */
    __asm("LDR sp, [r1]");
    /*  R1 still containts next threads address, so lets update tinyThread_current_tcb whos address
        we should still have in R0 from above */
    __asm("STR r1, [r0]");

    /* tinyThread_current_tcb has a timestamp 5 words from the start lets update that to tick value*/
    __asm("LDR r2, =tinyThread_tick"); // load current tick time
    __asm("LDR r2, [r2]");
    __asm__("STR r2, [r1, %0]" ::"I"(TINYTASK_TCB_LAST_RUNTIME_OFFSET)); // update last run time in tcb

    /*  restore r4-r11 from the stack, since we just updated the stack pointer to point to the next threads stack
        it will pop the values from there.
    */
    __asm("POP {r4-r11}");

    // enable interrupts
    __enable_irq();

    /*  return from interrupt
        this will pop the exception frame from the stack and return to the next thread
    */

    // port_dbg_signal_1_deassert();
    port_dbg_signal_1_deassert();

    __asm("BX LR");
}

tinyThreadsTime_ms_t tt_ThreadGetLastRunTime()
{
    return tinyThread_current_tcb->lastRunTime;
}

void tt_ThreadYield(void)
{
    // when yeilding we always want to go to the next thread
    tt_ThreadUpdateNextThreadPtr();
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

void tt_ThreadUpdateNextThreadPtr(void)
{
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
                tinyThread_non_ready_thread_remove_ll(temp->tcb->id);
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
                tinyThread_non_ready_thread_remove_ll(temp->tcb->id);
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
    tinyThread_non_ready_thread_add_ll(tinyThread_current_tcb->id);
    tt_CoreCsExit();
    tt_ThreadYield();
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
        err = tinyThread_non_ready_thread_remove_ll(id);
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

static bool tt_isValidTcb(tinyThread_tcb_idx id)
{
    bool valid = false;
    if (id < tinyThreads_thread_Count)
    {
        valid = true;
    }
    return valid;
}

TinyThreadsStatus tt_ThreadNotifyWait(tinyThreadsTime_ms_t timeout, uint32_t *val)
{
    tt_CoreCsEnter();
    TinyThreadsStatus err = TINYTHREADS_OK;
    tinyThread_tcb_t *tempTcb = tinyThread_current_tcb;

    // if there is a timeout and there is no data then put the thread in non ready list
    if (timeout && tempTcb->notifyVal == NULL)
    {
        // set notify value to point to the value passed in
        tempTcb->notifyVal = val;
        // set state to pending notify
        tempTcb->state |= THREAD_STATE_PENDING_NOTIFY;
        // set timeout counter
        tempTcb->notify_timeout_count = timeout;
        // add to non ready list
        tinyThread_non_ready_thread_add_ll(tempTcb->id);
        // yield
        tt_CoreCsExit();
        tt_ThreadYield();
        // thread will return here after timeout or notification
    }

    // We get to this point if there is a notification or we timedout
    // or if timeout is 0 or if there is a notification
    // TODO : we should check if we timedout or not could be useful
    if (tempTcb->notifyVal != NULL)
    {
        // read the value
        *val = *(tempTcb->notifyVal);
        // set value to null
        tempTcb->notifyVal = NULL;
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
        // write the data if overwrite is true or if there is no data
        if (overwrite || tempTcb->notifyVal == NULL)
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
                    tinyThread_non_ready_thread_remove_ll(tempTcb->id);
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