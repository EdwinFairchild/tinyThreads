#include "tinyKernel.h"
#include "tinyThreads_error.h"
#include "tinyThreads_config.h"
#include "tinyThreads_debug.h"
// TODO: remove Debugging-----------


//---------------------

/* Reserve stack space, include user thread space and system thread space */
// TODO : shoould I make a seprate system stack space ? If not , user musct know system will use some of
// their stack , so do not calcualte stack on their threads alone. 
uint32_t tinyThread_stack[TT_MAX_THREADS][TT_TOTAL_STACK_SIZE];

/* Thread Control Block */
typedef struct tinyThread_tcb  tinyThread_tcb_t;

typedef struct tinyThread_tcb{
    uint32_t *stackPointer; // Pointer to the current stack pointer
    tinyThread_tcb_t *next;   // Pointer to the next thread
    tinyThread_tcb_t *prev;   // Pointer to the previous thread
    tinyThreadPeriod_t period_ms;        // Period of the thread
    tinyThreadsTime_ms_t lastRunTime;       // Last time the thread ran
    tinyThreadPriority_t priority;      // Priority of the thread
    tinyThreadsState_t state;         // State of the thread

}tinyThread_tcb;

/* Static allocation of Thread Control Block Array*/
static tinyThread_tcb_t tinyThread_thread_ctl[TT_MAX_THREADS];
static tinyThread_tcb *tinyThread_current_tcb = NULL;
static tinyThread_tcb *tcb_ll_head = NULL;
static tinyThread_tcb *tcb_ll_tail = NULL;
typedef uint32_t tinyThread_tcb_idx;
static tinyThread_tcb_idx tinyThreads_thread_Count = 0;


/***************| system tick |**********************/
tinyThreadsTime_ms_t tinyThread_tick = 0;

/***************| function prototypes |**********************/
static void systemThread(void);

/**************************************************************************
* Thread control block linked list functions
***************************************************************************/

/**************************************************************************
* Attempt to add a new thread to the linked list
Can only get here through tinyKernel_addThread which veryfies tinyThread_canAddThread
***************************************************************************/
static TinyThreadsStatus tinyThread_tcb_ll_add(uint32_t period)
{
    TinyThreadsStatus err = TINYTHREADS_OK;
    // Add to account for system thread
    tinyThread_thread_ctl[tinyThreads_thread_Count].period_ms = period;
    tinyThread_thread_ctl[tinyThreads_thread_Count].priority = 0; // TODO :
    tinyThread_thread_ctl[tinyThreads_thread_Count].state = THREAD_STATE_READY;
    tinyThread_thread_ctl[tinyThreads_thread_Count].prev = NULL;
    tinyThread_thread_ctl[tinyThreads_thread_Count].lastRunTime = (tinyThreadsTime_ms_t)0;
    if(tcb_ll_head == NULL && tcb_ll_tail == NULL)
    {
        // this is first task (system task)
        tcb_ll_tail = &tinyThread_thread_ctl[tinyThreads_thread_Count]; 
        tcb_ll_head = tcb_ll_tail;
    }else{
        tcb_ll_tail->next = &tinyThread_thread_ctl[tinyThreads_thread_Count]; 
        tcb_ll_tail = &tinyThread_thread_ctl[tinyThreads_thread_Count]; 
        tcb_ll_tail->next = tcb_ll_head;
    }

    return err;
}

/**************************************************************************
 * Check if max num of threads have been added:
 * return : TinyThreadsStatus  
 **************************************************************************/
static TinyThreadsStatus tinyThread_canAddThread(void){
    TinyThreadsStatus err = TINYTHREADS_OK;
    if( ! (tinyThreads_thread_Count <= (TT_MAX_THREADS)) )
    {
        err = TINYTHREADS_MAX_THREADS_REACHED;
    }
    return err;
}

/**************************************************************************
 * Initialize the kernel:
 *  - Initialize the linked list of threads
 *  - There should be atleast 1 thread running at all times
 **************************************************************************/
TinyThreadsStatus tinyKernel_init(void)
{
    TinyThreadsStatus err = TINYTHREADS_OK;
    // add system related threads
    err = tinyKernel_addThread(systemThread, 10);
    // os cannot function without system thread
    if(err != TINYTHREADS_OK){
        return err;
    }

    // initialize current thread control block
    tinyThread_current_tcb = &tinyThread_thread_ctl[0];

    tinyThread_port_enable_tick_timer();
    tinyThreads_enable_context_switching_isr();

    return err;
}

/**************************************************************************
* - Initializes the stack for a thread to mostly dummy values
*  -set T-bit to 1 to make sure we run in thumb mode : see core_cm4.h > xPSR_Type
* stacks are in decending order 
* this is why we set the stack pointer to the last element of the stack 
***************************************************************************/
TinyThreadsStatus tinyKernel_thread_stack_init(uint32_t threadIDX){
    TinyThreadsStatus err = TINYTHREADS_OK;
    
    /*  initialize the stack pointer
        R13 is stack pointer, we manages the register manually using the stack pointer blow */
    tinyThread_thread_ctl[threadIDX].stackPointer = &tinyThread_stack[threadIDX][CFG_TINYTHREADS_STACK_SIZE - 16];
    tinyThread_thread_ctl[threadIDX].lastRunTime = tinyThread_tick_get();
    
    tinyThread_stack[threadIDX][TT_EXCEPTION_FRAME_PSR] |= (1 << 24); // xPSR --------
    /*  The PC get initialized in tinyKernel_addThread                               |
    tinyThread_stack[threadIDX][CFG_TINYTHREADS_STACK_SIZE - 2] = 0x12345678; // PC */ //    |
    tinyThread_stack[threadIDX][TT_EXCEPTION_FRAME_LR] = 0xe2345678; // R14 (LR)    |
    tinyThread_stack[threadIDX][TT_EXCEPTION_FRAME_R12] = 0x12345678; // R12         |
    tinyThread_stack[threadIDX][TT_EXCEPTION_FRAME_R3] = 0x22345678; // R3          ---- Exception frame
    tinyThread_stack[threadIDX][TT_EXCEPTION_FRAME_R2] = 0x32345678; // R2          |
    tinyThread_stack[threadIDX][TT_EXCEPTION_FRAME_R1] = 0x42345678; // R1          |
    tinyThread_stack[threadIDX][TT_EXCEPTION_FRAME_R0] = 0x52345678; // R0 ----------

    // R4-R11 are general purpose registers that are optional to save
    tinyThread_stack[threadIDX][CFG_TINYTHREADS_STACK_SIZE -  9] = 0x62345678; // R11           
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
 * Run the kernel
 * - Load the stack pointer for the current thread
 * - Pop the exception frame from the stack and return to the thread
 **************************************************************************/
TinyThreadsStatus tinyKernel_run(void){
    TinyThreadsStatus err = TINYTHREADS_OK;
    // disable interrupts
    __disable_irq();
    /*  Load the address of tinyThread_current_tcb into R0 */
    __asm("LDR r0, =tinyThread_current_tcb");
    /*  Dereference the address at R0 
        the first element of the tinyThread_tcb struct is the tak's stack pointer
        Now R1 will contain the address of the stack pointer for the current tinyThread_tcb
    */
    __asm("LDR r1, [r0]");
    /*  Load the value at the address pointed to by R1 (stak's stack pointer)
        into the stack pointer register */
    __asm("LDR sp, [r1]");
    /*  Pop the exception frame from the stack and return to the thread */
    __asm("POP {r4-r11}");
   
    __asm("POP {r12}");
    __asm("POP {r0-r3}");
    /*  skip LR in our stack by adding 4 the current SP */
    __asm("ADD sp, sp, 4");
    /*  Now we are at the PC in our stack, pop that into the LR 
        this will cause the processor to jump to the thread function.
        The PC in our thread was initialized in tinyKernel_addThread2
     */
    __asm("POP {lr}");
    /*  skip psr by adding 4 the current SP */
    __asm("ADD sp, sp, #4");

    /* enable interrupts */
    __enable_irq();

    /*  return from exception 
        this will jump to the value in the LR register
        which is the thread function
    */
    __asm("BX LR");

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
TinyThreadsStatus tinyKernel_addThread(void (*thread)(void), uint32_t period){
    TinyThreadsStatus err = TINYTHREADS_OK;
    /* disable interrupts */
    __disable_irq();
    if( tinyThread_canAddThread() == TINYTHREADS_OK && thread != NULL){
        tinyThread_tcb_ll_add(period);
        tinyKernel_thread_stack_init(tinyThreads_thread_Count);
        // initialize PC , initial program counter just points to the thread
        // during context switch we will push the PC to the stack
        // and pop it off when we want to return to the thread at its proper place
        tinyThread_stack[tinyThreads_thread_Count][TT_EXCEPTION_FRAME_PC] = (uint32_t)thread;
        tinyThreads_thread_Count++;
    }
    else{
        err = TINYTHREADS_MAX_THREADS_REACHED;
        
    }
    /* enable interrupts */
    __enable_irq();
    debug(err);
    return err;
}

// TODO: I should have a scheduler file and this will go in there scheduler_round_robin.c 
void tinyThread_isr_system_thread(void)
{
    tinyThread_tick_inc();
    // check the thread control block to see if its time to switch it out (Round Robin)
    if(tinyThread_current_tcb->period_ms <= (tinyThread_tick_get() - tinyThread_current_tcb->lastRunTime))
    {
        // generate pendsv interrupt
        SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
    }
  
}


    /* enter exception
        this will push the exception frame onto the stack
        and set the stack pointer to the top of the stack, knowing this
        information we save the stack pointer for the current thread
        and load the stack pointer for the next thread
    */    
__attribute__((naked)) void PendSV_Handler(void)
{
    
    // disble interrupts
    __disable_irq();
    //clear pendsv interrupt
    SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;
    // this is defined to be an inline function so we are not altering the stack
    NVIC_EnableIRQ(PendSV_IRQn); 

    /************************ Suspend and save current thread ************************/
    //save r4-r11 on the stack
    __asm("PUSH {r4-r11}");
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
    __asm("STR sp, [r1]");

    /************************ Restore next thread ************************/

    /*  Since R1 current holds the address of the stack pointer of the current tinyThread_tcb
        then 4 bytes above that address is the next pointer, add 4 to r1 and reload it into r1*/
    __asm("LDR r1, [r1, %0]"::"I" (TINYTASK_TCB_NEXT_PTR_OFFSET));
    /*  Since R1 now points to the next tinyThread_tcb and the first element of any tinyThread_tcb struct
        is the stack pointer for that given taks stack, now load the value at the address pointed to by r1 
        into the stack pointer register
    */
    __asm("LDR sp, [r1]");
    /*  R1 still containts next threads address, so lets update tinyThread_current_tcb whos address 
        we should still have in R0 from above */
    __asm("STR r1, [r0]");

    /* tinyThread_current_tcb has a timestamp 5 words from the start lets update that to tick value*/
    __asm("LDR r2, =tinyThread_tick");
    __asm("LDR r2, [r2]");
   // __asm("STR r2, [r1, #16]");
    __asm__("STR r2, [r1, %0]"::"I" (TINYTASK_TCB_LAST_RUNTIME_OFFSET));
   


    /*  restore r4-r11 from the stack, since we just updated the stack pointer to point to the next threads stack
        it will pop the values from there.
    */
    __asm("POP {r4-r11}");

    // enable interrupts
    __enable_irq();

    /*  return from interrupt
        this will pop the exception frame from the stack and return to the next thread
    */
    __asm("BX LR");

}

static void systemThread(void ){
    while(1){
        printf(">>>>>>>>> System thread\r\n");
    }
}

tinyThreadsTime_ms_t tinyKernel_getThreadLastRunTime(){
    return tinyThread_current_tcb->lastRunTime;
}
 
TinyThreadsStatus thread_yeild(void){
    // for context switch
    // generate pendsv interrupt
    // TODO : this should call something in tinyThreads_port.c
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;


}