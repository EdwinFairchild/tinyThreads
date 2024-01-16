#include "tinyKernel.h"
#include "tinyTasksError.h"
#include "tinyTasksConfig.h"

// TODO: remove Debugging-----------


//---------------------

/* Reserve stack space */
uint32_t tinyTask_stack[TINYTASKS_MAX_TASKS][TINYTASKS_STACK_SIZE];

/* Task Control Block */
typedef struct tinyTask_tcb  tinyTask_tcb_t;

typedef struct tinyTask_tcb{
    uint32_t *stackPointer; // Pointer to the current stack pointer
    tinyTask_tcb_t *next;   // Pointer to the next task
    uint32_t period;        // Period of the task
    
}tinyTask_tcb;

/* Static allocation of Task Control Block Array*/
tinyTask_tcb_t tinyTask_task_ctl[TINYTASKS_MAX_TASKS];
tinyTask_tcb *tinyTask_current_tcb = NULL;
typedef uint32_t tinyTask_tcb_idx;
tinyTask_tcb_idx tinyTasks_task_Count = 0;


/***************| system tick |**********************/
uint32_t tinyTask_tick = 0;

static void systemTask(void);

/**************************************************************************
* Task control block linked list functions
***************************************************************************/

/**************************************************************************
* Attempt to add a new task to the linked list
***************************************************************************/
static TinyTasksStatus tinyTask_tcb_ll_add(uint32_t period)
{
    TinyTasksStatus err = TINYTASKS_OK;
    // Add to account for system task
    if(tinyTasks_task_Count >= TINYTASKS_MAX_TASKS){
        err = TINYTASKS_MAX_TASKS_REACHED;
    }
    tinyTask_task_ctl[tinyTasks_task_Count].period = period;
    tinyTask_task_ctl[tinyTasks_task_Count].next = NULL;
    //if list is not empty simpy add the task to next pointer of previous task
    if(tinyTasks_task_Count !=0){
        tinyTask_task_ctl[tinyTasks_task_Count - 1].next = &tinyTask_task_ctl[tinyTasks_task_Count];        
    }
    //if this is the last taks point the next pointer to the first task
    if(tinyTasks_task_Count == (TINYTASKS_MAX_TASKS - 1)){
        tinyTask_task_ctl[tinyTasks_task_Count].next = &tinyTask_task_ctl[0];
    }

    return err;
}

/**************************************************************************
 * Initialize the kernel:
 *  - Initialize the linked list of tasks
 *  - There should be atleast 1 task running at all times
 **************************************************************************/
TinyTasksStatus tinyKernel_init(void)
{
    TinyTasksStatus err = TINYTASKS_OK;
    // add system related tasks
    err = tinyKernel_addTask(systemTask, 10);
    // os cannot function without system task
    if(err != TINYTASKS_OK){
        return err;
    }

    // initialize current task control block
    tinyTask_current_tcb = &tinyTask_task_ctl[0];

    tinyTask_port_enable_tick_timer();
    tinyTasks_enable_context_switching_isr();

    return err;
}

/**************************************************************************
* - Initializes the stack for a task to mostly dummy values
*  -set T-bit to 1 to make sure we run in thumb mode : see core_cm4.h > xPSR_Type
* stacks are in decending order 
* this is why we set the stack pointer to the last element of the stack 
***************************************************************************/
TinyTasksStatus tinyKernel_task_stack_init(uint32_t taskIDX){
    TinyTasksStatus err = TINYTASKS_OK;
    
    /*  initialize the stack pointer
        R13 is stack pointer, we manages the register manually using the stack pointer blow */
    tinyTask_task_ctl[taskIDX].stackPointer = &tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 16];
    
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 1] |= (1 << 24); // xPSR --------
    /*  The PC get initialized in tinyKernel_addTask                               |
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 2] = 0x12345678; // PC */ //    |
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 3] = 0xe2345678; // R14 (LR)    |
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 4] = 0x12345678; // R12         |
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 5] = 0x22345678; // R3          ---- Exception frame
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 6] = 0x32345678; // R2          |
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 7] = 0x42345678; // R1          |
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 8] = 0x52345678; // R0 ----------

    // R4-R11 are general purpose registers that are optional to save
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE -  9] = 0x62345678; // R11           
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 10] = 0x72345678; // R10
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 11] = 0x82345678; // R9
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 12] = 0x92345678; // R8
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 13] = 0xa2345678; // R7
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 14] = 0xb2345678; // R6
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 15] = 0xc2345678; // R5
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 16] = 0xd2345678; // R4
    
    return err;
}

/**************************************************************************
 * Run the kernel
 * - Load the stack pointer for the current task
 * - Pop the exception frame from the stack and return to the task
 **************************************************************************/
TinyTasksStatus tinyKernel_run(void){
    TinyTasksStatus err = TINYTASKS_OK;
    // disable interrupts
    __disable_irq();
    /*  Load the address of tinyTask_current_tcb into R0 */
    __asm("LDR r0, =tinyTask_current_tcb");
    /*  Dereference the address at R0 
        the first element of the tinyTask_tcb struct is the tak's stack pointer
        Now R1 will contain the address of the stack pointer for the current tinyTask_tcb
    */
    __asm("LDR r1, [r0]");
    /*  Load the value at the address pointed to by R1 (stak's stack pointer)
        into the stack pointer register */
    __asm("LDR sp, [r1]");
    /*  Pop the exception frame from the stack and return to the task */
    __asm("POP {r4-r11}");
   
    __asm("POP {r12}");
    __asm("POP {r0-r3}");
    /*  skip LR in our stack by adding 4 the current SP */
    __asm("ADD sp, sp, 4");
    /*  Now we are at the PC in our stack, pop that into the LR 
        this will cause the processor to jump to the task function.
        The PC in our task was initialized in tinyKernel_addTask2
     */
    __asm("POP {lr}");
    /*  skip psr by adding 4 the current SP */
    __asm("ADD sp, sp, #4");

    /* enable interrupts */
    __enable_irq();

    /*  return from exception 
        this will jump to the value in the LR register
        which is the task function
    */
    __asm("BX LR");

    return err;
}
/**************************************************************************
 * Add a task to the linked list
 * - Add the task to the linked list
 * - Initialize the stack for the task
 * - Initialize the PC for the task to the task function
 *   this is only valid on initilization, during execution the PC can be
 *   anywhere in the task function
 * - Increment the tinyTasks_task_Count
 * 
 *  TODO: make a task type with period and other things , also make task type 
 *  accept a 32bit argument that can be used to pass messages in the form of
 * a pointer to a struct or literal number
 **************************************************************************/
TinyTasksStatus tinyKernel_addTask(void (*task)(void), uint32_t period){
    TinyTasksStatus err = TINYTASKS_OK;
    /* disable interrupts */
    __disable_irq();
    if(tinyTasks_task_Count <= (TINYTASKS_MAX_TASKS -1) && task != NULL){
        tinyTask_tcb_ll_add(period);
        tinyKernel_task_stack_init(tinyTasks_task_Count);
        // initialize PC , initial program counter just points to the task
        // during context switch we will push the PC to the stack
        // and pop it off when we want to return to the task at its proper place
        tinyTask_stack[tinyTasks_task_Count][TINYTASKS_STACK_SIZE - 2] = (uint32_t)task;
        tinyTasks_task_Count++;
    }
    else{
        err = TINYTASKS_MAX_TASKS_REACHED;
        
    }
    /* enable interrupts */
    __enable_irq();
    debug(err);
    return err;
}

void tinyTask_isr_task_switch(uint32_t tick)
{
    // TODO : get period from current task control block
    if(tick > 1000)
    {
        // only switch if current tasks time has expired
        //printf("Task Switch\r\n");
        tinyTask_tick_reset();
        // generate pendsv interrupt
        SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;

    }
}


    /* enter exception
        this will push the exception frame onto the stack
        and set the stack pointer to the top of the stack, knowing this
        information we save the stack pointer for the current task
        and load the stack pointer for the next task
    */    
__attribute__((naked)) void PendSV_Handler(void)
{
    
    // disble interrupts
    __disable_irq();
    //clear pendsv interrupt
    SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;
    NVIC_EnableIRQ(PendSV_IRQn);

    /************************ Suspend and save current task ************************/
    //save r4-r11 on the stack
    __asm("PUSH {r4-r11}");
    // load address of current tinyTask_tcb into r0
    __asm("LDR r0, =tinyTask_current_tcb");
    /*  derefrence the address at r0 (get the value at that address)
        the address found at the first element of the tinyTask_tcb struct is the stack pointer
        now R1 will contain the address of the stack pointer for the current tinyTask_tcb
    */
    __asm("LDR r1, [r0]");
    /*  save the current stack pointer register  
        into the address pointed to by r1 which is the stack pointer for the current tinyTask_tcb
    */
    __asm("STR sp, [r1]");

    /************************ Restore next task ************************/

    /*  Since R1 current holds the address of the stack pointer of the current tinyTask_tcb
        then 4 bytes above that address is the next pointer, add 4 to r1 and reload it into r1*/
    __asm("LDR r1, [r1, #4]");
    /*  Since R1 now points to the next tinyTask_tcb and the first element of any tinyTask_tcb struct
        is the stack pointer for that given taks stack, now load the value at the address pointed to by r1 
        into the stack pointer register
    */
    __asm("LDR sp, [r1]");
    /*  R1 still containts next tasks address, so lets update tinyTask_current_tcb whos address 
        we should still have in R0 from above */
    __asm("STR r1, [r0]");

    /*  restore r4-r11 from the stack, since we just updated the stack pointer to point to the next tasks stack
        it will pop the values from there.
    */
    __asm("POP {r4-r11}");

    // enable interrupts
    __enable_irq();

    /*  return from interrupt
        this will pop the exception frame from the stack and return to the next task
    */
    __asm("BX LR");

}

static void systemTask(void){
    while(1){
    printf("SysTask\r\n");
    HAL_Delay(1500);
    }
}