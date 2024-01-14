#include "tinyKernel.h"
#include "tinyTasksError.h"
#include "tinyTasksConfig.h"

// TODO: remove Debugging-----------
extern void printMsg(char *msg, ...);
void (*funct_ptr)(void);
extern void HAL_Delay(uint32_t Delay);
void task1(void){
    while(1){
    printMsg("Task 1\r\n");
    HAL_Delay(500);
    }
}
void task2(void){
    while(1){
    printMsg("Task 2\r\n");
    HAL_Delay(1500);
    }
}
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

/**************************************************************************
* Task control block linked list functions
***************************************************************************/

/**************************************************************************
* Attempt to add a new task to the linked list
***************************************************************************/
static TinyTasksStatus tinyTask_tcb_ll_add(uint32_t period)
{
    TinyTasksStatus err = TINYTASKS_OK;
    if(tinyTasks_task_Count >= TINYTASKS_MAX_TASKS){
        err = TINYTASKS_MAX_TASKS_REACHED;
    }
    tinyTask_task_ctl[tinyTasks_task_Count].period = period;
    tinyTask_task_ctl[tinyTasks_task_Count].next = NULL;
    //if list is not empty simpy add the task to next pointer of previous task
    if(tinyTasks_task_Count !=0){
        tinyTask_task_ctl[tinyTasks_task_Count - 1].next = &tinyTask_task_ctl[tinyTasks_task_Count];        
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
    printMsg("Starting tinyTasks Kernel\r\n");
    tinyKernel_addTask(task1,530);
    tinyKernel_addTask(task2,420);

    // initialize current task control block
    tinyTask_current_tcb = &tinyTask_task_ctl[0];

    // TODO : use timer periph instead of systick
    //enable systick interrupt
    NVIC_SetPriority(SysTick_IRQn, 15);
    NVIC_EnableIRQ(SysTick_IRQn);
    // start systick timer with 1 ms period
    SysTick_Config(SystemCoreClock / 1000);

    //enable pendsv interrupt used for task switching
    NVIC_SetPriority(PendSV_IRQn, 15);
    NVIC_EnableIRQ(PendSV_IRQn);




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
    // TODO : make macro for top of stack is 512-1
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 1] |= (1 << 24); // xPSR

    // stack pointer will get initialized elsewhere 
    // tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 2] = (uint32_t)taskControlBlocks[taskIDX].stackPointer;
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 3] = 0x12345678; // R14 (LR)
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 4] = 0x12345678; // R12 
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 5] = 0x12345678; // R3 
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 6] = 0x12345678; // R2
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 7] = 0x12345678; // R1
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 8] = 0x12345678; // R0

    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 9] = 0x12345678; // R11
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 10] = 0x12345678; // R10
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 11] = 0x12345678; // R9
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 12] = 0x12345678; // R8
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 13] = 0x12345678; // R7
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 14] = 0x12345678; // R6
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 15] = 0x12345678; // R5
    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 16] = 0x12345678; // R4
    
    return err;
}

TinyTasksStatus tinyKernel_run(void){
    TinyTasksStatus err = TINYTASKS_OK;
    
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
    return err;
}

void tinyTask_isr_task_switch(uint32_t tick)
{
    // TODO : get period from current task control block
    if(tick > 1000)
    {
        // only switch if current tasks time has expired
        tinyTask_printMsg("Task Switch\r\n");
        tinyTask_tick_reset();
        // generate pendsv interrupt
        SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;

    }
}


__attribute__((naked)) void PendSV_Handler(void)
{
    // disble interrupts
    __disable_irq();
    //clear pendsv interrupt
    SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;

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

}