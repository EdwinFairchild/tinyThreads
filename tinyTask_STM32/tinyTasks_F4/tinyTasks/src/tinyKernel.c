#include "tinyKernel.h"
#include "tinyTasksError.h"
#include "tinyTasksConfig.h"

// TODO: remove Debugging-----------
extern void printMsg(char *msg, ...);
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

/* Task Control Block Array */
tinyTask_tcb_t tinyTask_task_ctl[TINYTASKS_MAX_TASKS];
typedef uint32_t tinyTask_tcb_idx;
tinyTask_tcb_idx currentInitTask = 0;


TinyTasksStatus tinyKernel_init(void)
{
    TinyTasksStatus err = TINYTASKS_OK;
     printMsg("Starting tinyTasks Kernel\r\n");
    return err;
}

/**************************************************************************
* set T-bit to 1 to make sure we run in thumb mode : see core_cm4.h > xPSR_Type
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

TinyTasksStatus tinyKetnell_addTask(void (*task)(void), uint32_t period){
    TinyTasksStatus err = TINYTASKS_OK;
    /* disable interrupts */
    __disable_irq();
    if(currentInitTask <= (TINYTASKS_MAX_TASKS -1)){
        // todo make this into a proper linked list with linked list functions for adding tasks
        tinyTask_task_ctl[currentInitTask].next = &tinyTask_task_ctl[currentInitTask + 1];
        //since this is a round robin scheduler this task will run for the period and then the next task will run
        tinyTask_task_ctl[currentInitTask].period = period;
        tinyKernel_task_stack_init(currentInitTask);
        // initialize PC , initial program counter just points to the task
        // during context switch we will push the PC to the stack
        // and pop it off when we want to return to the task at its proper place
        tinyTask_stack[currentInitTask][TINYTASKS_STACK_SIZE - 2] = (uint32_t)task;


        currentInitTask++;
    }
    else{
        err = TINYTASKS_MAX_TASKS_REACHED;
    }
    return err;
}

__attribute__((naked)) TinyTasksScheduler(void)
{
    TinyTasksStatus err = TINYTASKS_OK;

    return err;
}
