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
    
}tinyTask_tcb;

tinyTask_tcb_t taskControlBlocks[TINYTASKS_MAX_TASKS];



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

    tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 1] |= (1 << 24);

    // init stack for given task
    taskControlBlocks[taskIDX].stackPointer = &tinyTask_stack[taskIDX][TINYTASKS_STACK_SIZE - 1];
    
    
    return err;
}

TinyTasksStatus tinyKernel_run(void){
    TinyTasksStatus err = TINYTASKS_OK;
    
    return err;
}

TinyTasksStatus tinyKetnell_addTask(void (*task)(void), uint32_t period){
    TinyTasksStatus err = TINYTASKS_OK;
    
    return err;
}

__attribute__((naked)) TinyTasksScheduler(void)
{
    TinyTasksStatus err = TINYTASKS_OK;

    return err;
}
