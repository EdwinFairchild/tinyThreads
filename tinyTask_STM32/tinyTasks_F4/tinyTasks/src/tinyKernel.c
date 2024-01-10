#include "tinyKernel.h"
#include "tinyTasksError.h"
#include "tinyTasksConfig.h"

// TODO: remove Debugging-----------
extern void printMsg(char *msg, ...);
//---------------------


TinyTasksStatus tinyKernel_init(void){
    TinyTasksStatus err = TINYTASKS_OK;
    printMsg("Starting tinyTasks Kernel\r\n");
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
