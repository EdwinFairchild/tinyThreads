#include "tinyTasks_system.h"

static uint32_t cs_nesting = 0;

void tinyTasks_sys_CsEnter(void){
    //only need to do this once
    if(cs_nesting == 0){
        __disable_irq();
    }
    cs_nesting++;

}
void tinyTasks_sys_CsExit(void){
    cs_nesting--;
    if(cs_nesting == 0){
        __enable_irq();
    }
}