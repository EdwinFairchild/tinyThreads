#include "tinyThreads_system.h"

static uint32_t cs_nesting = 0;

void tinyThreads_sys_CsEnter(void){
    //only need to do this once
    if(cs_nesting == 0){
        __disable_irq();
    }
    cs_nesting++;

}
void tinyThreads_sys_CsExit(void){
    cs_nesting--;
    if(cs_nesting == 0){
        __enable_irq();
    }
}