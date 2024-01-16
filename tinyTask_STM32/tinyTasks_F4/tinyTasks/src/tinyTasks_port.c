#include "tinyKernel.h"
#include "tinyTasksPort.h"


void tinyTask_port_enable_tick_timer(void){
    // TODO : use timer periph instead of systick
    //enable systick interrupt
    NVIC_SetPriority(SysTick_IRQn, 15);
    NVIC_EnableIRQ(SysTick_IRQn);
    // start systick timer with 1 ms period
    SysTick_Config(SystemCoreClock / 1000);
}

void tinyTasks_enable_context_switching_isr(void){
    //enable pendsv interrupt used for task switching
    NVIC_SetPriority(PendSV_IRQn, 15);
    NVIC_EnableIRQ(PendSV_IRQn);
}