#include "tinyThreads_port.h"
#include "tinyKernel.h"
#include "tinyThreads_debug.h"

void tinyThread_port_enable_tick_timer(void)
{
    // TODO : use timer periph instead of systick
    // enable systick interrupt
    NVIC_SetPriority(SysTick_IRQn, 15);
    NVIC_EnableIRQ(SysTick_IRQn);
    // start systick timer with 1 ms period
    SysTick_Config(SystemCoreClock / 1000);
}

void tinyThreads_enable_context_switching_isr(void)
{
    // enable pendsv interrupt used for thread switching
    NVIC_SetPriority(PendSV_IRQn, 15);
    NVIC_EnableIRQ(PendSV_IRQn);
}
