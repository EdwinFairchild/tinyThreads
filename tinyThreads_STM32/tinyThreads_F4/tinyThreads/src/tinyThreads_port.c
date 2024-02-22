#include "tinyThreads_port.h"
#include "tinyThreads_core.h"

// TODO :  is there a better way to do this?
extern tinyThread_tcb tinyThread_current_tcb;
extern tinyThread_tcb tinyThread_next_tcb;

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

/* enter exception
    this will push the exception frame onto the stack
    and set the stack pointer to the top of the stack, knowing this
    information we save the stack pointer for the current thread
    and load the stack pointer for the next thread
*/
__attribute__((naked)) void PendSV_Handler(void)
{
    port_dbg_signal_1_assert();
    // disble interrupts
    __disable_irq();
    // clear pendsv interrupt
    SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;
    // this is defined to be an inline function so we are not altering the stack
    NVIC_EnableIRQ(PendSV_IRQn);

    /************************ Suspend and save current thread ************************/
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

    // save r4-r11 on the stack
    __asm("PUSH {r4-r11}");

    __asm("STR sp, [r1]");

    /************************ Restore next thread ************************
     * tt_CoreSystemTickHandler function will determine which thread comes next
     * and set tinyThread_next_tcb */
    __asm("LDR r1, =tinyThread_next_tcb");
    /*  derefrence the address at r1 (get the value at that address)
        the address found at the first element of the tinyThread_tcb struct is the stack pointer
        now R1 will contain the address of the stack pointer for the next tinyThread_tcb
    */
    __asm("LDR r1, [r1]");
    /*  Since R1 now points to the next tinyThread_tcb and the first element of any tinyThread_tcb struct
        is the stack pointer for that given taks stack, now load the value at the address pointed to by r1
        into the stack pointer register
    */
    __asm("LDR sp, [r1]");
    /*  R1 still containts next threads address, so lets update tinyThread_current_tcb whos address
        we should still have in R0 from above */
    __asm("STR r1, [r0]");

    /* tinyThread_current_tcb has a timestamp 5 words from the start lets update that to tick value*/
    __asm("LDR r2, =tinyThread_tick"); // load current tick time
    __asm("LDR r2, [r2]");
    __asm__("STR r2, [r1, %0]" ::"I"(TINYTASK_TCB_LAST_RUNTIME_OFFSET)); // update last run time in tcb

    /*  restore r4-r11 from the stack, since we just updated the stack pointer to point to the next threads stack
        it will pop the values from there.
    */
    __asm("POP {r4-r11}");

    // enable interrupts
    __enable_irq();

    /*  return from interrupt
        this will pop the exception frame from the stack and return to the next thread
    */

    // port_dbg_signal_1_deassert();
    port_dbg_signal_1_deassert();

    __asm("BX LR");
}