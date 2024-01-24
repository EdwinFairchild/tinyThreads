#include "tinyThreads_core.h"
#include "tinyThreads_error.h"

tinyThread_tcb *current_tcb = NULL;
static void systemThread(void)
{
    static volatile uint32_t threadCount = 0;
    while (1)
    {
        threadCount++;
    }
}

TinyThreadsStatus tt_CoreInit(void)
{
    TinyThreadsStatus err = TINYTHREADS_OK;
    // add system related threads
    err = tt_ThreadAdd(systemThread, 10, 1);
    // os cannot function without system thread
    if (err != TINYTHREADS_OK)
    {
        return err;
    }

    return err;
}

/**************************************************************************
 * Run the kernel
 * - Load the stack pointer for the current thread
 * - Pop the exception frame from the stack and return to the thread
 **************************************************************************/
TinyThreadsStatus tt_CoreRun(void)
{
    TinyThreadsStatus err = TINYTHREADS_OK;
    current_tcb = tt_ThreadGetCurrentTcb();

    tinyThread_port_enable_tick_timer();
    tinyThreads_enable_context_switching_isr();

    // disable interrupts
    __disable_irq();
    /*  Load the address of tinyThread_current_tcb into R0 */
    __asm("LDR r0, =current_tcb");

    /*  Dereference the address at R0
        the first element of the tinyThread_tcb struct is the tak's stack pointer
        Now R1 will contain the address of the stack pointer for the current tinyThread_tcb
    */
    __asm("LDR r1, [r0]");

    /*  Load the value at the address pointed to by R1 (stack's stack pointer)
        into the stack pointer register */
    __asm("LDR sp, [r1]");

    /*  Pop the exception frame from the stack and return to the thread */
    __asm("POP {r4-r12}");
    __asm("POP {r0-r3}");

    /*  skip LR in our stack by adding 4 the current SP */
    __asm("ADD sp, sp, 4");
    /*  Now we are at the PC in our stack, pop that into the LR
        this will cause the processor to jump to the thread function.
        The PC in our thread was initialized in tt_ThreadAdd2
     */
    __asm("POP {lr}");
    /*  skip psr by adding 4 the current SP */
    __asm("ADD sp, sp, #4");

    /* enable interrupts */
    __enable_irq();

    /*  return from exception
        this will jump to the value in the LR register
        which is the thread function
    */
    __asm("BX LR");

    return err;
}