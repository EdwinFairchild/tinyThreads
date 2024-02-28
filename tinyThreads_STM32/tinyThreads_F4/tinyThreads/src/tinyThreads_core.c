#include "tinyThreads_core.h"
#include "tinyThreads_error.h"
#include "tinyThreads_time.h"
// TODO : save state of interrupts before entering cs and restore afterwards
static uint32_t cs_nesting = 0;
tinyThread_tcb *current_tcb = NULL;
uint32_t        sysThreadCount = 0;

// TODO remove debuging
extern void removeAllFromNoneReadyList();
extern void printNoneReadyList();

uint32_t systemThreadStack[200];

static void systemThread(uint32_t arg)
{
    while (1)
    {
        sysThreadCount++;
        // yield
        tt_ThreadYield(true);
    }
}

TinyThreadsStatus tt_CoreInit(void)
{
    // TODO : the err logic here is not correct, i think
    TinyThreadsStatus err = TINYTHREADS_OK;
    err = tt_MemoryInit();
    // add system related threads
    tinyThread_tcb_idx id = tt_ThreadAdd(systemThread, systemThreadStack, 200, 10, 1, (uint8_t *)"Sys thread", true);
    // os cannot function without system thread
    if (id >= 0)
    {
        tt_SetCurrentTcb(tt_ThreadGetTcbByID(id));
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

// TODO: I should have a scheduler file and this will go in there scheduler_round_robin.c
/**************************************************************************
 * System timer interrupt handler
 * - Increment the system tick
 * - Update threads in non ready state
 * - Find next ready thread and go to it
 * - Generate PendSV interrupt
 **************************************************************************/
void tt_CoreSystemTickHandler(void)
{
    port_dbg_signal_2_assert();
    current_tcb = tt_ThreadGetCurrentTcb();
    tt_tick_inc();
    // update threads in non ready state
    if (tt_ThreadGetInactiveThreadCount())
    {
        tt_ThreadUpdateInactive();
    }

    // update timers
    tt_TimerUpdate();

    // this should be shecked in the linked list for non ready threads
    // check the thread control block to see if its time to switch it out (Round Robin)
    if (current_tcb->period_ms <= (tt_TimeGetTick() - current_tcb->lastRunTime))
    {
        tt_ThreadUpdateNextThreadPtr();
        // generate pendsv interrupt
        SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
    }
    port_dbg_signal_2_deassert();
}

void tt_CoreCsEnter(void)
{
    // only need to do this once
    if (cs_nesting == 0)
    {
        TT_DISABLE_INTERRUPTS();
    }
    cs_nesting++;
}
void tt_CoreCsExit(void)
{
    cs_nesting--;
    if (cs_nesting == 0)
    {
        TT_ENABLE_INTERRUPTS();
    }
}