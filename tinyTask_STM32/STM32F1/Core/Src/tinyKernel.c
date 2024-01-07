#include "tinyKernel.h"

// ************| Configurations |*****************************************************************
#define NUM_OF_THREADS      3
#define STACKSIZE           128
#define TOP_OF_STACK        (STACKSIZE -1)
#define THUMB_BIT_LOCATION (1<<24)
#define BUS_FREQ           (uint64_t)SystemCoreClock  // 72MHz
uint32_t MILLIS_PRESCALER = 0;
// ************| Locals |*****************************************************************

struct tcb_t; // forward declare struct to include it in the typedef
typedef struct tcb_t { 
    uint32_t *sp;
    struct tcb_t *next;
}tcb_t;

//  TCB for each thread
tcb_t tcbs[NUM_OF_THREADS];
tcb_t *currentTCB;

uint32_t TCB_STACK[NUM_OF_THREADS][STACKSIZE];

// ************| Prototypes |*****************************************************************
uint32_t tos_KernelStackInit(uint32_t i)
{  
    tcbs[i].sp = &TCB_STACK[i][STACKSIZE - 16];
    //initialize the stack to known values
    // Registers used according to the exception model
    TCB_STACK[i][STACKSIZE -1 ] = THUMB_BIT_LOCATION;   // Thumb bit in PSR (Program Status Register)
    // TCB_STACK[i][TOP_OF_STACK - 2] Program Counter initialized elsewhere when we get a thread function
   
    TCB_STACK[i][STACKSIZE - 3] = 0xDEADBEEF;           // R12
    TCB_STACK[i][STACKSIZE - 4] = 0xDEADBEEF;           // R3
    TCB_STACK[i][STACKSIZE - 5] = 0xDEADBEEF;           // R2
    TCB_STACK[i][STACKSIZE - 6] = 0xDEADBEEF;           // R1
    TCB_STACK[i][STACKSIZE - 7] = 0xDEADBEEF;           // R0
    //----------------------------------------------------------------------------------------
    TCB_STACK[i][STACKSIZE - 8]  = 0xDEADBEEF;           // R11   
    TCB_STACK[i][STACKSIZE - 9] = 0xDEADBEEF;          // R10
    TCB_STACK[i][STACKSIZE - 10] = 0xDEADBEEF;          // R9
    TCB_STACK[i][STACKSIZE - 11] = 0xDEADBEEF;          // R8
    TCB_STACK[i][STACKSIZE - 12] = 0xDEADBEEF;          // R7
    TCB_STACK[i][STACKSIZE - 13] = 0xDEADBEEF;           // R6
    TCB_STACK[i][STACKSIZE - 14] = 0xDEADBEEF;           // R5
    TCB_STACK[i][STACKSIZE - 15] = 0xDEADBEEF;           // R4
    // everything after this point is the remaining space for the tasks stack 
    // and therefore the location where stack pointer points to
    tcbs[i].sp = &TCB_STACK[i][STACKSIZE - 16];
    
    // TODO : return codes
    return 0;
}

uint8_t tos_KernelAddThread(thread_t thread_1,thread_t thread_2,thread_t thread_3)
{
    // disable global interrupts
    __disable_irq();
    // add thread to the linked list in a circular fashion
    tcbs[0].next = &tcbs[1];
    tcbs[1].next = &tcbs[2];
    tcbs[2].next = &tcbs[0];

    // initialize the stack for the thread
    tos_KernelStackInit(0); 
    tos_KernelStackInit(1);
    tos_KernelStackInit(2);

    // init PC with the thread function
    TCB_STACK[0][STACKSIZE - 2] = (uint32_t)(thread_1); 
    TCB_STACK[1][STACKSIZE - 2] = (uint32_t)(thread_2);
    TCB_STACK[2][STACKSIZE - 2] = (uint32_t)(thread_3);

    currentTCB = &tcbs[0]; // set the current TCB to the first thread

    // enable global interrupts
    __enable_irq();

    // TODO : return codes
    return 0;
}

uint8_t tos_KernelInit(void)
{

    // TODO : return codes
    return 0;
}

void tos_KernelStart(uint32_t quanta)
{
    //using systick as RTOS tick timer
    // reset systick
    SysTick->CTRL = 0;
    SysTick->VAL = 0;
    // load quanta
    MILLIS_PRESCALER =  (BUS_FREQ/1000); // 1000Hz/1ms
    SysTick->LOAD =((quanta * MILLIS_PRESCALER) - 1) ;
    // set priority to lowest
    NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
    //enable systick interrupt
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    // enable systick
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    //launch scheduler
    tos_StartScheduler();

}
/*
When you apply this attribute to a function, 
it instructs the compiler to not generate any prologue or epilogue code for that function. 
In other words, 
it won't generate the standard machine code that typically sets up (prologue) and tears down (epilogue) 
the stack frame for that function. 
This allows for more direct control over the function's assembly.

Here's what it does:

No Prologue: 
    Normally, when a function is called, 
    the compiler generates a prologue that prepares the stack 
    and registers to execute the function. 
    This might involve pushing certain registers onto the stack, 
    setting up a stack frame, etc.

No Epilogue: 
    Likewise, at the end of a function, 
    the compiler generates an epilogue to clean up the stack frame, 
    restore registers, and return to the calling function.

Manual Management Required: When using naked,
the responsibility is on the programmer to manage the stack and registers correctly. 
If you don't do this, it could result in very hard-to-debug problems.

*/
__attribute__((naked)) void SysTick_Handler(void)
{
    // assembly break point

    //suspend current thread
    __disable_irq();
    // save R4-R11 onto the c urrent threads stack
    __asm("PUSH {R4-R11}");

    /* 
    save StackPointer to currentTCB
    This instruction loads the address of the currentTCB variable into the R0 register. 
    After this instruction, R0 holds the memory address of currentTCB.
    */ 
    __asm("LDR R0, =currentTCB");

    /*
    The square brackets around R0 ([R0]) indicate that this is a memory dereference operation. 
    Instead of treating the value in R0 as a regular number, 
    the ARM assembly treats it as a memory address and loads the value at that memory address.
    */
    __asm("LDR R1, [R0]");

    /*
    This instruction stores the value of the stack pointer (SP) 
    at the address pointed to by R1 
    (which is the address of the currentTCB structure , so basically pointing to the 
    first element which is the sp ).
    */
    __asm("STR SP, [R1]");

    /*
    Select next thread to run
    Since R1 currently points to the currentTCB structure, specifically the sp element,
    we can can add four to this address to get the next pointer.
    R2 will now point to the nextTCB structure.
    */
    __asm("LDR R2, [R1, #4]"); // load the next pointer into R2

    /*
    Since R2 now points to the nextTCB structure, and R0 points to the currentTCB variable,
    We can update the currentTCB variable to point to the nextTCB structure. 
    */
    __asm("STR R2, [R0]");

    /*
    This instruction loads the value of the stack pointer from the currentTCB variable
    and stores it into the SP register.
    */
    __asm("LDR SP, [R2]");

    // restore R4-R11 from the next threads stack
    __asm("POP {R4-R11}");

    // enable interrupts
    __enable_irq();

    // return from interrupt
    __asm("BX LR");

}

void tos_StartScheduler(void)
{
    // disable interrupts
    __disable_irq();
    // load address of the first thread into the PC
    __asm("LDR R0, =currentTCB");
    // load the currentTCB into R1
    __asm("LDR R1, [R0]"); 
    // load the sp into the SP register
    __asm("LDR SP, [R1]"); 
    // restore R4-R11
    __asm("POP {R4-R12}");
    // Restor R0-R3
    __asm("POP {R0-R3}");
    // skip LR 
    __asm("ADD SP,SP, #4");
    // create new stack frame
    __asm("POP {LR}");
    // skip psr
    __asm("ADD SP,SP, #4");
    // enable interrupts
    __enable_irq();
    // return from interrupt
    __asm("BX LR");

}

void tos_KernelYield(void)
{
    // since systick is a countdown timer, we can just set the value to 0
    // and triggger an interrupt thus forcing a task switch
    SysTick->VAL = 0;
    // trigger systick interrupt
    SCB->ICSR |= SCB_ICSR_PENDSTSET_Msk;
    // return from interrupt
    __asm("BX LR");
}