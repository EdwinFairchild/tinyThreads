#ifndef TINYTHREADSPORT_H
#define TINYTHREADSPORT_H

#define STM32F4

#ifdef STM32F4
#include "stm32f4xx.h"
#endif
// clang-format off
#define FORCE_YEILD_INTERRUPT() SCB->ICSR|=SCB_ICSR_PENDSVSET_Msk

#define TT_DISABLE_INTERRUPTS() __disable_irq()
#define TT_ENABLE_INTERRUPTS() __enable_irq()

// clang-format on
void tinyThread_port_enable_tick_timer(void);
void tinyThreads_enable_context_switching_isr(void);
void tinyThreads_printMsg_init(void (*printMsgPtr)(char *msg, ...));
void tinyThread_printMsg(char *msg, ...);

#endif // TINYTHREADSPORT_H
