#ifndef TINYTHREADSPORT_H
#define TINYTHREADSPORT_H

#define STM32F4

#ifdef STM32F4
#include "stm32f4xx.h"
#endif

void tinyThread_port_enable_tick_timer(void);
void tinyThreads_enable_context_switching_isr(void);
void tinyThreads_printMsg_init(void (*printMsgPtr)(char *msg, ...));
void tinyThread_printMsg(char *msg, ...);

#endif // TINYTHREADSPORT_H
