#ifndef TINYTASKSPORT_H
#define TINYTASKSPORT_H

#define STM32F4

#ifdef STM32F4
#include "stm32f4xx.h"
#endif

void tinyTask_port_enable_tick_timer(void);
void tinyTasks_enable_context_switching_isr(void);
void tinyTasks_printMsg_init(void (*printMsgPtr)(char *msg, ...));
void tinyTask_printMsg(char *msg, ...);


#endif // TINYTASKSPORT_H
