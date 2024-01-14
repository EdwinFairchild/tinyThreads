#ifndef TINYTASKSPORT_H
#define TINYTASKSPORT_H

#define STM32F4

#ifdef STM32F4
#include "stm32f4xx.h"
#endif


void tinyTask_printMsg(char *msg, ...);
#endif // TINYTASKSPORT_H
