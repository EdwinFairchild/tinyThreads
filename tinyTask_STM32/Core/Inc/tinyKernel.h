#ifndef _TINY_KERNEL_H_
#define _TINY_KERNEL_H_

// deivce specific includes
#include "stm32f1xx.h"
#include <stdio.h>
/*
*   Initalizes a threads stacks
*/
typedef void (*thread_t)(void);
uint32_t tos_KernelStackInit(uint32_t i);
void tos_KernelStart(uint32_t quanta);
void tos_StartScheduler(void);

uint8_t tos_KernelAddThread(thread_t thread_1,thread_t thread_2,thread_t thread_3);
#endif // _TINY_KERNEL_H_