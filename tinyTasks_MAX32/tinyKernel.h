#ifndef _TINY_KERNEL_H_
#define _TINY_KERNEL_H_

// deivce specific includes
#include "mxc_device.h"
#include <stdio.h>
/*
*   Initalizes a threads stacks
*/
uint32_t tos_KernelStackInit(uint32_t i);
void tos_KernelStart(uint32_t quanta);
void tos_StartScheduler(void);
#endif // _TINY_KERNEL_H_