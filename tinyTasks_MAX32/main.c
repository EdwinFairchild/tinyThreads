/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

/**
 * @file    main.c
 * @brief   Hello World!
 * @details This example uses the UART to print to a terminal and flashes an LED.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "led.h"
#include "pb.h"
#include "board.h"
#include "mxc_delay.h"
#include "tinyKernel.h"
/***** Definitions *****/

/***** Globals *****/

/***** Functions *****/

// *****************************************************************************
void task_0(void)
{
  while(1)
  {
    printf("In task 0\n");
    
  }
}
void task_1(void)
{
  while(1)
  {
    LED_On(LED_RED);
    MXC_Delay(MXC_DELAY_MSEC(500));
    LED_Off(LED_RED);
    MXC_Delay(MXC_DELAY_MSEC(500));
  }
}
void task_2(void)
{
  while(1)
  {
    printf("in task 3\n");
  }
}

int main(void)
{
    tos_KernelStart(1000);
    int count = 0;

    printf("Hello World!\n");
    tos_KernelStart(100);

    while (1) {
        LED_On(LED_RED);
      //  MXC_Delay(MXC_DELAY_MSEC(500));
        LED_Off(LED_RED);
      //  MXC_Delay(MXC_DELAY_MSEC(500));
      //  printf("count = %d\n", count++);
    }
}
