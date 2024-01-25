/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "errno.h"
#include "stdarg.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "tinyThreads_core.h"
#include "tinyThreads_error.h"
#include "tinyThreads_thread.h"
#include "tinyThreads_types.h"
#include <sys/unistd.h> // For STDOUT_FILENO, STDERR_FILENO
                        /* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
tinyThread_tcb_idx thread1_id, thread2_id, thread3_id;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*retarget printf */
int _write(int file, char *data, int len)
{
    // Only write to STDOUT and STDERR
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
    {
        errno = EBADF;
        return -1;
    }

    // Transmit data using UART2
    for (int i = 0; i < len; i++)
    {
        // Wait for the transmit buffer to be empty
        while (!(USART2->SR & USART_SR_TXE))
            ;
        // Send the character
        USART2->DR = (uint16_t)data[i];
    }

    // Wait for the transmission of the last byte to complete
    while (!(USART2->SR & USART_SR_TC))
        ;

    return len;
}

void thread1(void)
{

    // static uint32_t prev_runtime = 0;
    while (1)
    {
        // elapsed time since last run
        // uint32_t currentTime = tinyThread_tick_get();
        // uint32_t elapsed_time = currentTime - tt_ThreadGetLastRunTime();
        // printf("Thread 1: %d\r\n", (int)elapsed_time);
        printf("[1] T3SC: %d\r\n", (int)tt_ThreadGetSleepCount(3));
    }
}
void thread2(void)
{
    static uint32_t counter = 0;
    while (1)
    {
        // elapsed time since last run
        // uint32_t currentTime = tinyThread_tick_get();
        // uint32_t elapsed_time = currentTime - tt_ThreadGetLastRunTime();
        // printf("Thread 2: %d\r\n", elapsed_time);
        // if (counter == 1000)
        // {
        //     tt_ThreadPause(thread3_id);
        // }
        // if (counter == 2000)
        // {
        //     tt_ThreadUnPause(thread3_id);
        //     counter = 0;
        // }
        counter++;
        if (counter == 1000)
        {
            tt_ThreadPause(thread3_id);
        }
        if (counter == 2000)
        {
            tt_ThreadUnPause(thread3_id);
            // counter = 0;
        }
        printf("[2] T3SC: %d\r\n", (int)tt_ThreadGetSleepCount(3));
    }
}

void thread3(void)
{
    // static uint32_t prev_runtime = 0;
    // static uint32_t count = 0;
    while (1)
    {
        // elapsed time since last run
        // uint32_t currentTime = tinyThread_tick_get();
        // uint32_t elapsed_time = currentTime - tt_ThreadGetLastRunTime();
        // printf("Thread 3: %d\r\n", elapsed_time);
        // toggle led
        HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
        tt_ThreadSleep(500);
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    //  add user threads
    printf("Start of application\r\n");

    if (tt_CoreInit() == TINYTHREADS_OK)
    {
        thread1_id = tt_ThreadAdd(thread1, 10, 1);
        thread2_id = tt_ThreadAdd(thread2, 10, 1);
        thread3_id = tt_ThreadAdd(thread3, 10, 1);
        if (thread1_id == TINYTHREADS_MAX_THREADS_REACHED || thread2_id == TINYTHREADS_MAX_THREADS_REACHED ||
            thread3_id == TINYTHREADS_MAX_THREADS_REACHED)
        {
            printf("!!! Error adding threads !!!\r\n");
        }
        tt_CoreRun(); // should not return from this
    }
    else
    {
        printf("!!! Error initializing tinyKernel !!!\r\n");
    }
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    uint32_t count = 0;
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        printf("Error should not be here %d\r\n", (int)count++);
        HAL_Delay(500);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Activate the Over-Drive mode
     */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM5 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM5)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
