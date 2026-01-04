/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "bdma.h"
#include "dac.h"
#include "dfsdm.h"
#include "dma.h"
#include "gpio.h"
#include "i2s.h"
#include "quadspi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "User/audio_buffer.h"
#include "User/audio_io.h"
#include "User/audio_processor.h"
#include "User/battery_monitor.h"
#include "User/event_group.h"
#include "User/retarget.h"
#include "audio/PCM_RES.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define IS_OUT_OF_DATE(current_tick, last_tick, interval) ((current_tick) - (last_tick) >= (interval))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* Enable the CPU Cache */

    /* Enable I-Cache---------------------------------------------------------*/
    SCB_EnableICache();

    /* Enable D-Cache---------------------------------------------------------*/
    SCB_EnableDCache();

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* Configure the peripherals common clocks */
    PeriphCommonClock_Config();

    /* USER CODE BEGIN SysInit */

    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    __HAL_RCC_D2SRAM2_CLK_ENABLE();
    __HAL_RCC_D2SRAM3_CLK_ENABLE();

    // 提前初始化QSPI Flash，以放置更多代码在QSPI Flash
    MX_QUADSPI_Init();
    // 清除ICache，放置ICache缓存了错误的指令
    SCB_InvalidateICache();

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_BDMA_Init();
    MX_QUADSPI_Init();
    MX_DFSDM1_Init();
    MX_USART1_UART_Init();
    MX_DAC1_Init();
    MX_ADC3_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_I2S3_Init();
    MX_ADC2_Init();
    /* USER CODE BEGIN 2 */

    HAL_Delay(1000);
    HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_SET);

    stdout_init();
    audio_io_init();
    audio_processor_init();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    uint32_t btn_low_continue_tick = 0;
    bool low_battery_triggered = false;

    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        audio_io_handler();

        if (audio_io_is_connected())
        {
            audio_process();
        }

        if (event_group_check_event(EventGroup1, EventGroup1AudioIoConnected, true))
        {
            printf("Audio IO connected.\n");
        }
        if (event_group_check_event(EventGroup1, EventGroup1AudioIoDisconnected, true))
        {
            audio_processor_reset();
            reset_audio_rb();
            low_battery_triggered = false;
            printf("Audio IO disconnected.\n");
        }
        if (event_group_check_event(EventGroup1, EventGroup1Tick50Pass, true))
        {
            stdout_maintain();
            battery_monitor_handler();
            if (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_RESET)
            {
                btn_low_continue_tick += 50;
                if (btn_low_continue_tick >= 3000)
                {
                    HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET);
                    HAL_PWR_EnterSTANDBYMode();
                }
            }
            else
            {
                btn_low_continue_tick = 0;
            }
        }
        if (event_group_check_event(EventGroup1, EventGroup1Tick250Pass, true))
        {
            if (audio_io_is_connected() == false)
            {
                HAL_GPIO_TogglePin(STAT_LED_GPIO_Port, STAT_LED_Pin);
            }
        }
        if (event_group_check_event(EventGroup1, EventGroup1Tick500Pass, true))
        {
            HAL_GPIO_TogglePin(SYS_LED_GPIO_Port, SYS_LED_Pin);
        }
        if (event_group_check_event(EventGroup1, EventGroup1Tick1000Pass, true))
        {
            if (audio_io_is_connected() == true)
            {
                HAL_GPIO_TogglePin(STAT_LED_GPIO_Port, STAT_LED_Pin);
            }
        }
        if (event_group_check_event(EventGroup1, EventGroup1Tick5000Pass, true))
        {
            float_t bat_voltage = battery_monitor_get_voltage();
            printf("Battery voltage: %.2fV\n", bat_voltage);
            if (bat_voltage < 3.3F && audio_io_is_connected() && low_battery_triggered == false)
            {
                low_battery_triggered = true;
                audio_io_play_prompt(BATTERY_LOW_PCM_IDX);
            }
        }
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

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 120;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Initializes the peripherals clock
     */
    PeriphClkInitStruct.PeriphClockSelection =
        RCC_PERIPHCLK_USB | RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_SAI1 | RCC_PERIPHCLK_SPI3 | RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.PLL2.PLL2M = 1;
    PeriphClkInitStruct.PLL2.PLL2N = 48;
    PeriphClkInitStruct.PLL2.PLL2P = 5;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 5;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.PLL3.PLL3M = 1;
    PeriphClkInitStruct.PLL3.PLL3N = 30;
    PeriphClkInitStruct.PLL3.PLL3P = 5;
    PeriphClkInitStruct.PLL3.PLL3Q = 5;
    PeriphClkInitStruct.PLL3.PLL3R = 5;
    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
    PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_PLL3;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

void period_event_tick()
{
    static size_t last_tick[5] = {0U};
    const size_t interval_tick[] = {50U, 250U, 500U, 1000U, 5000U};
    const uint32_t event_bit[] = {EventGroup1Tick50Pass, EventGroup1Tick250Pass, EventGroup1Tick500Pass,
                                  EventGroup1Tick1000Pass, EventGroup1Tick5000Pass};
    size_t tick = HAL_GetTick();
    for (size_t i = 0; i < sizeof(last_tick) / sizeof(last_tick[0]); i++)
    {
        if (IS_OUT_OF_DATE(tick, last_tick[i], interval_tick[i]))
        {
            last_tick[i] = tick;
            event_group_set_event(EventGroup1, event_bit[i]);
        }
    }
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    MPU_InitStruct.BaseAddress = 0x30000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Number = MPU_REGION_NUMBER2;
    MPU_InitStruct.BaseAddress = 0x38000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    HAL_UART_Transmit(&huart1, (uint8_t *)"Error!", sizeof("Error!"), 1000);
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
