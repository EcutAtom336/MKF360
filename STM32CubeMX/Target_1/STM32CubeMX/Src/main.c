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
#include "dfsdm.h"
#include "dma.h"
#include "gpio.h"
#include "quadspi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>

#include "EventRecorder.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
    EventGroup1Mic1FirstHalfReady,
    EventGroup1Mic1SecondHalfReady,
    EventGroup1Mic2FirstHalfReady,
    EventGroup1Mic2SecondHalfReady,
    EventGroup1Mic3FirstHalfReady,
    EventGroup1Mic3SecondHalfReady,
    EventGroup1Mic4FirstHalfReady,
    EventGroup1Mic4SecondHalfReady,

    EventGroup1Max = 32,
} EventGroup1Idx_t;

#if EventGroup1Max >= 32
#error "Event group 1 event too many!"
#endif

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define AUDIO_FREQ (48000U)
#define DFSDM_DMA_FRAME_SAMPLE_COUNT (AUDIO_FREQ / 1000U)

#define MIC1_FH_RDY_BIT (1U << EventGroup1Mic1FirstHalfReady)
#define MIC1_SH_RDY_BIT (1U << EventGroup1Mic1SecondHalfReady)
#define MIC2_FH_RDY_BIT (1U << EventGroup1Mic2FirstHalfReady)
#define MIC2_SH_RDY_BIT (1U << EventGroup1Mic2SecondHalfReady)
#define MIC3_FH_RDY_BIT (1U << EventGroup1Mic3FirstHalfReady)
#define MIC3_SH_RDY_BIT (1U << EventGroup1Mic3SecondHalfReady)
#define MIC4_FH_RDY_BIT (1U << EventGroup1Mic4FirstHalfReady)
#define MIC4_SH_RDY_BIT (1U << EventGroup1Mic4SecondHalfReady)

#define MIC_FH_RDY_BIT (MIC1_FH_RDY_BIT | MIC2_FH_RDY_BIT | MIC3_FH_RDY_BIT | MIC4_FH_RDY_BIT)
#define MIC_SH_RDY_BIT (MIC1_SH_RDY_BIT | MIC2_SH_RDY_BIT | MIC3_SH_RDY_BIT | MIC4_SH_RDY_BIT)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

__attribute__((section("DTCM"))) static uint32_t event = 0;

__attribute__((section("DMA_RAM_D2"))) static int16_t mic1_data[2][DFSDM_DMA_FRAME_SAMPLE_COUNT];
__attribute__((section("DMA_RAM_D2"))) static int16_t mic2_data[2][DFSDM_DMA_FRAME_SAMPLE_COUNT];
__attribute__((section("DMA_RAM_D2"))) static int16_t mic3_data[2][DFSDM_DMA_FRAME_SAMPLE_COUNT];
__attribute__((section("DMA_RAM_D2"))) static int16_t mic4_data[2][DFSDM_DMA_FRAME_SAMPLE_COUNT];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
    MX_QUADSPI_Init();
    MX_DFSDM1_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    HAL_StatusTypeDef ret_hal = HAL_OK;
    uint32_t ret_uint32 = 0;

    ret_uint32 = EventRecorderInitialize(EventRecordAll, 1);
    if (ret_uint32 != 1)
    {
        printf("EventRecorderInitialize fail, code: %u", ret_uint32);
        abort();
    }

    ret_hal = HAL_DFSDM_FilterRegularMsbStart_DMA(&hdfsdm1_filter0, mic1_data[0], DFSDM_DMA_FRAME_SAMPLE_COUNT * 2);
    if (ret_hal != HAL_OK)
    {
        printf("hdfsdm1 filter0 start fail, code: %u", ret_hal);
        abort();
    }

    ret_hal = HAL_DFSDM_FilterRegularMsbStart_DMA(&hdfsdm1_filter1, mic2_data[0], DFSDM_DMA_FRAME_SAMPLE_COUNT * 2);
    if (ret_hal != HAL_OK)
    {
        printf("hdfsdm1 filter1 start fail, code: %u", ret_hal);
        abort();
    }

    ret_hal = HAL_DFSDM_FilterRegularMsbStart_DMA(&hdfsdm1_filter2, mic3_data[0], DFSDM_DMA_FRAME_SAMPLE_COUNT * 2);
    if (ret_hal != HAL_OK)
    {
        printf("hdfsdm1 filter2 start fail, code: %u", ret_hal);
        abort();
    }

    ret_hal = HAL_DFSDM_FilterRegularMsbStart_DMA(&hdfsdm1_filter3, mic4_data[0], DFSDM_DMA_FRAME_SAMPLE_COUNT * 2);
    if (ret_hal != HAL_OK)
    {
        printf("hdfsdm1 filter3 start fail, code: %u", ret_hal);
        abort();
    }

    uint32_t full_cnt[4] = {0};

    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        if (event & MIC1_FH_RDY_BIT)
        {
            ATOMIC_CLEAR_BIT(event, MIC1_FH_RDY_BIT);
        }
        else if (event & MIC1_SH_RDY_BIT)
        {
            if (full_cnt[0]++ % 50 >= 49)
            {
                __ASM("SVC 0");
            }
            ATOMIC_CLEAR_BIT(event, MIC1_SH_RDY_BIT);
        }
        else if (event & MIC2_FH_RDY_BIT)
        {
            ATOMIC_CLEAR_BIT(event, MIC2_FH_RDY_BIT);
        }
        else if (event & MIC2_SH_RDY_BIT)
        {
            if (full_cnt[1]++ % 50 >= 49)
            {
                __ASM("SVC 1");
            }
            ATOMIC_CLEAR_BIT(event, MIC2_SH_RDY_BIT);
        }
        else if (event & MIC3_FH_RDY_BIT)
        {
            ATOMIC_CLEAR_BIT(event, MIC3_FH_RDY_BIT);
        }
        else if (event & MIC3_SH_RDY_BIT)
        {
            if (full_cnt[2]++ % 50 >= 49)
            {
                __ASM("SVC 2");
            }
            ATOMIC_CLEAR_BIT(event, MIC3_SH_RDY_BIT);
        }
        else if (event & MIC4_FH_RDY_BIT)
        {
            ATOMIC_CLEAR_BIT(event, MIC4_FH_RDY_BIT);
        }
        else if (event & MIC4_SH_RDY_BIT)
        {
            if (full_cnt[3]++ % 50 >= 49)
            {
                __ASM("SVC 3");
            }
            ATOMIC_CLEAR_BIT(event, MIC4_SH_RDY_BIT);
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
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
    RCC_OscInitStruct.PLL.PLLN = 100;
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

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

void mkf360_svc_handler(uint32_t *stacked)
{
    uint16_t *pc_ptr = (uint16_t *)(stacked[6] - 2);
    uint8_t svc_number = (uint8_t)(*pc_ptr & 0xFF);

    switch (svc_number)
    {
    case 0: {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        break;
    }
    case 1: {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        break;
    }
    case 2: {
        HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
        break;
    }
    case 3: {
        HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
        break;
    }
    }
}

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    if (hdfsdm_filter == &hdfsdm1_filter0)
    {
        ATOMIC_SET_BIT(event, MIC1_FH_RDY_BIT);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter1)
    {
        ATOMIC_SET_BIT(event, MIC2_FH_RDY_BIT);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter2)
    {
        ATOMIC_SET_BIT(event, MIC3_FH_RDY_BIT);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter3)
    {
        ATOMIC_SET_BIT(event, MIC4_FH_RDY_BIT);
    }
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    if (hdfsdm_filter == &hdfsdm1_filter0)
    {
        ATOMIC_SET_BIT(event, MIC1_SH_RDY_BIT);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter1)
    {
        ATOMIC_SET_BIT(event, MIC2_SH_RDY_BIT);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter2)
    {
        ATOMIC_SET_BIT(event, MIC3_SH_RDY_BIT);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter3)
    {
        ATOMIC_SET_BIT(event, MIC4_SH_RDY_BIT);
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
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

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
