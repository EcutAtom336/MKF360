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
#include "mdma.h"
#include "quadspi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "usbd_core.h"

#include "MKF360_config.h"
#include "User/audio_dac.h"
#include "User/audio_iis.h"
#include "User/mic.h"
#include "User/usb_desc.h"
#include "audio/PCM_RES.h"
#include "mdma.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
    None,
    Usb,
} InterfaceType_t;

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

__attribute__((section(".DTCM"))) static InterfaceType_t current_interface = None;

__attribute__((section(".bss.DTCM"))) static uint32_t event;

// MDMA链接寄存器必须双字对齐

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

static void on_mic_interlaced_data_ready();
static void on_dac_buffer_empty(void *frame, const size_t sample_num);
static void common_connect();
static void common_disconnect();

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
    MX_MDMA_Init();
    MX_BDMA_Init();
    MX_QUADSPI_Init();
    MX_DFSDM1_Init();
    MX_USART1_UART_Init();
    MX_DAC1_Init();
    MX_I2S3_Init();
    MX_ADC3_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    /* USER CODE BEGIN 2 */

    HAL_GPIO_WritePin(BT_DISABLE__GPIO_Port, BT_DISABLE__Pin, GPIO_PIN_RESET);

    mic_mdma_init();
    iis_start();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    audio_dac_register_on_dac_dma_buffer_empty_isr_callback(on_dac_buffer_empty);

    usb_init(0, USB_OTG_FS_PERIPH_BASE);
    uint8_t flag = 0;
    uint32_t verify_pass_cnt = 0;

    register_mic_interlaced_data_ready_callback(on_mic_interlaced_data_ready);

    audio_dac_ctl(AudioDacCmdEnableCh1);

    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        if (event & EVENT_BIT(EventGroup1MicDataInterlaceComplete))
        {
            ATOMIC_CLEAR_BIT(event, EVENT_BIT(EventGroup1MicDataInterlaceComplete));
            // 验证Mic data被MDMA正确交错
            if (mic_verify_interlaced_data())
            {
                if (++verify_pass_cnt >= 100U)
                {
                    verify_pass_cnt = 0;
                    printf("MDMA interlace mic data right 100 times. Tick: %u\n", HAL_GetTick());
                }
            }
            else
            {
                printf("MDMA interlace mic data error!\n");
                Error_Handler();
            }
            flag = flag == 0 ? 1 : 0;
        }
        else if (event & EVENT_BIT(EventGroup1UacConnect))
        {
            ATOMIC_CLEAR_BIT(event, EVENT_BIT(EventGroup1UacConnect));
            common_connect();
            current_interface = Usb;
            printf("USB connect.\n");
        }
        else if (event & EVENT_BIT(EventGroup1Disconnect))
        {
            ATOMIC_CLEAR_BIT(event, EVENT_BIT(EventGroup1Disconnect));
            common_disconnect();
            current_interface = None;
            printf("USB disconnect.\n");
        }
        else if (event & EVENT_BIT(EventGroup1Tick500Pass))
        {
            ATOMIC_CLEAR_BIT(event, EVENT_BIT(EventGroup1Tick500Pass));
            HAL_GPIO_TogglePin(SYS_LED_GPIO_Port, SYS_LED_Pin);
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
        RCC_PERIPHCLK_USB | RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_SAI1 | RCC_PERIPHCLK_SPI3;
    PeriphClkInitStruct.PLL2.PLL2M = 1;
    PeriphClkInitStruct.PLL2.PLL2N = 32;
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
    PeriphClkInitStruct.PLL3.PLL3R = 8;
    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
    PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

size_t n;
static void on_dac_buffer_empty(void *frames, const size_t sample_num)
{
    if (n + sample_num >= BOOT_PCM_LEN)
    {
        return;
    }
    audio_dac_util_write_ch(frames, &BOOT_PCM[n], sample_num, DacCh1);
    audio_dac_util_write_ch(frames, &BOOT_PCM[n], sample_num, DacCh2);
    n += sample_num;
}

static void common_connect()
{
    mic_start();
}

static void common_disconnect()
{
    mic_stop();
}

void on_uac_connect()
{
    ATOMIC_SET_BIT(event, EVENT_BIT(EventGroup1UacConnect));
}

void on_disconnect()
{
    if (current_interface == Usb)
    {
        ATOMIC_SET_BIT(event, EVENT_BIT(EventGroup1Disconnect));
    }
}

void period_event_generator()
{
    static size_t last_tick[] = {0U};
    const size_t interval_tick[] = {500U};
    const uint32_t event_bit[] = {EVENT_BIT(EventGroup1Tick500Pass)};
    size_t tick = HAL_GetTick();
    for (size_t i = 0; i < sizeof(last_tick) / sizeof(last_tick[0]); i++)
    {
        if (IS_OUT_OF_DATE(tick, last_tick[i], interval_tick[i]))
        {
            last_tick[i] = tick;
            ATOMIC_SET_BIT(event, event_bit[i]);
        }
    }
}

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

void HAL_PCD_MspInit(PCD_HandleTypeDef *pcdHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (pcdHandle->Instance == USB_OTG_FS)
    {
        /* USER CODE BEGIN USB_OTG_FS_MspInit 0 */

        /* USER CODE END USB_OTG_FS_MspInit 0 */

        /** Initializes the peripherals clock
         */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
        PeriphClkInitStruct.PLL3.PLL3M = 1;
        PeriphClkInitStruct.PLL3.PLL3N = 30;
        PeriphClkInitStruct.PLL3.PLL3P = 5;
        PeriphClkInitStruct.PLL3.PLL3Q = 5;
        PeriphClkInitStruct.PLL3.PLL3R = 2;
        PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
        PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
        PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
            Error_Handler();
        }

        /** Enable USB Voltage detector
         */
        HAL_PWREx_EnableUSBVoltageDetector();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USB_OTG_FS GPIO Configuration
        PA11     ------> USB_OTG_FS_DM
        PA12     ------> USB_OTG_FS_DP
        */
        GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USB_OTG_FS clock enable */
        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

        /* USB_OTG_FS interrupt Init */
        HAL_NVIC_SetPriority(OTG_FS_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
        /* USER CODE BEGIN USB_OTG_FS_MspInit 1 */

        /* USER CODE END USB_OTG_FS_MspInit 1 */
    }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcdHandle)
{

    if (pcdHandle->Instance == USB_OTG_FS)
    {
        /* USER CODE BEGIN USB_OTG_FS_MspDeInit 0 */

        /* USER CODE END USB_OTG_FS_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USB_OTG_FS_CLK_DISABLE();

        /**USB_OTG_FS GPIO Configuration
        PA11     ------> USB_OTG_FS_DM
        PA12     ------> USB_OTG_FS_DP
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

        /* USB_OTG_FS interrupt Deinit */
        HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
        /* USER CODE BEGIN USB_OTG_FS_MspDeInit 1 */

        /* USER CODE END USB_OTG_FS_MspDeInit 1 */
    }
}

static void on_mic_interlaced_data_ready()
{
    ATOMIC_SET_BIT(event, EVENT_BIT(EventGroup1MicDataInterlaceComplete));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // CherryUSB不支持USB disconnect事件触发，
    // 使用VBUS下降沿触发USB disconnect
    if (GPIO_Pin == VBUS_DETECT_Pin)
    {
        on_disconnect();
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
    MPU_InitStruct.BaseAddress = 0x30040000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
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
