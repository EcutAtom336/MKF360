#include "User/audio_adc.h"

#include "arm_math.h"

#include "stm32h7xx_hal.h"

#include "MKF360_config.h"
#include "User/event_group.h"
#include "adc.h"
#include "tim.h"

__attribute__((section(".bss.BDMA_RAM_D3"))) int16_t adc3_dma_buffer[2][MKF360_DMA_FRAME_SAMPLE_NUM];
__attribute__((section(".bss.DTCM"))) static uint32_t idle_buffer;

void audio_adc_start()
{
    HAL_StatusTypeDef ret_hal = HAL_OK;

    ret_hal = HAL_ADC_Start_DMA(&hadc3, (uint32_t *)&adc3_dma_buffer[0][0], MKF360_DMA_FRAME_SAMPLE_NUM * 2U);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }

    ret_hal = HAL_TIM_Base_Start(&htim6);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
    idle_buffer = 1U;
}

void audio_adc_stop()
{
    HAL_StatusTypeDef ret_hal = HAL_OK;

    ret_hal = HAL_TIM_Base_Stop(&htim6);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }

    ret_hal = HAL_ADC_Stop_DMA(&hadc3);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
}

void audio_adc_read(int16_t *buffer)
{
    arm_copy_q15(&adc3_dma_buffer[idle_buffer][0], buffer, MKF360_DMA_FRAME_SAMPLE_NUM);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    idle_buffer = 0U;
    bool before = event_group_set_event(EventGroup1, EventGroup1Adc3DmaBufferReady);
    if (before)
    {
        Error_Handler();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    idle_buffer = 1U;
    bool before = event_group_set_event(EventGroup1, EventGroup1Adc3DmaBufferReady);
    if (before)
    {
        Error_Handler();
    }
}
