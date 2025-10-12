#include "User/audio_adc.h"

#include <stdio.h>

#include "stm32h7xx_hal.h"

#include "MKF360_config.h"
#include "adc.h"
#include "tim.h"

__attribute__((section(".bss.BDMA_RAM_D3"))) int16_t adc3_dma_buffer[2][MKF360_DMA_FRAME_SAMPLE_NUM];

__attribute__((section(".bss.DTCM"))) OnAdcDmaBufferReadyIsrCallback_t callback;

void audio_adc_register_on_adc_dma_buffer_ready_isr_callback(OnAdcDmaBufferReadyIsrCallback_t cb)
{
    callback = cb;
}

void audio_adc_start()
{
    HAL_StatusTypeDef ret_hal = HAL_OK;

    if (callback == NULL)
    {
        printf("on adc dma buffer ready isr callback is null.\n");
    }

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

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    if (callback != NULL)
    {
        callback(&adc3_dma_buffer[0][0], MKF360_DMA_FRAME_SAMPLE_NUM);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    if (callback != NULL)
    {
        callback(&adc3_dma_buffer[0][0], MKF360_DMA_FRAME_SAMPLE_NUM);
    }
}
