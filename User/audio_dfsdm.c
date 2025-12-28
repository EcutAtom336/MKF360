#include "User/audio_dfsdm.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "arm_math.h"
#include "stm32h7xx_hal.h"

#include "User/event_group.h"
#include "dfsdm.h"
#include "main.h"

__attribute__((section(".bss.DMA_RAM_D2"))) static int16_t
    filter_dma_buffer[2][2][MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST];
__attribute__((section(".bss.DTCM"))) static volatile uint8_t filter0_idle_buffer;
__attribute__((section(".bss.DTCM"))) static volatile uint8_t filter1_idle_buffer;

void audio_dfsdm_start()
{
    HAL_StatusTypeDef ret_hal = HAL_OK;

    ret_hal =
        HAL_DFSDM_FilterRegularMsbStart_DMA(&hdfsdm1_filter1, filter_dma_buffer[1][0],
                                            MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * 2);
    if (ret_hal != HAL_OK)
    {
        printf("hdfsdm1 filter1 start fail, code: %u", ret_hal);
        Error_Handler();
    }

    ret_hal =
        HAL_DFSDM_FilterRegularMsbStart_DMA(&hdfsdm1_filter0, filter_dma_buffer[0][0],
                                            MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * 2);
    if (ret_hal != HAL_OK)
    {
        printf("hdfsdm1 filter0 start fail, code: %u", ret_hal);
        Error_Handler();
    }
}

void audio_dfsdm_stop()
{
    HAL_StatusTypeDef ret_hal = HAL_OK;

    ret_hal = HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter1);
    if (ret_hal != HAL_OK)
    {
        printf("hdfsdm1 filter1 stop fail, code: %u", ret_hal);
        Error_Handler();
    }

    ret_hal = HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
    if (ret_hal != HAL_OK)
    {
        printf("hdfsdm1 filter0 stop fail, code: %u", ret_hal);
        Error_Handler();
    }
}

int16_t *audio_dfsdm_get_filter0_buffer_address()
{
    return &filter_dma_buffer[0][filter0_idle_buffer][0];
}

int16_t *audio_dfsdm_get_filter1_buffer_address()
{
    return &filter_dma_buffer[1][filter1_idle_buffer][0];
}

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    if (hdfsdm_filter == &hdfsdm1_filter0)
    {
        filter0_idle_buffer = 0;
        event_group_set_event(EventGroup1, EventGroup1DfsdmFilter0DmaBufferReady);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter1)
    {
        filter1_idle_buffer = 0;
        event_group_set_event(EventGroup1, EventGroup1DfsdmFilter1DmaBufferReady);
    }
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    if (hdfsdm_filter == &hdfsdm1_filter0)
    {
        filter0_idle_buffer = 1;
        event_group_set_event(EventGroup1, EventGroup1DfsdmFilter0DmaBufferReady);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter1)
    {
        filter1_idle_buffer = 1;
        event_group_set_event(EventGroup1, EventGroup1DfsdmFilter1DmaBufferReady);
    }
}

void HAL_DFSDM_FilterErrorCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    if (hdfsdm_filter == &hdfsdm1_filter0)
    {
        event_group_set_event(EventGroup1, EventGroup1DfsdmFilter0DmaError);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter1)
    {
        event_group_set_event(EventGroup1, EventGroup1DfsdmFilter1DmaError);
    }
}
