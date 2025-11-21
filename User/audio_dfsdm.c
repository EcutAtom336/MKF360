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
__attribute__((section(".bss.DTCM"))) static volatile uint32_t filter0_latest_timestamp;
__attribute__((section(".bss.DTCM"))) static volatile uint32_t filter1_latest_timestamp;

void audio_dfsdm_start()
{
    HAL_StatusTypeDef ret_hal = HAL_OK;
    DFSDM_Filter_HandleTypeDef *const dfsdm_filters[] = {
        &hdfsdm1_filter0,
        &hdfsdm1_filter1,
    };

    for (size_t i = 0; i < sizeof(dfsdm_filters) / sizeof(dfsdm_filters[0]); i++)
    {
        ret_hal =
            HAL_DFSDM_FilterRegularMsbStart_DMA(dfsdm_filters[i], filter_dma_buffer[i][0],
                                                MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * 2);
        if (ret_hal != HAL_OK)
        {
            printf("hdfsdm1 filter%u start fail, code: %u", i, ret_hal);
            Error_Handler();
        }
    }
}

void audio_dfsdm_stop()
{
    HAL_StatusTypeDef ret_hal = HAL_OK;
    DFSDM_Filter_HandleTypeDef *const dfsdm_filters[] = {
        &hdfsdm1_filter0,
        &hdfsdm1_filter1,
    };

    for (size_t i = 0; i < sizeof(dfsdm_filters) / sizeof(dfsdm_filters[0]); i++)
    {
        ret_hal = HAL_DFSDM_FilterRegularStop_DMA(dfsdm_filters[i]);
        if (ret_hal != HAL_OK)
        {
            printf("hdfsdm1 filter%u stop fail, code: %u", i, ret_hal);
            Error_Handler();
        }
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

uint32_t audio_dfsdm_get_filter0_latest_timestamp()
{
    return filter0_latest_timestamp;
}

uint32_t audio_dfsdm_get_filter1_latest_timestamp()
{
    return filter1_latest_timestamp;
}

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    if (hdfsdm_filter == &hdfsdm1_filter0)
    {
        filter0_idle_buffer = 0;
        filter0_latest_timestamp = HAL_GetTick();
        event_group_set_event(EventGroup1, EventGroup1DfsdmFilter0DmaBufferReady);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter1)
    {
        filter1_idle_buffer = 0;
        filter1_latest_timestamp = HAL_GetTick();
        event_group_set_event(EventGroup1, EventGroup1DfsdmFilter1DmaBufferReady);
    }
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    if (hdfsdm_filter == &hdfsdm1_filter0)
    {
        filter0_idle_buffer = 1;
        filter0_latest_timestamp = HAL_GetTick();
        event_group_set_event(EventGroup1, EventGroup1DfsdmFilter0DmaBufferReady);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter1)
    {
        filter1_idle_buffer = 1;
        filter1_latest_timestamp = HAL_GetTick();
        event_group_set_event(EventGroup1, EventGroup1DfsdmFilter1DmaBufferReady);
    }
}
