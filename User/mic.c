#include "User/mic.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "arm_math.h"
#include "stm32h7xx_hal.h"

#include "User/event_group.h"
#include "dfsdm.h"
#include "main.h"

__attribute__((section(".bss.DMA_RAM_D2"))) static int16_t mic_data[2][2][MKF360_AUDIO_PERIPH_DMA_DEST_SAMPLE_NUM];
__attribute__((section(".bss.DTCM"))) static volatile uint8_t mic1_idle_buffer;
__attribute__((section(".bss.DTCM"))) static volatile uint8_t mic2_idle_buffer;

void mic_start()
{
    HAL_StatusTypeDef ret_hal = HAL_OK;
    DFSDM_Filter_HandleTypeDef *const dfsdm_filters[] = {
        &hdfsdm1_filter0,
        &hdfsdm1_filter1,
    };

    for (size_t i = 0; i < sizeof(dfsdm_filters) / sizeof(dfsdm_filters[0]); i++)
    {
        ret_hal = HAL_DFSDM_FilterRegularMsbStart_DMA(dfsdm_filters[i], mic_data[i][0],
                                                      MKF360_AUDIO_PERIPH_DMA_DEST_SAMPLE_NUM * 2);
        if (ret_hal != HAL_OK)
        {
            printf("hdfsdm1 filter%u start fail, code: %u", i, ret_hal);
            Error_Handler();
        }
    }
}

void mic_stop()
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

int16_t *mic_get_mic1_buffer_address()
{
    return &mic_data[0][mic1_idle_buffer][0];
}

int16_t *mic_get_mic2_buffer_address()
{
    return &mic_data[1][mic1_idle_buffer][0];
}

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    if (hdfsdm_filter == &hdfsdm1_filter0)
    {
        mic1_idle_buffer = 0;
        event_group_set_event(EventGroup1, EventGroup1Mic1DataReady);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter1)
    {
        mic2_idle_buffer = 0;
        event_group_set_event(EventGroup1, EventGroup1Mic2DataReady);
    }
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    if (hdfsdm_filter == &hdfsdm1_filter0)
    {
        mic1_idle_buffer = 1;
        event_group_set_event(EventGroup1, EventGroup1Mic1DataReady);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter1)
    {
        mic2_idle_buffer = 1;
        event_group_set_event(EventGroup1, EventGroup1Mic2DataReady);
    }
}
