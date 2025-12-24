#include "User/audio_iis.h"

#include <stdbool.h>

#include "arm_math.h"

#include "stm32h7xx_hal.h"

#include "User/event_group.h"
#include "i2s.h"
#include "main.h"

// 只使用一个声道，但 IIS 有两个声道，
// 为节约内存，数据不区分声道，连续储存在内存中
// | Left CH sample 1 | Right CH sample 1 | Left CH sample 2 | Right CH sample 2 | ... |
// | MONO sample 1    | MONO sample 2     | MONO sample 3    | MONO sample 4     | ... |
// 实际音频采样率为 MKF360_AUDIO_SAMPLE_RATE_HZ，IIS 配置为音频采样率配置为 MKF360_AUDIO_SAMPLE_RATE_HZ/2

// TODO：每30MS会有几个样本丢失（值为0），待查明原因，现在 audio_io.c 中使用软件修复

__attribute__((section(".bss.DMA_RAM_D2"))) static int16_t
    iis_tx_dma_buffer[2][MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST];
__attribute__((section(".bss.DMA_RAM_D2"))) static int16_t
    iis_rx_dma_buffer[2][MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST];

__attribute__((section(".bss.DTCM"))) volatile static uint32_t idle_buffer;

void iis_start()
{
    HAL_StatusTypeDef ret_hal = HAL_I2SEx_TransmitReceive_DMA(
        &hi2s3, (uint16_t *)&iis_tx_dma_buffer[0][0], (uint16_t *)&iis_rx_dma_buffer[0][0],
        MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * 2);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
    idle_buffer = 1U;
}

void iis_stop()
{
    HAL_StatusTypeDef ret_hal = HAL_I2S_DMAStop(&hi2s3);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
}

int16_t *iis_get_tx_idle_buffer_address()
{
    return &iis_tx_dma_buffer[idle_buffer][0];
}

int16_t *iis_get_rx_idle_buffer_address()
{
    return &iis_rx_dma_buffer[idle_buffer][0];
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s == &hi2s3)
    {
        idle_buffer = 0U;
        event_group_set_event(EventGroup1, EventGroup1IisDmaBufferReady);
    }
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s == &hi2s3)
    {
        idle_buffer = 1U;
        event_group_set_event(EventGroup1, EventGroup1IisDmaBufferReady);
    }
}
