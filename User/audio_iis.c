#include "User/audio_iis.h"

#include <stdbool.h>

#include "arm_math.h"

#include "stm32h7xx_hal.h"

#include "User/event_group.h"
#include "i2s.h"
#include "main.h"

__attribute__((
    section(".bss.DMA_RAM_D2"))) static int16_t iis_tx_dma_buffer[2][MKF360_AUDIO_PERIPH_DMA_DEST_SAMPLE_NUM];
__attribute__((
    section(".bss.DMA_RAM_D2"))) static int16_t iis_rx_dma_buffer[2][MKF360_AUDIO_PERIPH_DMA_DEST_SAMPLE_NUM];

__attribute__((section(".bss.DTCM"))) volatile static uint32_t idle_buffer;

void iis_start()
{
    HAL_StatusTypeDef ret_hal = HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (uint16_t *)&iis_tx_dma_buffer[0][0],
                                                              (uint16_t *)&iis_rx_dma_buffer[0][0],
                                                              MKF360_AUDIO_PERIPH_DMA_DEST_SAMPLE_NUM * 2);
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

void iis_tx_write(const int16_t *buffer)
{
    arm_copy_q15(buffer, &iis_tx_dma_buffer[idle_buffer][0], MKF360_AUDIO_PERIPH_DMA_DEST_SAMPLE_NUM);
}

void iis_rx_read(int16_t *buffer)
{
    arm_copy_q15(&iis_rx_dma_buffer[idle_buffer][0], buffer, MKF360_AUDIO_PERIPH_DMA_DEST_SAMPLE_NUM);
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
