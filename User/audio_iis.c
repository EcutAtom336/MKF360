#include "User/audio_iis.h"

#include "arm_math.h"

#include "stm32h7xx_hal.h"

#include "MKF360_config.h"
#include "User/event_group.h"
#include "i2s.h"
#include "main.h"

__attribute__((section(".bss.DMA_RAM_D2"))) static int16_t iis_tx_dma_buffer[2][IIS_DMA_FRAME_SAMPLE_NUM];
__attribute__((section(".bss.DMA_RAM_D2"))) static int16_t iis_rx_dma_buffer[2][IIS_DMA_FRAME_SAMPLE_NUM];

__attribute__((section(".bss.DTCM"))) static uint32_t idle_buffer;

void iis_start()
{
    HAL_StatusTypeDef ret_hal =
        HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (uint16_t *)&iis_tx_dma_buffer[0][0],
                                      (uint16_t *)&iis_rx_dma_buffer[0][0], IIS_DMA_FRAME_SAMPLE_NUM * 2);
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

void iis_tx_write(const int16_t *buffer)
{
    arm_copy_q15(buffer, &iis_tx_dma_buffer[idle_buffer][0], IIS_DMA_FRAME_SAMPLE_NUM);
}

void iis_rx_read(int16_t *buffer)
{
    arm_copy_q15(&iis_rx_dma_buffer[idle_buffer][0], buffer, IIS_DMA_FRAME_SAMPLE_NUM);
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s == &hi2s3)
    {
        idle_buffer = 0U;
        bool before = event_group_set_event(EventGroup1, EventGroup1IisDmaBufferReady);
        if (before)
        {
            Error_Handler();
        }
    }
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s == &hi2s3)
    {
        idle_buffer = 1U;
        bool before = event_group_set_event(EventGroup1, EventGroup1IisDmaBufferReady);
        if (before)
        {
            Error_Handler();
        }
    }
}
