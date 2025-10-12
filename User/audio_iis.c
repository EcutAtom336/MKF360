#include "User/audio_iis.h"

#include "stm32h7xx_hal.h"

#include "MKF360_config.h"
#include "i2s.h"
#include "main.h"

__attribute__((section(".bss.DMA_RAM_D2"))) static int16_t iis_tx_dma_buffer[2][IIS_DMA_FRAME_SAMPLE_NUM];
__attribute__((section(".bss.DMA_RAM_D2"))) static int16_t iis_rx_dma_buffer[2][IIS_DMA_FRAME_SAMPLE_NUM];

__attribute__((section(".bss.DTCM"))) static OnIisDmaBufferReadyIsrCallback_t isr_callback;

void audio_iis_register_on_dma_buffer_ready_isr_callback(const OnIisDmaBufferReadyIsrCallback_t isr_cb)
{
    isr_callback = isr_cb;
}

void iis_start()
{
    HAL_StatusTypeDef ret_hal =
        HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (uint16_t *)&iis_tx_dma_buffer[0][0],
                                      (uint16_t *)&iis_rx_dma_buffer[0][0], IIS_DMA_FRAME_SAMPLE_NUM * 2);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
}

void iis_stop()
{
    HAL_StatusTypeDef ret_hal = HAL_I2S_DMAStop(&hi2s3);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s == &hi2s3)
    {
        if (isr_callback != NULL)
        {
            isr_callback(&iis_rx_dma_buffer[0][0], &iis_tx_dma_buffer[0][0], IIS_DMA_FRAME_SAMPLE_NUM);
        }
    }
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s == &hi2s3)
    {
        if (isr_callback != NULL)
        {
            isr_callback(&iis_rx_dma_buffer[1][0], &iis_tx_dma_buffer[1][0], IIS_DMA_FRAME_SAMPLE_NUM);
        }
    }
}
