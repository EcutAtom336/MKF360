#include "User/audio_dac.h"

#include <stdio.h>

#include "arm_math.h"
#include "stm32h7xx_hal.h"

#include "MKF360_config.h"
#include "User/event_group.h"
#include "dac.h"
#include "main.h"
#include "tim.h"

#define DAC_START_FLAG (1U << 0U)
#define SPK_ENABLE_FLAG (1U << 1U)
#define HDST_ENABLE_FLAG (1U << 2U)

#define IS_DAC_STARTED() (flags & DAC_START_FLAG)
#define IS_SPK_ENABLE() (flags & SPK_ENABLE_FLAG)
#define IS_HDST_ENABLE() (flags & HDST_ENABLE_FLAG)

typedef struct
{
    uint16_t ch1;
    uint16_t ch2;
} DacFrame_t;

__attribute__((section(".DMA_RAM_D2")))
__attribute__((aligned(1024))) static uint32_t dac_dma_buffer[2][DAC_DMA_FRAME_SAMPLE_NUM];

__attribute__((section(".DTCM"))) static uint32_t flags;
__attribute__((section(".DTCM"))) static uint32_t idle_buffer;

static inline void dac_irq_handler(const uint8_t dma_frame_idx);

static void dac_start()
{
    arm_fill_q31((32768 << 16) + 32768, (int32_t *)&dac_dma_buffer[0][0], DAC_DMA_FRAME_SAMPLE_NUM * 2);
    HAL_StatusTypeDef ret_hal = HAL_DACEx_DualStart_DMA(&hdac1, DAC_CHANNEL_1, &dac_dma_buffer[0][0],
                                                        DAC_DMA_FRAME_SAMPLE_NUM * 2U, DAC_ALIGN_12B_L);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
    ret_hal = HAL_TIM_Base_Start(&htim7);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
    ATOMIC_SET_BIT(flags, DAC_START_FLAG);
    idle_buffer = 1U;
}

static void dac_stop()
{
    HAL_StatusTypeDef ret_hal = HAL_TIM_Base_Stop(&htim7);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
    ret_hal = HAL_DACEx_DualStop_DMA(&hdac1, DAC_CHANNEL_1);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
    ATOMIC_CLEAR_BIT(flags, DAC_START_FLAG);
}

void audio_dac_ctl(const AudioDacCmd_t cmd)
{
    if ((cmd == AudioDacCmdEnableCh1 || cmd == AudioDacCmdEnableCh2) && !IS_DAC_STARTED())
    {
        dac_start();
    }
    switch (cmd)
    {
    case AudioDacCmdEnableCh1: {
        ATOMIC_SET_BIT(flags, SPK_ENABLE_FLAG);
        break;
    }
    case AudioDacCmdDisableCh1: {
        ATOMIC_CLEAR_BIT(flags, SPK_ENABLE_FLAG);
        break;
    }
    case AudioDacCmdEnableCh2: {
        ATOMIC_SET_BIT(flags, HDST_ENABLE_FLAG);
        break;
    }
    case AudioDacCmdDisableCh2: {
        ATOMIC_CLEAR_BIT(flags, HDST_ENABLE_FLAG);
        break;
    }
    }
    if ((cmd == AudioDacCmdDisableCh1 || cmd == AudioDacCmdDisableCh2) && IS_DAC_STARTED())
    {
        dac_stop();
    }
}

void audio_dac_write_ch(const int16_t *data, const DacCh_t ch)
{
    if (ch == DacCh1)
    {
        for (size_t i = 0; i < DAC_DMA_FRAME_SAMPLE_NUM; ++i)
        {
            ((DacFrame_t *)&dac_dma_buffer[idle_buffer][i])->ch1 = (uint16_t)((int32_t)data[i] + 32768U);
        }
    }
    else if (ch == DacCh2)
    {
        for (size_t i = 0; i < DAC_DMA_FRAME_SAMPLE_NUM; ++i)
        {
            ((DacFrame_t *)&dac_dma_buffer[idle_buffer][i])->ch2 = (uint16_t)((int32_t)data[i] + 32768U);
        }
    }
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    (void)hdac;
    dac_irq_handler(0);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    (void)hdac;
    dac_irq_handler(1);
}

static inline void dac_irq_handler(const uint8_t dma_frame_idx)
{
    idle_buffer = dma_frame_idx == 0U ? 1U : 0U;
    arm_fill_q31((32768U << 16U) + 32768U, (q31_t *)&dac_dma_buffer[dma_frame_idx][0], DAC_DMA_FRAME_SAMPLE_NUM);
    bool before = event_group_set_event(EventGroup1, EventGroup1DacDmaBufferReady);
    if (before)
    {
        Error_Handler();
    }
}
