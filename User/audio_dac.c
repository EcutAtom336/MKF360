#include "User/audio_dac.h"

#include <stdio.h>

#include "arm_math.h"
#include "stm32h7xx_hal.h"

#include "User/event_group.h"
#include "dac.h"
#include "main.h"
#include "tim.h"

#define DAC_STARTED_FLAG (1U << 0U)
#define CH1_ENABLED_FLAG (1U << 1U)
#define CH2_ENABLED_FLAG (1U << 2U)

#define IS_DAC_STARTED() (flags & DAC_STARTED_FLAG)
#define IS_CH1_ENABLED() (flags & CH1_ENABLED_FLAG)
#define IS_CH2_ENABLED() (flags & CH2_ENABLED_FLAG)

typedef struct
{
    uint16_t ch1;
    uint16_t ch2;
} DacFrame_t;

__attribute__((section(".bss.DMA_RAM_D2"))) __attribute__((aligned(
    1024))) static uint32_t dac_dma_buffer[2][MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST];

__attribute__((section(".bss.DTCM"))) static uint32_t flags;
__attribute__((section(".bss.DTCM"))) volatile static uint32_t idle_buffer;
__attribute__((section(".bss.DTCM"))) volatile static uint32_t send_complete_timestamp;

static void dac_start()
{
    arm_fill_q31((32768 << 16) + 32768, (int32_t *)&dac_dma_buffer[0][0],
                 MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * 2);
    HAL_StatusTypeDef ret_hal = HAL_DACEx_DualStart_DMA(
        &hdac1, DAC_CHANNEL_1, &dac_dma_buffer[0][0],
        MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * 2U, DAC_ALIGN_12B_L);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
    ret_hal = HAL_TIM_Base_Start(&AUDIO_DAC_TRIG_TIM);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
    ATOMIC_SET_BIT(flags, DAC_STARTED_FLAG);
    idle_buffer = 1U;
}

static void dac_stop()
{
    HAL_StatusTypeDef ret_hal = HAL_TIM_Base_Stop(&AUDIO_DAC_TRIG_TIM);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
    ret_hal = HAL_DACEx_DualStop_DMA(&hdac1, DAC_CHANNEL_1);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
    ATOMIC_CLEAR_BIT(flags, DAC_STARTED_FLAG);
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
        ATOMIC_SET_BIT(flags, CH1_ENABLED_FLAG);
        break;
    }
    case AudioDacCmdDisableCh1: {
        ATOMIC_CLEAR_BIT(flags, CH1_ENABLED_FLAG);
        break;
    }
    case AudioDacCmdEnableCh2: {
        ATOMIC_SET_BIT(flags, CH2_ENABLED_FLAG);
        break;
    }
    case AudioDacCmdDisableCh2: {
        ATOMIC_CLEAR_BIT(flags, CH2_ENABLED_FLAG);
        break;
    }
    }
    if (!IS_CH1_ENABLED() && !IS_CH2_ENABLED() && IS_DAC_STARTED())
    {
        dac_stop();
    }
}

void audio_dac_write_ch(const int16_t *data, const DacCh_t ch)
{
    if (ch == DacCh1)
    {
        if (!IS_CH1_ENABLED())
        {
            printf("dac ch1 not enable.");
            return;
        }
        for (size_t i = 0; i < MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST; ++i)
        {
            ((DacFrame_t *)&dac_dma_buffer[idle_buffer][i])->ch1 = (uint16_t)((int32_t)data[i] + 32768U);
        }
    }
    else if (ch == DacCh2)
    {
        if (!IS_CH2_ENABLED())
        {
            printf("dac ch2 not enable.");
            return;
        }
        for (size_t i = 0; i < MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST; ++i)
        {
            ((DacFrame_t *)&dac_dma_buffer[idle_buffer][i])->ch2 = (uint16_t)((int32_t)data[i] + 32768U);
        }
    }
}

void audio_dac_read_ch(int16_t *const data, const DacCh_t ch)
{
    if (ch == DacCh1)
    {
        if (!IS_CH1_ENABLED())
        {
            printf("dac ch1 not enable.\n");
            return;
        }
        for (size_t i = 0; i < MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST; ++i)
        {
            int32_t origin = ((DacFrame_t *)&dac_dma_buffer[idle_buffer][i])->ch1;
            data[i] = origin - 32768U;
        }
    }
    else if (ch == DacCh2)
    {
        if (!IS_CH2_ENABLED())
        {
            printf("dac ch2 not enable.\n");
            return;
        }
        for (size_t i = 0; i < MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST; ++i)
        {
            int32_t origin = ((DacFrame_t *)&dac_dma_buffer[idle_buffer][i])->ch2;
            data[i] = origin - 32768U;
        }
    }
}

uint32_t audio_dac_get_send_complete_timestamp()
{
    return send_complete_timestamp;
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    if (hdac == &hdac1)
    {
        // arm_fill_q31((32768U << 16U) + 32768U, (q31_t *)&dac_dma_buffer[0][0],
        //              MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST);
        idle_buffer = 0;
        send_complete_timestamp = HAL_GetTick();
        if (IS_CH1_ENABLED())
        {
            event_group_set_event(EventGroup1, EventGroup1DacCh1DmaBufferReady);
        }
        if (IS_CH2_ENABLED())
        {
            event_group_set_event(EventGroup1, EventGroup1DacCh2DmaBufferReady);
        }
    }
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    if (hdac == &hdac1)
    {
        // arm_fill_q31((32768U << 16U) + 32768U, (q31_t *)&dac_dma_buffer[1][0],
        //              MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST);
        idle_buffer = 1;
        send_complete_timestamp = HAL_GetTick();
        if (IS_CH1_ENABLED())
        {
            event_group_set_event(EventGroup1, EventGroup1DacCh1DmaBufferReady);
        }
        if (IS_CH2_ENABLED())
        {
            event_group_set_event(EventGroup1, EventGroup1DacCh2DmaBufferReady);
        }
    }
}
