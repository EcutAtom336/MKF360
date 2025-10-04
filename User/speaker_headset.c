#include "User/speaker_headset.h"

#include <stdio.h>

#include "arm_math.h"
#include "stm32h7xx_hal.h"

#include "MKF360_config.h"
#include "dac.h"
#include "dma.h"
#include "main.h"
#include "tim.h"

#define INIT_FLAG (1U << 0U)
#define DAC_START_FLAG (1U << 1U)
#define SPK_ENABLE_FLAG (1U << 2U)
#define HDST_ENABLE_FLAG (1U << 3U)

#define IS_INIT() (flag & INIT_FLAG)
#define IS_DAC_STARTED() (flags & DAC_START_FLAG)
#define IS_SPK_ENABLE() (flags & SPK_ENABLE_FLAG)
#define IS_HDST_ENABLE() (flags & HDST_ENABLE_FLAG)

typedef struct
{
    uint16_t ch1;
    uint16_t ch2;
} DacFrame_t;

__attribute__((section(".bss.DMA_RAM_D2")))
__attribute__((aligned(1024))) uint32_t dac_dma_buffer[2][DAC_DMA_FRAME_SAMPLE_NUM];
__attribute__((section(".bss.DTCM"))) uint8_t dac_idle_buffer_idx;
__attribute__((section(".bss.DTCM"))) uint16_t dac_idle_buffer_spk_full;
__attribute__((section(".bss.DTCM"))) uint16_t dac_idle_buffer_hdst_full;

__attribute__((section(".bss.DTCM"))) DacBufferResetCallback dac_buffer_reset_callback;

__attribute__((section(".bss.DTCM"))) uint32_t flags;

void spk_hdst_init()
{
    flags |= INIT_FLAG;
}

static void dac_start()
{
    arm_fill_q31((32768 << 16) + 32768, (int32_t *)&dac_dma_buffer[0][0], DAC_DMA_FRAME_SAMPLE_NUM * 2);
    dac_idle_buffer_idx = 1U;
    dac_idle_buffer_hdst_full = 0U;
    dac_idle_buffer_spk_full = 0U;
    HAL_StatusTypeDef ret_hal = HAL_DACEx_DualStart_DMA(&hdac1, DAC_CHANNEL_1, &dac_dma_buffer[0][0],
                                                        DAC_DMA_FRAME_SAMPLE_NUM * 2U, DAC_ALIGN_12B_L);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
    ret_hal = HAL_TIM_Base_Start(&htim6);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }
    ATOMIC_SET_BIT(flags, DAC_START_FLAG);
}

static void dac_stop()
{
    HAL_StatusTypeDef ret_hal = HAL_TIM_Base_Stop(&htim6);
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

void register_dac_buffer_reset_callback(DacBufferResetCallback callback)
{
    dac_buffer_reset_callback = callback;
}

void spk_hdst_ctl(const SpkHdstCmd_t cmd, SpkHdstCtlData_t *data)
{
    if ((cmd == SpkHdstCmdEnableSpk || cmd == SpkHdstCmdEnableHdst) && !IS_DAC_STARTED())
    {
        dac_start();
    }
    switch (cmd)
    {
    case SpkHdstCmdEnableSpk: {
        // HAL_GPIO_WritePin(SPEAKER_EN_GPIO_Port, SPEAKER_EN_Pin, GPIO_PIN_SET);
        ATOMIC_SET_BIT(flags, SPK_ENABLE_FLAG);
        break;
    }
    case SpkHdstCmdDisableSpk: {
        // HAL_GPIO_WritePin(SPEAKER_EN_GPIO_Port, SPEAKER_EN_Pin, GPIO_PIN_RESET);
        ATOMIC_CLEAR_BIT(flags, SPK_ENABLE_FLAG);
        break;
    }
    case SpkHdstCmdEnableHdst: {
        // HAL_GPIO_WritePin(HEADSET_OUT_EN_GPIO_Port, HEADSET_OUT_EN_Pin, GPIO_PIN_SET);
        ATOMIC_SET_BIT(flags, HDST_ENABLE_FLAG);
        break;
    }
    case SpkHdstCmdDisableHdst: {
        // HAL_GPIO_WritePin(HEADSET_OUT_EN_GPIO_Port, HEADSET_OUT_EN_Pin, GPIO_PIN_RESET);
        ATOMIC_CLEAR_BIT(flags, HDST_ENABLE_FLAG);
        break;
    }
    case SpkHdstCmdSpkPlay: {
        for (size_t i = 0; i < data->spk_play.sample_num; i++)
        {
            if (dac_idle_buffer_spk_full >= DAC_DMA_FRAME_SAMPLE_NUM)
            {
                return;
            }
            ((DacFrame_t *)&dac_dma_buffer[dac_idle_buffer_idx][dac_idle_buffer_spk_full++])->ch1 =
                (uint16_t)(((uint32_t)data->spk_play.data[i]) + 32768);
        }
        break;
    }
    case SpkHdstCmdHdstPlay: {
        for (size_t i = 0; i < data->hdst_play.sample_num; i++)
        {
            if (dac_idle_buffer_hdst_full >= DAC_DMA_FRAME_SAMPLE_NUM)
            {
                return;
            }
            ((DacFrame_t *)&dac_dma_buffer[dac_idle_buffer_idx][dac_idle_buffer_hdst_full++])->ch2 =
                (uint16_t)(((uint32_t)data->hdst_play.data[i]) + 32768);
        }
        break;
    }
    }
    if ((cmd == SpkHdstCmdDisableSpk || cmd == SpkHdstCmdDisableHdst) && IS_DAC_STARTED())
    {
        dac_stop();
    }
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    (void)hdac;
    arm_fill_q31((32768U << 16U) + 32768U, (q31_t *)&dac_dma_buffer[0][0], DAC_DMA_FRAME_SAMPLE_NUM);
    dac_idle_buffer_idx = 0U;
    dac_idle_buffer_spk_full = 0U;
    dac_idle_buffer_hdst_full = 0U;
    if (dac_buffer_reset_callback != NULL)
    {
        dac_buffer_reset_callback();
    }
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    (void)hdac;
    arm_fill_q31((32768U << 16U) + 32768U, (q31_t *)&dac_dma_buffer[1][0], DAC_DMA_FRAME_SAMPLE_NUM);
    dac_idle_buffer_idx = 1U;
    dac_idle_buffer_spk_full = 0U;
    dac_idle_buffer_hdst_full = 0U;
    if (dac_buffer_reset_callback != NULL)
    {
        dac_buffer_reset_callback();
    }
}
