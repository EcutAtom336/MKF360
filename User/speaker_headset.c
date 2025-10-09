#include "User/speaker_headset.h"

#include <stdio.h>

#include "arm_math.h"
#include "stm32h7xx_hal.h"

#include "MKF360_config.h"
#include "dac.h"
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

__attribute__((section(".DMA_RAM_D2")))
__attribute__((aligned(1024))) uint32_t dac_dma_buffer[2][DAC_DMA_FRAME_SAMPLE_NUM];

__attribute__((section(".DTCM"))) OnDacDmaFrameEmptyIsrCallback isr_callback;

__attribute__((section(".DTCM"))) uint32_t flags;

static inline void dac_irq_handler(const uint8_t dma_frame_idx);

void spk_hdst_init(const OnDacDmaFrameEmptyIsrCallback isr_cb)
{
    if (isr_cb == NULL)
    {
        Error_Handler();
    }
    isr_callback = isr_cb;

    flags |= INIT_FLAG;
}

static void dac_start()
{
    arm_fill_q31((32768 << 16) + 32768, (int32_t *)&dac_dma_buffer[0][0], DAC_DMA_FRAME_SAMPLE_NUM * 2);
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

void spk_hdst_ctl(const SpkHdstCmd_t cmd)
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
    }
    if ((cmd == SpkHdstCmdDisableSpk || cmd == SpkHdstCmdDisableHdst) && IS_DAC_STARTED())
    {
        dac_stop();
    }
}

void audio_dac_util_write_speaker(void *frame, const int16_t *data, const size_t sample_num)
{
    for (size_t i = 0; i < sample_num; ++i)
    {
        ((DacFrame_t *)frame)[i].ch1 = (uint16_t)((int32_t)data[i] + 32768U);
    }
}

void audio_dac_util_write_headset(void *frame, const int16_t *data, const size_t sample_num)
{
    for (size_t i = 0; i < sample_num; ++i)
    {
        ((DacFrame_t *)frame)[i].ch2 = (uint16_t)((int32_t)data[i] + 32768U);
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
    arm_fill_q31((32768U << 16U) + 32768U, (q31_t *)&dac_dma_buffer[dma_frame_idx][0], DAC_DMA_FRAME_SAMPLE_NUM);
    if (IS_SPK_ENABLE())
    {
        isr_callback(&dac_dma_buffer[dma_frame_idx][0], DAC_DMA_FRAME_SAMPLE_NUM);
    }
    if (IS_HDST_ENABLE())
    {
        isr_callback(&dac_dma_buffer[dma_frame_idx][0], DAC_DMA_FRAME_SAMPLE_NUM);
    }
}
