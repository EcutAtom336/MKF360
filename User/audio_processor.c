#include "User/audio_processor.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "acoustic_ec.h"
#include "arm_math.h"

#include "User/audio_buffer.h"
#include "audio/PCM_RES.h"
#include "main.h"

__attribute__((section(".bss.DTCM"))) static int16_t buffer1_1ms[MKF360_AUDIO_SAMPLE_NUM_1MS];
__attribute__((section(".bss.DTCM"))) static int16_t buffer2_1ms[MKF360_AUDIO_SAMPLE_NUM_1MS];
__attribute__((section(".bss.DTCM"))) static int16_t buffer3_1ms[MKF360_AUDIO_SAMPLE_NUM_1MS];

__attribute__((section(".bss.DTCM"))) static uint32_t aec_in_cnt;
__attribute__((section(".bss.DTCM"))) static uint32_t aec_run_cnt;

__attribute__((section(".DTCM"))) static float32_t gain = 1.0f;
__attribute__((section(".DTCM"))) static float32_t alpha = 0.1f;

__attribute__((section(".DTCM"))) static AcousticEC_Handler_t aec_handler = {
    .tail_length = 512,
    .preprocess_init = ACOUSTIC_EC_PREPROCESS_ENABLE,
    .ptr_primary_channels = 1,
    .ptr_reference_channels = 1,
    .ptr_output_channels = 1,
    .internal_memory_size = 0,
    .pInternalMemory = NULL,
};

static int32_t aec_init();

int32_t audio_processor_init()
{
    int32_t ret_int32 = 0;

    ret_int32 = aec_init();
    if (ret_int32 != 0)
    {
        printf("AEC init fail.\n");
        return -1;
    }

    return 0;
}

void audio_process()
{
    uint32_t ret_uint32 = 0;
    int32_t ret_int32 = 0;

    // 处理接口输入数据
    ret_int32 = interface_in_read(&buffer1_1ms[0], 1);
    if (ret_int32 == 1)
    {
        speaker_write(&buffer1_1ms[0], 1);
    }

    // 处理麦克风数据
    // 读取麦克风数据
    // 只使用了一个麦克风的数据
    ret_int32 = mic2_read(&buffer1_1ms[0], 1);
    ret_int32 = mic1_read(&buffer1_1ms[0], 1);

    if (ret_int32 == 1)
    {
        // 增益补偿
        arm_scale_q15(&buffer1_1ms[0], 16, 15, &buffer1_1ms[0], MKF360_AUDIO_SAMPLE_NUM_1MS);

        // AEC
        int16_t *const AEC_IN = &buffer1_1ms[0];
        int16_t *const AEC_REF = &buffer3_1ms[0];
        int16_t *const AEC_OUT = &buffer2_1ms[0];
        arm_fill_q15(0, AEC_REF, MKF360_AUDIO_SAMPLE_NUM_1MS);
        ret_uint32 = AcousticEC_Data_Input(AEC_IN, AEC_REF, AEC_OUT, &aec_handler);
        if (ret_uint32 == 1)
        {
            ret_uint32 = AcousticEC_Process(&aec_handler);
            if (ret_uint32 != 0)
            {
                printf("AEC process error, code: %u\n", ret_uint32);
            }
            if (++aec_run_cnt % 1000 == 0)
            {
                printf("AEC run cnt: %u\n", aec_run_cnt);
                printf("gain: %f\n", gain);
            }
        }
        if (++aec_in_cnt % 1000 == 0)
        {
            printf("AEC input cnt: %u\n", aec_in_cnt);
        }

        // AGC
        int16_t *const AGC_IN = AEC_OUT;
        int16_t *const AGC_OUT = AEC_REF;
        // 计算 RMS 和归一化 RMS
        int64_t power_sum = 0;
        arm_power_q15(AGC_IN, MKF360_AUDIO_SAMPLE_NUM_1MS, &power_sum);
        const float32_t SAMPLE_NUM_1MS = (float32_t)MKF360_AUDIO_SAMPLE_RATE_HZ / 1000;
        const float32_t POWER_MEAN = power_sum / SAMPLE_NUM_1MS;
        float32_t rms = 0.0F;
        arm_sqrt_f32(POWER_MEAN, &rms);
        const float32_t NORMALIZED_RMS = rms / 32768.0F;
        // 计算增益限值
        int16_t abs_max = 0;
        arm_absmax_no_idx_q15(AGC_IN, MKF360_AUDIO_SAMPLE_NUM_1MS, &abs_max);
        abs_max = abs(abs_max);
        const float32_t GAIN_LIMIT = (32768.0F * 0.2F) / abs_max;
        // 调整增益
        const float32_t TARGET_NORMALIZED_RMS = 1e-2;
        if (NORMALIZED_RMS > 1e-5)
        {
            float32_t adj = TARGET_NORMALIZED_RMS / NORMALIZED_RMS;
            gain = fmin(gain * (1 - alpha) + adj * alpha, fmin(GAIN_LIMIT, 50));
        }
        // 应用增益
        arm_scale_q15(AGC_IN, (q15_t)gain, 15, AGC_OUT, MKF360_AUDIO_SAMPLE_NUM_1MS);

        interface_out_write(AGC_OUT, 1);
    }
}

static int32_t aec_init()
{
    uint32_t ret_uint32 = 0;

    ret_uint32 = AcousticEC_getMemorySize(&aec_handler);
    if (ret_uint32 != 0)
    {
        printf("AEC get memory size fail, code: %u\n", ret_uint32);
        return -1;
    }
    printf("AEC memory size: %u bytes.\n", aec_handler.internal_memory_size);
    aec_handler.pInternalMemory = malloc(aec_handler.internal_memory_size);
    if (aec_handler.pInternalMemory == NULL)
    {
        printf("malloc AEC memory fail.\n");
        return -1;
    }
    ret_uint32 = AcousticEC_Init(&aec_handler);
    if (ret_uint32 != 0)
    {
        printf("AEC init fail, code: %u\n", ret_uint32);
        return -1;
    }
    AcousticEC_Config_t aec_config = {
        .preprocess_state = ACOUSTIC_EC_PREPROCESS_ENABLE,
        .AGC_value = 0,
        .residual_echo_remove = 1,     // Default: 1
        .noise_suppress_default = -20, // Default: -15
        .echo_suppress_default = -40,  // Default: -40
        .echo_suppress_active = -15,   // Default: -15
    };
    ret_uint32 = AcousticEC_setConfig(&aec_handler, &aec_config);
    if (ret_uint32 != 0)
    {
        printf("AEC set config fail, code: %u\n", ret_uint32);
        return -1;
    }

    return 0;
}