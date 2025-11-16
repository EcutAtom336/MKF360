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

__attribute__((section(".bss.DTCM"))) static int16_t buffer1[MKF360_AUDIO_SAMPLE_NUM_1MS * 4];
__attribute__((section(".bss.DTCM"))) static int16_t buffer2[MKF360_AUDIO_SAMPLE_NUM_1MS];
__attribute__((section(".bss.DTCM"))) static int16_t buffer3[MKF360_AUDIO_SAMPLE_NUM_1MS];

__attribute__((section(".DTCM"))) static int16_t *speaker_read_buffer = &buffer1[0];

__attribute__((section(".bss.DTCM"))) static uint32_t aec_in_cnt;
__attribute__((section(".bss.DTCM"))) static uint32_t aec_run_cnt;
__attribute__((section(".DTCM"))) static int16_t *interlaced_mic_data = &buffer1[0];
__attribute__((section(".DTCM"))) static int16_t *buffer1_1ms = &buffer2[0];
__attribute__((section(".DTCM"))) static int16_t *buffer2_1ms = &buffer3[0];

__attribute__((section(".DTCM"))) static float32_t gain = 1.0f;
__attribute__((section(".DTCM"))) static float32_t alpha = 0.01f;

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
    ret_int32 = interface_in_read(speaker_read_buffer, MKF360_AUDIO_SAMPLE_NUM_1MS);
    if (ret_int32 == MKF360_AUDIO_SAMPLE_NUM_1MS)
    {
        speaker_write(speaker_read_buffer, MKF360_AUDIO_SAMPLE_NUM_1MS);
    }

    // 处理麦克风数据
    // 读取麦克风数据
    // 只使用了一个麦克风的数据
    ret_int32 = mic_read(&interlaced_mic_data[0], MKF360_AUDIO_SAMPLE_NUM_1MS * 4);

    if (ret_int32 == MKF360_AUDIO_SAMPLE_NUM_1MS * 4)
    {
        for (size_t i = 0; i < MKF360_AUDIO_SAMPLE_NUM_1MS; ++i)
        {
            interlaced_mic_data[i] = interlaced_mic_data[i * 4];
        }

        // AGC
        int64_t power_sum = 0;
        arm_power_q15(&interlaced_mic_data[0], MKF360_AUDIO_SAMPLE_NUM_1MS, &power_sum);
        float32_t rms = 0.0F;
        const float32_t SAMPLE_NUM_1MS = (float32_t)MKF360_AUDIO_SAMPLE_RATE_HZ / 1000;
        arm_sqrt_f32((float32_t)power_sum / SAMPLE_NUM_1MS, &rms);
        rms /= 32768.0f;
        const float32_t TARGET_RMS = 0.1f;
        if (rms > 1e-6)
        {
            float32_t adj = TARGET_RMS / rms;
            gain = gain * (1 - alpha) + adj * alpha;
        }
        arm_scale_q15(&interlaced_mic_data[0], (q15_t)gain, 15, &interlaced_mic_data[0], MKF360_AUDIO_SAMPLE_NUM_1MS);

        // AEC
        arm_fill_q15(0, &buffer1_1ms[0], MKF360_AUDIO_SAMPLE_NUM_1MS);
        ret_uint32 = AcousticEC_Data_Input(&interlaced_mic_data[0], &buffer1_1ms[0], &buffer2_1ms[0], &aec_handler);
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
            }
        }
        if (++aec_in_cnt % 1000 == 0)
        {
            printf("AEC input cnt: %u\n", aec_in_cnt);
        }

        interface_out_write(&buffer2_1ms[0], MKF360_AUDIO_SAMPLE_NUM_1MS);
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
        .noise_suppress_default = -15, // Default: -15
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