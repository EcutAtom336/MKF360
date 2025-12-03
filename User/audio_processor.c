#include "User/audio_processor.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "arm_math.h"
#include "dsp/basic_math_functions.h"
#include "dsp/statistics_functions.h"
#include "lwrb/lwrb.h"
#include "speex/speex_echo.h"
#include "speex/speex_preprocess.h"

#include "User/audio_buffer.h"
#include "User/share_buffer.h"
#include "audio/PCM_RES.h"
#include "main.h"

#define AUDIO_PROCESS_DEBUG (1)
#define PROCESS_FRAME_SAMPLES (128)
#define FEEDBACK_DELAY_SAMPLES (1970U)

__attribute__((section(".bss.DTCM"))) SpeexPreprocessState *speex_preprocess_state_feedback;
__attribute__((section(".bss.DTCM"))) SpeexPreprocessState *speex_preprocess_state;
__attribute__((section(".bss.DTCM"))) SpeexEchoState *speex_echo_state;

__attribute__((section(".bss.DTCM"))) uint32_t last_log_tick;
__attribute__((section(".bss.DTCM"))) bool aligned;

static int32_t speexdsp_init();
static void process_interface_input_audio();
static void process_capture_audio();

int32_t audio_processor_init()
{
    int32_t ret_int32 = 0;

    ret_int32 = speexdsp_init();
    if (ret_int32 != 0)
    {
        printf("SpeexDSP init fail, code: %d\n", ret_int32);
        return -1;
    }

    return 0;
}

void audio_processor_reset()
{
    aligned = false;
    speex_echo_state_reset(speex_echo_state);
}

void audio_process()
{
    process_interface_input_audio();
    process_capture_audio();
    uint32_t tick = HAL_GetTick();
    if (tick - last_log_tick > 1000)
    {
        printf("Audio processor running.\n");
        last_log_tick = tick;
    }
}

static int32_t speexdsp_init()
{
    int ret_int = 0;
    int i = 0;
    float f = 0.0F;

    // 降噪算法初始化
    speex_preprocess_state = speex_preprocess_state_init(PROCESS_FRAME_SAMPLES, 48000);
    if (speex_preprocess_state == NULL)
    {
        return -1;
    }
    // i = 1;
    // ret_int = speex_preprocess_ctl(speex_preprocess_state, SPEEX_PREPROCESS_SET_DENOISE, &i);
    // if (ret_int != 0)
    // {
    //     return -2;
    // }
    i = 1;
    ret_int = speex_preprocess_ctl(speex_preprocess_state, SPEEX_PREPROCESS_SET_AGC, &i);
    if (ret_int != 0)
    {
        return -3;
    }
    ret_int = speex_preprocess_ctl(speex_preprocess_state, SPEEX_PREPROCESS_SET_ECHO_STATE, speex_echo_state);
    if (ret_int != 0)
    {
        return -3;
    }
    // f = 16384.0F;
    // ret_int = speex_preprocess_ctl(speex_preprocess_state, SPEEX_PREPROCESS_SET_AGC_LEVEL, &f);
    // if (ret_int != 0)
    // {
    //     return -4;
    // }
    // i = 0;
    // ret_int = speex_preprocess_ctl(speex_preprocess_state, SPEEX_PREPROCESS_SET_DEREVERB, &i);
    // if (ret_int != 0)
    // {
    //     return -5;
    // }
    // f = .0;
    // ret_int = speex_preprocess_ctl(speex_preprocess_state, SPEEX_PREPROCESS_SET_DEREVERB_DECAY, &f);
    // if (ret_int != 0)
    // {
    //     return -6;
    // }
    // f = .0;
    // ret_int = speex_preprocess_ctl(speex_preprocess_state, SPEEX_PREPROCESS_SET_DEREVERB_LEVEL, &f);
    // if (ret_int != 0)
    // {
    //     return -7;
    // }
    // i = 30;
    // ret_int = speex_preprocess_ctl(speex_preprocess_state, SPEEX_PREPROCESS_SET_AGC_MAX_GAIN, &i);
    // if (ret_int != 0)
    // {
    //     return -8;
    // }

    speex_preprocess_state_feedback = speex_preprocess_state_init(PROCESS_FRAME_SAMPLES, 48000);
    if (speex_preprocess_state_feedback == NULL)
    {
        return -2;
    }
    i = 1;
    ret_int = speex_preprocess_ctl(speex_preprocess_state_feedback, SPEEX_PREPROCESS_SET_AGC, &i);
    if (ret_int != 0)
    {
        return -3;
    }

    // 回声消除算法初始化
    speex_echo_state = speex_echo_state_init(PROCESS_FRAME_SAMPLES, 1024);
    if (speex_echo_state == NULL)
    {
        return -9;
    }
    speex_echo_state_reset(speex_echo_state);
    i = 48000;
    ret_int = speex_echo_ctl(speex_echo_state, SPEEX_ECHO_SET_SAMPLING_RATE, &i);
    if (ret_int != 0)
    {
        return -10;
    }

    return 0;
}

static void process_interface_input_audio()
{
    int ret_int = 0;

    int16_t *buffer1 = &shared_buffer[0];

    // 处理接口输入数据
    ret_int = interface_in_read(&buffer1[0], PROCESS_FRAME_SAMPLES);
    if (ret_int != 0)
    {
        return;
    }
    ret_int = speaker_write(&buffer1[0], PROCESS_FRAME_SAMPLES);
    if (ret_int == 1)
    {
        printf("Speaker data overwrite.\n");
    }
}

static void process_capture_audio()
{
    int16_t *buffer1 = &shared_buffer[0];
    int16_t *buffer2 = &shared_buffer[PROCESS_FRAME_SAMPLES];
    int16_t *buffer3 = &shared_buffer[PROCESS_FRAME_SAMPLES * 2];
    int16_t *buffer4 = &shared_buffer[PROCESS_FRAME_SAMPLES * 3];

    // 处理麦克风数据
    // 读取麦克风数据
    // 只使用了一个麦克风的数据
    if (mic1_get_sample_num() < PROCESS_FRAME_SAMPLES || mic2_get_sample_num() < PROCESS_FRAME_SAMPLES ||
        feedback_get_sample_num() < PROCESS_FRAME_SAMPLES)
    {
        return;
    }

    mic2_read(&buffer1[0], PROCESS_FRAME_SAMPLES);
    mic1_read(&buffer1[0], PROCESS_FRAME_SAMPLES);

    if (aligned == false && feedback_get_sample_num() >= FEEDBACK_DELAY_SAMPLES + PROCESS_FRAME_SAMPLES)
    {
        aligned = true;
    }

    if (aligned == true)
    {
        feedback_read(buffer2, PROCESS_FRAME_SAMPLES);
    }
    else
    {
        memset(buffer2, 0, PROCESS_FRAME_SAMPLES * 2);
    }

    int16_t mean = 0;
    arm_mean_q15(buffer1, PROCESS_FRAME_SAMPLES, &mean);
    arm_offset_q15(buffer1, -mean, buffer1, PROCESS_FRAME_SAMPLES);

    // speex_preprocess_run(speex_preprocess_state_feedback, buffer2);
    speex_echo_cancellation(speex_echo_state, buffer1, buffer2, buffer3);
    speex_preprocess_run(speex_preprocess_state, buffer3);

#if AUDIO_PROCESS_DEBUG == 1
#pragma unroll PROCESS_FRAME_SAMPLES
    for (size_t i = 0; i < PROCESS_FRAME_SAMPLES; i++)
    {
        buffer4[i * 2 + 0] = buffer3[i];
        buffer4[i * 2 + 1] = buffer2[i];
    }
#else
#pragma unroll PROCESS_FRAME_SAMPLES
    for (size_t i = 0; i < PROCESS_FRAME_SAMPLES; i++)
    {
        buffer4[i * 2 + 0] = buffer3[i];
        buffer4[i * 2 + 1] = buffer3[i];
    }
#endif
    interface_out_write(&buffer4[0], PROCESS_FRAME_SAMPLES);
}
