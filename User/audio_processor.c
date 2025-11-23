#include "User/audio_processor.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "arm_math.h"
#include "speex/speex_echo.h"
#include "speex/speex_preprocess.h"

#include "User/audio_buffer.h"
#include "audio/PCM_RES.h"
#include "main.h"

#define PROCESS_FRAME_MS (10U)

__attribute__((section(".bss.DTCM"))) static int16_t buffer1[MKF360_AUDIO_SAMPLE_NUM_1MS * PROCESS_FRAME_MS];
__attribute__((section(".bss.DTCM"))) static int16_t buffer2[MKF360_AUDIO_SAMPLE_NUM_1MS * PROCESS_FRAME_MS];
__attribute__((section(".bss.DTCM"))) static int16_t buffer3[MKF360_AUDIO_SAMPLE_NUM_1MS * PROCESS_FRAME_MS];

uint32_t last_log_tick;

SpeexPreprocessState *speex_preprocess_state;
SpeexEchoState *speex_echo_state;

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
    // 降噪算法初始化
    speex_preprocess_state = speex_preprocess_state_init(MKF360_AUDIO_SAMPLE_NUM_1MS * PROCESS_FRAME_MS, 48000);
    if (speex_preprocess_state == NULL)
    {
        return -1;
    }
    int ret_int = 0;
    int i = 0;
    float f = 0.0F;

    // i = 1;
    // ret_int = speex_preprocess_ctl(speex_preprocess_state, SPEEX_PREPROCESS_SET_DENOISE, &i);
    // if (ret_int != 0)
    // {
    //     return -2;
    // }

    // i = 1;
    // ret_int = speex_preprocess_ctl(speex_preprocess_state, SPEEX_PREPROCESS_SET_AGC, &i);
    // if (ret_int != 0)
    // {
    //     return -3;
    // }

    // i = 16384;
    // ret_int = speex_preprocess_ctl(speex_preprocess_state, SPEEX_PREPROCESS_SET_AGC_LEVEL, &i);
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

    // 回声消除算法初始化
    speex_echo_state = speex_echo_state_init(MKF360_AUDIO_SAMPLE_NUM_1MS * PROCESS_FRAME_MS, 1024);
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

    // 处理接口输入数据
    ret_int = interface_in_read(&buffer1[0], PROCESS_FRAME_MS);
    if (ret_int != 0)
    {
        return;
    }
    ret_int = speaker_write(&buffer1[0], PROCESS_FRAME_MS);
    if (ret_int == 1)
    {
        printf("Speaker data overwrite.\n");
    }
}

static void process_capture_audio()
{
    int ret_int = 0;

    // 处理麦克风数据
    // 读取麦克风数据
    // 只使用了一个麦克风的数据
    uint32_t mic1_timestamp = 0;
    uint32_t mic2_timestamp = 0;
    ret_int = mic2_read(&buffer1[0], PROCESS_FRAME_MS, &mic2_timestamp);
    ret_int = mic1_read(&buffer1[0], PROCESS_FRAME_MS, &mic1_timestamp);
    if (ret_int != 0)
    {
        return;
    }

    // 回声消除
    // DAC 有个两个空帧？
    ret_int = feedback_read(&buffer2[0], mic1_timestamp + MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * 2, PROCESS_FRAME_MS);
    if (ret_int != 0)
    {
        arm_fill_q15(0, &buffer2[0], MKF360_AUDIO_SAMPLE_NUM_1MS * PROCESS_FRAME_MS);
        printf("Read feedback data fail, code: %d\n", ret_int);
    }
    speex_echo_cancellation(speex_echo_state, &buffer1[0], &buffer2[0], &buffer3[0]);

    // 预处理
    ret_int = speex_preprocess_run(speex_preprocess_state, &buffer3[0]);
    if (ret_int == 0) // No speech active
    {
        // printf("No speech active.\n");
        // arm_fill_q15(0, &buffer1_1ms[0], MKF360_AUDIO_SAMPLE_NUM_1MS * PROCESS_FRAME_MS);
    }
    else if (ret_int == 1) // Speech active
    {
        // printf("Speech active.\n");
    }

    interface_out_write(&buffer3[0], PROCESS_FRAME_MS);
}
