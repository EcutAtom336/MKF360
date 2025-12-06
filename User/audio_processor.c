#include "User/audio_processor.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "arm_math.h"
#include "speex/speex_echo.h"
#include "speex/speex_preprocess.h"

#include "User/audio_buffer.h"
#include "User/share_buffer.h"
#include "audio/PCM_RES.h"
#include "main.h"

#define AUDIO_PROCESS_DEBUG (1)
#define PROCESS_FRAME_SAMPLES (512U)
#define FEEDBACK_DELAY_SAMPLES (1922U)

__attribute__((section(".bss.DTCM"))) SpeexPreprocessState *speex_preprocess_state;
__attribute__((section(".bss.DTCM"))) SpeexEchoState *speex_echo_state;

__attribute__((section(".bss.DTCM"))) uint8_t ifout_ch_num;

__attribute__((section(".bss.DTCM"))) uint32_t last_log_tick;
__attribute__((section(".bss.DTCM"))) uint32_t delayed_samples;

static int32_t speexdsp_init();
static void process_interface_input_audio();
static void process_capture_audio();
static int16_t get_rms(const int16_t *const in, const size_t num);

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
    ifout_ch_num = 1;
    delayed_samples = 0;
    speex_echo_state_reset(speex_echo_state);
}

void audio_processor_set_ifout_ch_num(const uint8_t ch_num)
{
    printf("Set interface out ch num to %u\n", ch_num);
    ifout_ch_num = ch_num;
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

    // 回声消除算法初始化
    speex_echo_state = speex_echo_state_init(PROCESS_FRAME_SAMPLES, 1024);
    if (speex_echo_state == NULL)
    {
        return -1;
    }
    i = 48000;
    ret_int = speex_echo_ctl(speex_echo_state, SPEEX_ECHO_SET_SAMPLING_RATE, &i);
    if (ret_int != 0)
    {
        return -2;
    }

    // 降噪算法初始化
    speex_preprocess_state = speex_preprocess_state_init(PROCESS_FRAME_SAMPLES, 48000);
    if (speex_preprocess_state == NULL)
    {
        return -3;
    }
    i = 1;
    ret_int = speex_preprocess_ctl(speex_preprocess_state, SPEEX_PREPROCESS_SET_AGC, &i);
    if (ret_int != 0)
    {
        return -4;
    }
    i = 1;
    ret_int = speex_preprocess_ctl(speex_preprocess_state, SPEEX_PREPROCESS_SET_ECHO_STATE, speex_echo_state);
    if (ret_int != 0)
    {
        return -5;
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

    const uint32_t mic1_sample_num = mic1_get_sample_num();
    const uint32_t mic2_sample_num = mic2_get_sample_num();
    const uint32_t feedback_sample_num = feedback_get_sample_num();
    if (mic1_sample_num < PROCESS_FRAME_SAMPLES || mic2_sample_num < PROCESS_FRAME_SAMPLES ||
        feedback_sample_num < PROCESS_FRAME_SAMPLES)
    {
        return;
    }

    // 处理麦克风数据
    // 读取麦克风数据
    // 只使用了一个麦克风的数据
    mic2_read(buffer1, PROCESS_FRAME_SAMPLES);
    mic1_read(buffer1, PROCESS_FRAME_SAMPLES);

    arm_scale_q15(buffer1, 8, 15, buffer1, PROCESS_FRAME_SAMPLES);

    int16_t mean = 0;
    arm_mean_q15(buffer1, PROCESS_FRAME_SAMPLES, &mean);
    arm_offset_q15(buffer1, -mean, buffer1, PROCESS_FRAME_SAMPLES);

    if (delayed_samples + PROCESS_FRAME_SAMPLES < FEEDBACK_DELAY_SAMPLES)
    {
        memset(buffer2, 0, PROCESS_FRAME_SAMPLES * 2);
        delayed_samples += PROCESS_FRAME_SAMPLES;
    }
    else if (delayed_samples < FEEDBACK_DELAY_SAMPLES)
    {
        const uint32_t remain_samples_to_delay = FEEDBACK_DELAY_SAMPLES - delayed_samples;
        memset(buffer2, 0, remain_samples_to_delay * MKF360_AUDIO_SAMPLE_SIZE);
        feedback_read(&buffer2[remain_samples_to_delay], PROCESS_FRAME_SAMPLES - remain_samples_to_delay);
        delayed_samples += remain_samples_to_delay;
        printf("%u samples delayed, mic1 samples num: %u, mic2 samples num: %u, feedback samples num: %u\n",
               delayed_samples, mic1_sample_num, mic2_sample_num, feedback_sample_num);
    }
    else
    {
        feedback_read(buffer2, PROCESS_FRAME_SAMPLES);
    }
    arm_scale_q15(buffer2, 4, 15, buffer2, PROCESS_FRAME_SAMPLES);

    // SpeexDSP回声消除对样本非线性的影响及其敏感，
    // 回声消除前不应进行影响样本线性的处理
    speex_echo_cancellation(speex_echo_state, buffer1, buffer2, buffer3);
    speex_preprocess_run(speex_preprocess_state, buffer3);

    if (ifout_ch_num == 1)
    {
        interface_out_write(buffer3, PROCESS_FRAME_SAMPLES);
    }
    else if (ifout_ch_num == 2)
    {
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
        interface_out_write(buffer4, PROCESS_FRAME_SAMPLES * 2);
    }
}

static int16_t get_rms(const int16_t *const in, const size_t num)
{
    int64_t power_sum = 0;
    arm_power_q15(in, num, &power_sum);
    return sqrt((float_t)power_sum / num);
}
