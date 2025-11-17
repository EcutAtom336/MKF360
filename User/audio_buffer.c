#include "User/audio_buffer.h"

#include "lwrb/lwrb.h"

#include <stdbool.h>
#include <stdio.h>

#include "main.h"

__attribute__((section(".bss.DTCM"))) static lwrb_t input_rb;
__attribute__((section(".bss.DTCM"))) static lwrb_t output_rb;
__attribute__((section(".bss.DTCM"))) static lwrb_t capture_rb;
__attribute__((section(".bss.DTCM"))) static lwrb_t playback_rb;
__attribute__((section(".bss.DTCM"))) static uint8_t
    input_rb_buf[MKF360_AUDIO_PERIPH_DMA_DEST_SAMPLE_NUM * MKF360_AUDIO_SAMPLE_SIZE * 2U + 1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    output_rb_buf[MKF360_AUDIO_PERIPH_DMA_DEST_SAMPLE_NUM * MKF360_AUDIO_SAMPLE_SIZE * 2U + 1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    capture_rb_buf[MKF360_AUDIO_PERIPH_DMA_DEST_SAMPLE_NUM * 4 * MKF360_AUDIO_SAMPLE_SIZE * 2U + 1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    playback_rb_buf[MKF360_AUDIO_PERIPH_DMA_DEST_SAMPLE_NUM * MKF360_AUDIO_SAMPLE_SIZE * 2U + 1U];
__attribute__((section(".bss.DTCM"))) static bool input_rb_read_grant;
__attribute__((section(".bss.DTCM"))) static bool output_rb_read_grant;
__attribute__((section(".bss.DTCM"))) static bool capture_rb_read_grant;
__attribute__((section(".bss.DTCM"))) static bool playback_rb_read_grant;

static inline int32_t generic_read(lwrb_t *rb, void *const out, const size_t sample_num);
static inline int32_t generic_write(lwrb_t *rb, const void *const in, const size_t sample_num);

void audio_buffer_init()
{
    uint8_t ret_uint8 = 0;

    ret_uint8 = lwrb_init(&capture_rb, &capture_rb_buf[0], sizeof(capture_rb_buf));
    if (ret_uint8 != 1U)
    {
        printf("");
    }
    capture_rb_read_grant = false;
    lwrb_set_arg(&capture_rb, &capture_rb_read_grant);

    ret_uint8 = lwrb_init(&playback_rb, &playback_rb_buf[0], sizeof(playback_rb_buf));
    if (ret_uint8 != 1U)
    {
        printf("");
    }
    playback_rb_read_grant = false;
    lwrb_set_arg(&playback_rb, &playback_rb_read_grant);

    ret_uint8 = lwrb_init(&input_rb, &input_rb_buf[0], sizeof(input_rb_buf));
    if (ret_uint8 != 1U)
    {
        printf("");
    }
    input_rb_read_grant = false;
    lwrb_set_arg(&input_rb, &input_rb_read_grant);

    ret_uint8 = lwrb_init(&output_rb, &output_rb_buf[0], sizeof(output_rb_buf));
    if (ret_uint8 != 1U)
    {
        printf("");
    }
    output_rb_read_grant = false;
    lwrb_set_arg(&output_rb, &output_rb_read_grant);
}

int32_t mic_read(void *const out, const size_t sample_num)
{
    return generic_read(&capture_rb, out, sample_num);
}

int32_t mic_write(const void *const in, const size_t sample_num)
{
    return generic_write(&capture_rb, in, sample_num);
}

int32_t speaker_read(void *const out, const size_t sample_num)
{
    return generic_read(&playback_rb, out, sample_num);
}

int32_t speaker_write(const void *const in, const size_t sample_num)
{
    return generic_write(&playback_rb, in, sample_num);
}

int32_t interface_in_read(void *const out, const size_t sample_num)
{
    return generic_read(&input_rb, out, sample_num);
}

int32_t interface_in_write(const void *const in, const size_t sample_num)
{
    return generic_write(&input_rb, in, sample_num);
}

int32_t interface_out_read(void *const out, const size_t sample_num)
{
    return generic_read(&output_rb, out, sample_num);
}

int32_t interface_out_write(const void *const in, const size_t sample_num)
{
    return generic_write(&output_rb, in, sample_num);
}

void reset_audio_rb()
{
    __disable_irq();
    lwrb_reset(&capture_rb);
    *(bool *)lwrb_get_arg(&capture_rb) = false;
    lwrb_reset(&playback_rb);
    *(bool *)lwrb_get_arg(&playback_rb) = false;
    lwrb_reset(&input_rb);
    *(bool *)lwrb_get_arg(&input_rb) = false;
    lwrb_reset(&output_rb);
    *(bool *)lwrb_get_arg(&output_rb) = false;
    __enable_irq();
}

static inline int32_t generic_read(lwrb_t *rb, void *const out, const size_t sample_num)
{
    bool *read_grant = lwrb_get_arg(rb);

    if (*read_grant == false)
    {
        if (lwrb_get_full(rb) > lwrb_get_free(rb) * 2)
        {
            *read_grant = true;
        }
        else
        {
            return -1;
        }
    }
    if (lwrb_get_full(rb) < sample_num * MKF360_AUDIO_SAMPLE_SIZE)
    {
        *read_grant = false;
        return -1;
    }
    __disable_irq();
    lwrb_read(rb, out, sample_num * MKF360_AUDIO_SAMPLE_SIZE);
    __enable_irq();
    return sample_num;
}

static inline int32_t generic_write(lwrb_t *rb, const void *const in, const size_t sample_num)
{
    if (lwrb_get_free(rb) < sample_num * MKF360_AUDIO_SAMPLE_SIZE)
    {
        return -1;
    }
    __disable_irq();
    lwrb_write(rb, in, sample_num * MKF360_AUDIO_SAMPLE_SIZE);
    __enable_irq();
    return sample_num;
}
