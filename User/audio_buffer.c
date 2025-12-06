#include "User/audio_buffer.h"

#include "lwrb/lwrb.h"

#include <stdbool.h>
#include <stdio.h>

#include "main.h"

__attribute__((section(".bss.DTCM"))) static lwrb_t input_rb;
__attribute__((section(".bss.DTCM"))) static lwrb_t output_rb;
__attribute__((section(".bss.DTCM"))) static lwrb_t capture1_rb;
__attribute__((section(".bss.DTCM"))) static lwrb_t capture2_rb;
__attribute__((section(".bss.DTCM"))) static lwrb_t playback_rb;
__attribute__((section(".bss.DTCM"))) static lwrb_t feedback_rb;
__attribute__((section(".bss.DTCM"))) static uint8_t
    input_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 2U +
                 1U];
__attribute__((section(".bss.DTCM"))) static uint8_t output_rb_buf
    [MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 2U * 2U + 1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    capture1_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 3U +
                    1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    capture2_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 3U +
                    1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    playback_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 3U +
                    1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    feedback_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 6U +
                    1U];

static inline int generic_read(lwrb_t *rb, void *const out, const size_t sample_num);
static inline int generic_write(lwrb_t *rb, const void *const in, const size_t sample_num);

void audio_buffer_init()
{
    uint8_t ret_uint8 = 0;

    ret_uint8 = lwrb_init(&capture1_rb, &capture1_rb_buf[0], sizeof(capture1_rb_buf));
    if (ret_uint8 != 1U)
    {
        printf("");
    }

    ret_uint8 = lwrb_init(&capture2_rb, &capture2_rb_buf[0], sizeof(capture2_rb_buf));
    if (ret_uint8 != 1U)
    {
        printf("");
    }

    ret_uint8 = lwrb_init(&feedback_rb, &feedback_rb_buf[0], sizeof(feedback_rb_buf));
    if (ret_uint8 != 1U)
    {
        printf("");
    }

    ret_uint8 = lwrb_init(&playback_rb, &playback_rb_buf[0], sizeof(playback_rb_buf));
    if (ret_uint8 != 1U)
    {
        printf("");
    }

    ret_uint8 = lwrb_init(&input_rb, &input_rb_buf[0], sizeof(input_rb_buf));
    if (ret_uint8 != 1U)
    {
        printf("");
    }

    ret_uint8 = lwrb_init(&output_rb, &output_rb_buf[0], sizeof(output_rb_buf));
    if (ret_uint8 != 1U)
    {
        printf("");
    }
}

int mic1_read(void *const out, const size_t sample_num)
{
    return generic_read(&capture1_rb, out, sample_num);
}

int mic1_write(const void *const in, const size_t sample_num)
{
    return generic_write(&capture1_rb, in, sample_num);
}

uint32_t mic1_get_sample_num()
{
    return lwrb_get_full(&capture1_rb) / sizeof(int16_t);
}

int mic2_read(void *const out, const size_t sample_num)
{
    return generic_read(&capture2_rb, out, sample_num);
}

int mic2_write(const void *const in, const size_t sample_num)
{
    return generic_write(&capture2_rb, in, sample_num);
}

uint32_t mic2_get_sample_num()
{
    return lwrb_get_full(&capture2_rb) / sizeof(int16_t);
}

int feedback_read(void *const out, const size_t sample_num)
{
    return generic_read(&feedback_rb, out, sample_num);
}

int feedback_write(const void *const in, const size_t sample_num)
{
    return generic_write(&feedback_rb, in, sample_num);
}

uint32_t feedback_get_sample_num()
{
    return lwrb_get_full(&feedback_rb) / sizeof(int16_t);
}

int speaker_read(void *const out, const size_t sample_num)
{
    return generic_read(&playback_rb, out, sample_num);
}

int speaker_write(const void *const in, const size_t sample_num)
{
    return generic_write(&playback_rb, in, sample_num);
}

int interface_in_read(void *const out, const size_t sample_num)
{
    return generic_read(&input_rb, out, sample_num);
}

int interface_in_write(const void *const in, const size_t sample_num)
{
    return generic_write(&input_rb, in, sample_num);
}

int interface_out_read(void *const out, const size_t sample_num)
{
    return generic_read(&output_rb, out, sample_num);
}

int interface_out_write(const void *const in, const size_t sample_num)
{
    return generic_write(&output_rb, in, sample_num);
}

void reset_audio_rb()
{
    lwrb_reset(&capture1_rb);
    lwrb_reset(&capture2_rb);
    lwrb_reset(&playback_rb);
    lwrb_reset(&feedback_rb);
    lwrb_reset(&input_rb);
    lwrb_reset(&output_rb);
}

static inline int generic_read(lwrb_t *rb, void *const out, const size_t sample_num)
{
    const uint32_t size_to_read = sample_num * MKF360_AUDIO_SAMPLE_SIZE;
    const uint32_t full_size = lwrb_get_full(rb);
    if (full_size < size_to_read)
    {
        return -1;
    }
    lwrb_read(rb, out, size_to_read);
    return 0;
}

static inline int generic_write(lwrb_t *rb, const void *const in, const size_t sample_num)
{
    const uint32_t size_to_write = sample_num * MKF360_AUDIO_SAMPLE_SIZE;
    const uint32_t free_size = lwrb_get_free(rb);
    lwrb_overwrite(rb, in, size_to_write);
    if (size_to_write > free_size)
    {
        return 1;
    }
    return 0;
}
