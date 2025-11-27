#include "User/audio_buffer.h"

#include "lwrb/lwrb.h"

#include <stdbool.h>
#include <stdio.h>

#include "main.h"

#define MIC_DELAY_MS (MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * 3)
#define MIC_DELAY_SIZE (MIC_DELAY_MS * MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_SAMPLE_SIZE)

__attribute__((section(".bss.DTCM"))) static lwrb_t input_rb;
__attribute__((section(".bss.DTCM"))) static lwrb_t output_rb;
__attribute__((section(".bss.DTCM"))) static lwrb_t output_auxiliary_rb;
__attribute__((section(".bss.DTCM"))) static lwrb_t capture1_rb;
__attribute__((section(".bss.DTCM"))) static lwrb_t capture2_rb;
__attribute__((section(".bss.DTCM"))) static lwrb_t playback_rb;
__attribute__((section(".bss.DTCM"))) static lwrb_t feedback_rb;
__attribute__((section(".bss.DTCM"))) static uint8_t
    input_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 2U +
                 1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    output_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 2U +
                  1U];
__attribute__((section(".bss.DTCM"))) static uint8_t output_auxiliary_rb_buf
    [MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 2U + 1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    capture1_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 4U +
                    1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    capture2_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 4U +
                    1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    playback_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 3U +
                    1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    feedback_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 8U +
                    1U];
__attribute__((section(".bss.DTCM"))) static uint32_t capture1_latest_timestamp;
__attribute__((section(".bss.DTCM"))) static uint32_t capture2_latest_timestamp;
__attribute__((section(".bss.DTCM"))) static uint32_t feedback_latest_timestamp;

static inline int generic_read(lwrb_t *rb, void *const out, const size_t ms);
static inline int generic_write(lwrb_t *rb, const void *const in, const size_t ms);
static inline int generic_mic_write(lwrb_t *rb, const void *const in, const size_t ms, const uint32_t latest_timestamp);
static inline int generic_mic_read(lwrb_t *rb, void *const out, const size_t ms_to_read,
                                   uint32_t *const earliest_timestamp);

void audio_buffer_init()
{
    uint8_t ret_uint8 = 0;

    ret_uint8 = lwrb_init(&capture1_rb, &capture1_rb_buf[0], sizeof(capture1_rb_buf));
    if (ret_uint8 != 1U)
    {
        printf("");
    }
    lwrb_set_arg(&capture1_rb, &capture1_latest_timestamp);

    ret_uint8 = lwrb_init(&capture2_rb, &capture2_rb_buf[0], sizeof(capture2_rb_buf));
    if (ret_uint8 != 1U)
    {
        printf("");
    }
    lwrb_set_arg(&capture2_rb, &capture2_latest_timestamp);

    ret_uint8 = lwrb_init(&feedback_rb, &feedback_rb_buf[0], sizeof(feedback_rb_buf));
    if (ret_uint8 != 1U)
    {
        printf("");
    }
    lwrb_set_arg(&feedback_rb, &feedback_latest_timestamp);

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

    ret_uint8 = lwrb_init(&output_auxiliary_rb, &output_auxiliary_rb_buf[0], sizeof(output_auxiliary_rb_buf));
    if (ret_uint8 != 1U)
    {
        printf("");
    }
}

int mic1_read(void *const out, const size_t ms, uint32_t *const earliest_timestamp)
{
    return generic_mic_read(&capture1_rb, out, ms, earliest_timestamp);
}

int mic1_write(const void *const in, const size_t ms, const uint32_t latest_timestamp)
{
    return generic_mic_write(&capture1_rb, in, ms, latest_timestamp);
}

int mic2_read(void *const out, const size_t ms, uint32_t *const earliest_timestamp)
{
    return generic_mic_read(&capture2_rb, out, ms, earliest_timestamp);
}

int mic2_write(const void *const in, const size_t ms, const uint32_t latest_timestamp)
{
    return generic_mic_write(&capture2_rb, in, ms, latest_timestamp);
}

int feedback_read(void *const out, const size_t start_timestamp, const size_t ms_to_read)
{
    const uint32_t size_to_read = ms_to_read * MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_SAMPLE_SIZE;
    const uint32_t full_size = lwrb_get_full(&feedback_rb);
    if (full_size < size_to_read)
    {
        return -1;
    }
    const uint32_t sample_ms = full_size / (MKF360_AUDIO_SAMPLE_SIZE * MKF360_AUDIO_SAMPLE_NUM_1MS);
    const uint32_t lastest_timestamp = *(uint32_t *)lwrb_get_arg(&feedback_rb);
    const uint32_t earliest_timestamp = lastest_timestamp - sample_ms;
    if (start_timestamp > lastest_timestamp)
    {
        return -2;
    }
    if (earliest_timestamp > start_timestamp)
    {
        return -3;
    }
    if (start_timestamp - earliest_timestamp + ms_to_read > sample_ms)
    {
        return -4;
    }
    const uint32_t actual_read_size = lwrb_peek(
        &feedback_rb, (start_timestamp - earliest_timestamp) * MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_SAMPLE_SIZE,
        out, size_to_read);
    if (actual_read_size != size_to_read)
    {
        return -5;
    }
    return 0;
}

int feedback_write(const void *const in, const size_t ms, const uint32_t latest_timestamp)
{
    *(uint32_t *)lwrb_get_arg(&feedback_rb) = latest_timestamp;
    return generic_write(&feedback_rb, in, ms);
}

int speaker_read(void *const out, const size_t ms)
{
    return generic_read(&playback_rb, out, ms);
}

int speaker_write(const void *const in, const size_t ms)
{
    return generic_write(&playback_rb, in, ms);
}

int interface_in_read(void *const out, const size_t ms)
{
    return generic_read(&input_rb, out, ms);
}

int interface_in_write(const void *const in, const size_t ms)
{
    return generic_write(&input_rb, in, ms);
}

int interface_out_read(void *const out, const size_t ms)
{
    return generic_read(&output_rb, out, ms);
}

int interface_out_write(const void *const in, const size_t ms)
{
    return generic_write(&output_rb, in, ms);
}

int interface_out_auxiliary_read(void *const out, const size_t ms)
{
    return generic_read(&output_auxiliary_rb, out, ms);
}

int interface_out_auxiliary_write(const void *const in, const size_t ms)
{
    return generic_write(&output_auxiliary_rb, in, ms);
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

static inline int generic_read(lwrb_t *rb, void *const out, const size_t ms)
{
    const uint32_t size_to_read = ms * MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_SAMPLE_SIZE;
    const uint32_t full_size = lwrb_get_full(rb);
    if (full_size < size_to_read)
    {
        return -1;
    }
    lwrb_read(rb, out, size_to_read);
    return 0;
}

static inline int generic_write(lwrb_t *rb, const void *const in, const size_t ms)
{
    const uint32_t size_to_write = ms * MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_SAMPLE_SIZE;
    const uint32_t free_size = lwrb_get_free(rb);
    lwrb_overwrite(rb, in, size_to_write);
    if (size_to_write > free_size)
    {
        return 1;
    }
    return 0;
}

static inline int generic_mic_write(lwrb_t *rb, const void *const in, const size_t ms, const uint32_t latest_timestamp)
{
    *(uint32_t *)lwrb_get_arg(rb) = latest_timestamp;
    return generic_write(rb, in, ms);
}

static inline int generic_mic_read(lwrb_t *rb, void *const out, const size_t ms_to_read,
                                   uint32_t *const earliest_timestamp)
{
    const uint32_t size_to_read = ms_to_read * MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_SAMPLE_SIZE;
    const uint32_t full_size = lwrb_get_full(rb);
    if (size_to_read + MIC_DELAY_SIZE > full_size)
    {
        return -1;
    }
    const uint32_t latest_timestamp = *(uint32_t *)lwrb_get_arg(rb);
    const uint32_t contain_ms = full_size / (MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_SAMPLE_SIZE);
    *earliest_timestamp = latest_timestamp - contain_ms;
    const uint32_t actual_read_size = lwrb_read(rb, out, size_to_read);
    if (actual_read_size != size_to_read)
    {
        return -2;
    }
    return 0;
}
