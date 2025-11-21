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
__attribute__((section(".bss.DTCM"))) static uint8_t
    output_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 2U +
                  1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    capture1_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 4U +
                    1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    capture2_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 4U +
                    1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    playback_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 2U +
                    1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    feedback_rb_buf[MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE * 8U +
                    1U];
__attribute__((section(".bss.DTCM"))) static uint32_t capture1_latest_timestamp;
__attribute__((section(".bss.DTCM"))) static uint32_t capture2_latest_timestamp;
__attribute__((section(".bss.DTCM"))) static uint32_t feedback_latest_timestamp;

static inline int generic_read(lwrb_t *rb, void *const out, const size_t ms);
static inline int generic_write(lwrb_t *rb, const void *const in, const size_t ms);

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
}

int mic1_read(void *const out, const size_t ms)
{
    return generic_read(&capture1_rb, out, ms);
}

int mic1_write(const void *const in, const size_t ms, const uint32_t latest_timestamp)
{
    *(uint32_t *)lwrb_get_arg(&capture1_rb) = latest_timestamp;
    return generic_write(&capture1_rb, in, ms);
}

uint32_t mic1_get_earliest_timestamp()
{
    const uint32_t FULL_SIZE = lwrb_get_full(&capture1_rb);
    if (FULL_SIZE == 0)
    {
        return 0;
    }
    const uint32_t SAMPLE_MS = FULL_SIZE / (MKF360_AUDIO_SAMPLE_SIZE * MKF360_AUDIO_SAMPLE_NUM_1MS);
    return *(uint32_t *)lwrb_get_arg(&capture1_rb) - SAMPLE_MS;
}

int mic2_read(void *const out, const size_t ms)
{
    return generic_read(&capture2_rb, out, ms);
}

int mic2_write(const void *const in, const size_t ms, const uint32_t latest_timestamp)
{
    *(uint32_t *)lwrb_get_arg(&capture2_rb) = latest_timestamp;
    return generic_write(&capture2_rb, in, ms);
}

uint32_t mic2_get_earliest_timestamp()
{
    const uint32_t FULL_SIZE = lwrb_get_full(&capture2_rb);
    if (FULL_SIZE == 0)
    {
        return 0;
    }
    const uint32_t SAMPLE_MS = FULL_SIZE / (MKF360_AUDIO_SAMPLE_SIZE * MKF360_AUDIO_SAMPLE_NUM_1MS);
    return *(uint32_t *)lwrb_get_arg(&capture2_rb) - SAMPLE_MS;
}

int feedback_read(void *const out, const size_t timestamp, const size_t ms)
{
    const uint32_t FULL_SIZE = lwrb_get_full(&feedback_rb);
    if (FULL_SIZE < MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_SAMPLE_SIZE)
    {
        return -1;
    }
    const uint32_t SAMPLE_MS = FULL_SIZE / (MKF360_AUDIO_SAMPLE_SIZE * MKF360_AUDIO_SAMPLE_NUM_1MS);
    const uint32_t EARLIEST_TIMESTAMP = *(uint32_t *)lwrb_get_arg(&feedback_rb) - SAMPLE_MS;
    if (EARLIEST_TIMESTAMP > timestamp)
    {
        return -2;
    }
    uint32_t sz = lwrb_peek(&feedback_rb,
                            (timestamp - EARLIEST_TIMESTAMP) * MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_SAMPLE_SIZE,
                            out, ms * MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_SAMPLE_SIZE);
    if (sz != ms * MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_SAMPLE_SIZE)
    {
        return -3;
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
    uint32_t read_size = ms * MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_SAMPLE_SIZE;
    if (lwrb_get_full(rb) < read_size)
    {
        return -1;
    }
    lwrb_read(rb, out, read_size);
    return ms;
}

static inline int generic_write(lwrb_t *rb, const void *const in, const size_t ms)
{
    uint32_t write_size = ms * MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_SAMPLE_SIZE;
    bool is_overwrite = lwrb_get_free(rb) < write_size ? true : false;
    lwrb_overwrite(rb, in, write_size);
    return is_overwrite ? 1 : 0;
}
