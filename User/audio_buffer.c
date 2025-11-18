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

static inline int generic_read(lwrb_t *rb, void *const out, const size_t sample_num);
static inline int generic_write(lwrb_t *rb, const void *const in, const size_t sample_num);

void audio_buffer_init()
{
    uint8_t ret_uint8 = 0;

    ret_uint8 = lwrb_init(&capture_rb, &capture_rb_buf[0], sizeof(capture_rb_buf));
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

int mic_read(void *const out, const size_t sample_num)
{
    return generic_read(&capture_rb, out, sample_num);
}

int mic_write(const void *const in, const size_t sample_num)
{
    return generic_write(&capture_rb, in, sample_num);
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
    lwrb_reset(&capture_rb);
    lwrb_reset(&playback_rb);
    lwrb_reset(&input_rb);
    lwrb_reset(&output_rb);
}

static inline int generic_read(lwrb_t *rb, void *const out, const size_t sample_num)
{
    uint32_t read_size = sample_num * MKF360_AUDIO_SAMPLE_SIZE;
    if (lwrb_get_full(rb) < read_size)
    {
        return -1;
    }
    lwrb_read(rb, out, read_size);
    return sample_num;
}

static inline int generic_write(lwrb_t *rb, const void *const in, const size_t sample_num)
{
    uint32_t write_size = sample_num * MKF360_AUDIO_SAMPLE_SIZE;
    bool is_overwrite = lwrb_get_free(rb) < write_size ? true : false;
    lwrb_overwrite(rb, in, write_size);
    return is_overwrite ? 1 : 0;
}
