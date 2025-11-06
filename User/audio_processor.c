#include "User/audio_processor.h"

#include <stdint.h>

#include "MKF360_config.h"
#include "User/audio_buffer.h"

__attribute__((section(".bss.DTCM"))) static int16_t buffer1[MKF360_AUDIO_PERIPH_DMA_FRAME_SAMPLE_NUM * 4];

void audio_process()
{
    int32_t ret_int32 = 0;

    ret_int32 = interface_in_read(&buffer1[0], MKF360_AUDIO_PERIPH_DMA_FRAME_SAMPLE_NUM);
    if (ret_int32 == MKF360_AUDIO_PERIPH_DMA_FRAME_SAMPLE_NUM)
    {
        speaker_write(&buffer1[0], MKF360_AUDIO_PERIPH_DMA_FRAME_SAMPLE_NUM);
    }

    ret_int32 = mic_read(&buffer1[0], MKF360_AUDIO_PERIPH_DMA_FRAME_SAMPLE_NUM * 4);
    if (ret_int32 == MKF360_AUDIO_PERIPH_DMA_FRAME_SAMPLE_NUM * 4)
    {
        for (size_t i = 0; i < MKF360_AUDIO_PERIPH_DMA_FRAME_SAMPLE_NUM; i++)
        {
            buffer1[i] = (buffer1[i * 4 + 0] + buffer1[i * 4 + 1] + buffer1[i * 4 + 2] + buffer1[i * 4 + 3]) * 20;
        }
        interface_out_write(&buffer1[0], MKF360_AUDIO_PERIPH_DMA_FRAME_SAMPLE_NUM);
    }
}
