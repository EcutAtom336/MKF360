#ifndef __AUDIO_DAC_H__
#define __AUDIO_DAC_H__

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    AudioDacCmdEnableCh1,
    AudioDacCmdDisableCh1,
    AudioDacCmdEnableCh2,
    AudioDacCmdDisableCh2,
} AudioDacCmd_t;

typedef enum
{
    DacCh1,
    DacCh2,
} DacCh_t;

/**
 * @brief 写入 DAC DMA 缓冲区函数。
 *
 * @param data 指向要写入的数据的指针。
 * @param sample_num 要写入的数据个数。
 * @param ch 写入的 DAC 通道。
 */
void audio_dac_write_ch(const int16_t *data, const DacCh_t ch);

void audio_dac_ctl(const AudioDacCmd_t cmd);

#endif // !__AUDIO_DAC_H__
