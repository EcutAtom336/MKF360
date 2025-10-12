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

/** @typedef OnDacDmaFrameEmptyIsrCallback */
/**
 * @brief DAC DMA 缓冲空回调，在 DMA 半完成和全完成中断中被调用。
 *
 * @details 用户可以在此函数中读取接收到的数据并填充要发送的数据。
 *
 * @note 中断环境回调。
 *
 * @param buffer 指向缓冲区的指针。
 *               DAC DMA 缓冲区由 uint32_t 组成，
 *               每个 uint32_t 的低 16 位 CH1 的数据，高 16 位 CH2 的数据。
 *               建议使用 @see audio_dac_util_write_ch 写入。
 * @param sample_num 缓冲可容纳的数据个数。
 */
typedef void (*OnDacDmaBufferEmptyIsrCallback_t)(void *buffer, const size_t sample_num);

void audio_dac_register_on_dac_dma_buffer_empty_isr_callback(const OnDacDmaBufferEmptyIsrCallback_t cb);

/**
 * @brief 写入 DAC DMA 缓冲区工具函数。
 *
 * @param buffer 指向 DAC DMA 缓冲区的指针。
 * @param data 指向要写入的数据的指针。
 * @param sample_num 要写入的数据个数。
 * @param ch 写入的 DAC 通道。
 */
void audio_dac_util_write_ch(void *buffer, const int16_t *data, const size_t sample_num, const DacCh_t ch);

void audio_dac_ctl(const AudioDacCmd_t cmd);

#endif // !__AUDIO_DAC_H__
