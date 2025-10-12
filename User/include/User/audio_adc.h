#ifndef __AUDIO_ADC_H__
#define __AUDIO_ADC_H__

#include <stddef.h>

/** @typedef OnAdcDmaBufferReadyIsrCallback_t */
/**
 * @brief ADC DMA 缓冲空回调，在 DMA 半完成和全完成中断中被调用。
 * 
 * @details 用户可以在此函数中读取接收到的数据并填充要发送的数据。
 * 
 * @note 中断环境回调。
 * 
 * @param buffer DMA 缓冲区指针。
 *               每个数据单元为 int16_t，已使用数字去偏置。
 * @param sample_num 数据个数。
 */
typedef void (*OnAdcDmaBufferReadyIsrCallback_t)(const void *buffer, const size_t sample_num);

void audio_adc_register_on_adc_dma_buffer_ready_isr_callback(OnAdcDmaBufferReadyIsrCallback_t cb);

void audio_adc_start();

void audio_adc_stop();

#endif // !__AUDIO_ADC_H__
