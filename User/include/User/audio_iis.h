#ifndef __AUDIO_IIS_H__
#define __AUDIO_IIS_H__

#include <stddef.h>
#include <stdint.h>

/** @typedef OnIisDmaFrameReadyIsrCallback */
/**
 * @brief IIS DMA 缓冲就绪回调，在 DMA 半完成和全完成中断中被调用。
 *
 * @details 用户可以在此函数中读取接收到的数据并填充要发送的数据。
 *
 * @note 中断环境回调。
 *
 * @param rx_buffer IIS 接收数据 DMA 缓冲区。从这复制接收到的数据。
 * @param tx_buffer IIS 发送数据 DMA 缓冲区。在这写入要发送的数据。
 * @param sample_num 缓冲区数据量（以数据个数为单位）。
 */
typedef void (*OnIisDmaBufferReadyIsrCallback)(int16_t *rx_buffer, int16_t *tx_buffer, const size_t sample_num);

void audio_iis_register_on_dma_buffer_ready_isr_callback(const OnIisDmaBufferReadyIsrCallback isr_cb);

void iis_start();

void iis_stop();

#endif // !__AUDIO_IIS_H__