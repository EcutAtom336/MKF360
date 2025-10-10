#ifndef __AUDIO_IIS_H__
#define __AUDIO_IIS_H__

#include <stddef.h>
#include <stdint.h>

typedef void (*OnIisDmaFrameReadyIsrCallback)(int16_t *rx_data, int16_t *tx_data, const size_t sample_num);

void audio_iis_register_on_dma_frame_ready_isr_callback(const OnIisDmaFrameReadyIsrCallback callback);

void iis_start();

void iis_stop();

#endif // !__AUDIO_IIS_H__