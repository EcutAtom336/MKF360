#ifndef __AUDIO_IIS_H__
#define __AUDIO_IIS_H__

#include <stdint.h>

void iis_start();

void iis_stop();

int16_t *iis_get_tx_idle_buffer_address();

int16_t *iis_get_rx_idle_buffer_address();

#endif // !__AUDIO_IIS_H__