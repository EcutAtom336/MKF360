#ifndef __MIC_H__
#define __MIC_H__

#include <stdbool.h>
#include <stdint.h>

void mic_start();

void mic_stop();

int16_t *mic_get_mic1_buffer_address();

int16_t *mic_get_mic2_buffer_address();

#endif // !__MIC_H__
