#ifndef __MIC_H__
#define __MIC_H__

#include <stdbool.h>
#include <stdint.h>

void mic_mdma_init();

void mic_start();

void mic_stop();

bool mic_verify_interlaced_data();

int16_t *get_mic_interlaces_data_address();

#endif // !__MIC_H__
