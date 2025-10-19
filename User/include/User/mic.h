#ifndef __MIC_H__
#define __MIC_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "MKF360_config.h"

void mic_mdma_init();

void mic_start();

void mic_stop();

bool mic_verify_interlaced_data();

void mic_interlace_data_read(int16_t *buffer);

int16_t *get_mic_interlaces_data_address();

#endif // !__MIC_H__
