#ifndef __MIC_H__
#define __MIC_H__

#include <stdbool.h>
#include <stdint.h>

#include "MKF360_config.h"

typedef void (*MicInterlacedDataReadyCallback)();

void mic_mdma_init();

void register_mic_interlaced_data_ready_callback(MicInterlacedDataReadyCallback callback);

void mic_start();

void mic_stop();

void mic_interlaced_data_read(int16_t *const buffer);

bool mic_verify_interlaced_data();

#endif // !__MIC_H__
