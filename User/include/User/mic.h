#ifndef __MIC_H__
#define __MIC_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "MKF360_config.h"

/** @typedef MicInterlacedDataReadyIsrCallback */
/**
 * @brief Mic 数据交错完成回调，在 MDMA 中断中调用。
 *
 * @note 中断环境回调。
 *
 * @param buffer Mic 交错数据地址。
 * @param sample_num 数据数量。
 */
typedef void (*MicInterlacedDataReadyIsrCallback)(const int16_t *buffer, const size_t sample_num);

void mic_mdma_init();

void register_mic_interlaced_data_ready_callback(MicInterlacedDataReadyIsrCallback callback);

void mic_start();

void mic_stop();

bool mic_verify_interlaced_data();

#endif // !__MIC_H__
