#ifndef __AUDIO_PROCESSOR_H__
#define __AUDIO_PROCESSOR_H__

#include <stdint.h>

int32_t audio_processor_init();

void audio_processor_reset();

void audio_processor_set_ifout_ch_num(const uint8_t ch_num);

void audio_process();

#endif // ! __AUDIO_PROCESSOR_H__
