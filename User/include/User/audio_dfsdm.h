#ifndef __AUDIO_DFSDM_H__
#define __AUDIO_DFSDM_H__

#include <stdbool.h>
#include <stdint.h>

void audio_dfsdm_start();

void audio_dfsdm_stop();

int16_t *audio_dfsdm_get_filter0_buffer_address();

int16_t *audio_dfsdm_get_filter1_buffer_address();

#endif // !__AUDIO_DFSDM_H__
