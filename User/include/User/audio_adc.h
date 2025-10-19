#ifndef __AUDIO_ADC_H__
#define __AUDIO_ADC_H__

#include <stdint.h>

void audio_adc_start();

void audio_adc_stop();

void audio_adc_read(int16_t *buffer);

#endif // !__AUDIO_ADC_H__
