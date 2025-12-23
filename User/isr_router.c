#include "User/audio_adc.h"
#include "adc.h"

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc3)
    {
        audio_adc_dma_half_cplt_isr_callback();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc3)
    {
        audio_adc_dma_cplt_isr_callback();
    }
}
