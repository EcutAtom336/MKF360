#include "User/audio_adc.h"
#include "User/event_group.h"
#include "adc.h"
#include "main.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // CherryUSB 不支持 USB disconnect 事件触发，
    // 使用 VBUS 下降沿触发 USB disconnect
    if (GPIO_Pin == VBUS_DETECT_Pin)
    {
        if (HAL_GPIO_ReadPin(VBUS_DETECT_GPIO_Port, VBUS_DETECT_Pin) == GPIO_PIN_RESET)
        {
            event_group_set_event(EventGroup1, EventGroup1UsbDisconnect);
        }
    }
    else if (GPIO_Pin == BT_STAT_Pin)
    {
        if (HAL_GPIO_ReadPin(BT_STAT_GPIO_Port, BT_STAT_Pin) == GPIO_PIN_SET)
        {
            // BT connect
            event_group_set_event(EventGroup1, EventGroup1BtConnect);
        }
        else
        {
            // BT disconnect
            event_group_set_event(EventGroup1, EventGroup1BtDisconnect);
        }
    }
    else if (GPIO_Pin == HEADSET_DET_Pin)
    {
        if (HAL_GPIO_ReadPin(HEADSET_DET_GPIO_Port, HEADSET_DET_Pin) == GPIO_PIN_SET)
        {
            event_group_set_event(EventGroup1, EventGroup1AuxConnect);
        }
        else
        {
            event_group_set_event(EventGroup1, EventGroup1AuxDisconnect);
        }
    }
}

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
