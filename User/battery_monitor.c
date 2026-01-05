#include "User/battery_monitor.h"

#include <stdbool.h>
#include <stdio.h>

#include "adc.h"
#include "main.h"

#define BATTERY_CHANGE_DELAY_TICK (500U)

#define BATTERY_LEVEL_LOW_VOLTAGE_THRESHOLD (3.3F)
#define BATTERY_LEVEL_MIDDLE_VOLTAGE_THRESHOLD (3.7F)
#define BATTERY_LEVEL_HIGH_VOLTAGE_THRESHOLD (4.0F)

typedef enum
{
    BatteryLevelVeryLow,
    BatteryLevelLow,
    BatteryLevelMiddle,
    BatteryLevelHigh,
    BatteryLevelNone,
} BatteryLevel;

static const char *BATTERY_LEVEL_NAMES[] = {
    [BatteryLevelVeryLow] = "Very low",
    [BatteryLevelLow] = "Low",
    [BatteryLevelMiddle] = "Middle",
    [BatteryLevelHigh] = "High",
};

__attribute__((section(".DTCM"))) static bool adc2_started;
__attribute__((section(".DTCM"))) static bool battery_voltage_available;

__attribute__((section(".DTCM"))) static float_t battery_voltage;

__attribute__((section(".DTCM"))) static BatteryLevel level_report = BatteryLevelNone;
__attribute__((section(".DTCM"))) static BatteryLevel level_internal = BatteryLevelNone;
__attribute__((section(".DTCM"))) static uint32_t level_change_tick;

__attribute__((section(".DTCM"))) static uint32_t last_led_change_tick;

static void renew_indicator();
static void start_measure();
static void calculate_voltage();
static void renew_level();
static void renew_indicator();

void battery_monitor_handler()
{
    start_measure();
    calculate_voltage();
    renew_level();
    renew_indicator();
}

float_t battery_monitor_get_voltage()
{
    return battery_voltage_available ? battery_voltage : -1.0F;
}

static void start_measure()
{
    if (adc2_started)
    {
        return;
    }
    if (HAL_ADC_Start(&hadc2) != HAL_OK)
    {
        printf("Start ADC2 fail.\n");
        return;
    }
    adc2_started = true;
}

static void calculate_voltage()
{
    if (adc2_started == false || LL_ADC_REG_IsConversionOngoing(hadc2.Instance))
    {
        return;
    }

    uint32_t adc2_val = HAL_ADC_GetValue(&hadc2);
    adc2_val &= 0xFF;
    battery_voltage = adc2_val / 255.0F * 3.0F * 2.0F;
    battery_voltage_available = true;

    adc2_started = false;
}

static void renew_level()
{
    if (battery_voltage_available == false)
    {
        return;
    }

    BatteryLevel current_level = BatteryLevelVeryLow;
    if (battery_voltage >= BATTERY_LEVEL_HIGH_VOLTAGE_THRESHOLD)
    {
        current_level = BatteryLevelHigh;
    }
    else if (battery_voltage >= BATTERY_LEVEL_MIDDLE_VOLTAGE_THRESHOLD)
    {
        current_level = BatteryLevelMiddle;
    }
    else if (battery_voltage >= BATTERY_LEVEL_LOW_VOLTAGE_THRESHOLD)
    {
        current_level = BatteryLevelLow;
    }

    if (current_level != level_internal || level_internal == BatteryLevelNone)
    {
        level_internal = current_level;
        level_change_tick = HAL_GetTick();
    }

    if ((level_internal != level_report && HAL_GetTick() - level_change_tick >= BATTERY_CHANGE_DELAY_TICK) ||
        level_report == BatteryLevelNone)
    {
        level_report = level_internal;
        printf("Battery level changed: %s\n", BATTERY_LEVEL_NAMES[level_report]);
    }
}

static void renew_indicator()
{

    if (level_report == BatteryLevelHigh)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    }
    else if (level_report == BatteryLevelMiddle)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    }
    else if (level_report == BatteryLevelLow)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    }
    else if (level_report == BatteryLevelVeryLow)
    {
        if (HAL_GetTick() - last_led_change_tick >= 200)
        {
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        }
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    }
}
