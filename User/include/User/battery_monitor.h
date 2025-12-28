#ifndef __BATTERY_MONITOR_H__
#define __BATTERY_MONITOR_H__

#include <math.h>

void battery_monitor_handler();

float_t battery_monitor_get_voltage();

#endif // !__BATTERY_MONITOR_H__
