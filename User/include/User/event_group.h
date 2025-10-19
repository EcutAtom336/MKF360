#ifndef __EVENT_GROUP_H__
#define __EVENT_GROUP_H__

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    EventGroup1,

    EventGroupNum,
} EventGroupIndex_t;

typedef enum
{
    EventGroup1MicDataInterlaced,

    EventGroup1DacDmaBufferReady,
    EventGroup1IisDmaBufferReady,
    EventGroup1Adc3DmaBufferReady,

    EventGroup1UacConnect,
    EventGroup1Disconnect,

    EventGroup1Tick50Pass,
    EventGroup1Tick500Pass,
} EventGroupEventIndex_t;

bool event_group_set_event(const EventGroupIndex_t group_idx, const uint8_t event_idx);

bool event_group_check_event(const EventGroupIndex_t group_idx, const uint8_t event_idx, const bool clean_if_set);

#endif // !__EVENT_GROUP_H__
