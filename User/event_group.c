#include "User/event_group.h"

#include "stm32h7xx.h"

#include <stdint.h>

__attribute__((section(".bss.DTCM"))) uint32_t event_groups[EventGroupNum];

void event_group_set_event(const EventGroupIndex_t group_idx, const uint8_t event_idx)
{
    ATOMIC_SET_BIT(event_groups[group_idx], (1U << event_idx));
}

bool event_group_check_event(const EventGroupIndex_t group_idx, const uint8_t event_idx, const bool clean_if_set)
{
    bool is_set = event_groups[group_idx] & (1U << event_idx);
    if (clean_if_set && is_set)
    {
        ATOMIC_CLEAR_BIT(event_groups[group_idx], (1U << event_idx));
    }
    return is_set;
}
