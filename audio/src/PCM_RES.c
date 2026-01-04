#include "audio/PCM_RES.h"

#include <stdint.h>

const int16_t *PCM_RES[] = {
    BATTERY_LOW_PCM,
    BOOT_PCM,
    CONNECTED_PCM,
};

const uint32_t PCM_RES_LEN[] = {
    BATTERY_LOW_PCM_LEN,
    BOOT_PCM_LEN,
    CONNECTED_PCM_LEN,
};
