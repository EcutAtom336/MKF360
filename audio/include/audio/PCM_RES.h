#ifndef __PCM_RES_H__
#define __PCM_RES_H__

#include <stdint.h>

#define BATTERY_LOW_PCM_LEN ((uint32_t)48000U)
#define BATTERY_LOW_PCM_IDX ((uint8_t)0U)
#define BOOT_PCM_LEN ((uint32_t)48000U)
#define BOOT_PCM_IDX ((uint8_t)1U)
#define CONNECTED_PCM_LEN ((uint32_t)48000U)
#define CONNECTED_PCM_IDX ((uint8_t)2U)

#define NONE_PCM_IDX ((uint8_t)3U)

extern const int16_t BATTERY_LOW_PCM[BATTERY_LOW_PCM_LEN];
extern const int16_t BOOT_PCM[BOOT_PCM_LEN];
extern const int16_t CONNECTED_PCM[CONNECTED_PCM_LEN];

extern const int16_t *PCM_RES[];
extern const uint32_t PCM_RES_LEN[];

#endif // __PCM_RES_H__
