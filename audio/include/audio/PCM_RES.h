#ifndef __PCM_RES_H__
#define __PCM_RES_H__

#include <stdint.h>

#define BATTERY_LOW_PCM_LEN ((uint32_t)21400U)
#define BATTERY_LOW_PCM_IDX ((uint8_t)0U)
#define BLUETOOTH_CONNECT_PCM_LEN ((uint32_t)28400U)
#define BLUETOOTH_CONNECT_PCM_IDX ((uint8_t)1U)
#define BLUETOOTH_DISCONNECT_PCM_LEN ((uint32_t)28400U)
#define BLUETOOTH_DISCONNECT_PCM_IDX ((uint8_t)2U)
#define BOOT_PCM_LEN ((uint32_t)21800U)
#define BOOT_PCM_IDX ((uint8_t)3U)

#define NONE_PCM_IDX ((uint8_t)4U)

extern const int16_t BATTERY_LOW_PCM[BATTERY_LOW_PCM_LEN];
extern const int16_t BLUETOOTH_CONNECT_PCM[BLUETOOTH_CONNECT_PCM_LEN];
extern const int16_t BLUETOOTH_DISCONNECT_PCM[BLUETOOTH_DISCONNECT_PCM_LEN];
extern const int16_t BOOT_PCM[BOOT_PCM_LEN];

#endif // __PCM_RES_H__
