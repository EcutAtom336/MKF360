#ifndef __AUDIO_IO__
#define __AUDIO_IO__

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    AudioIoStateConnected,
    AudioIoStateDisconnected,
} AudioIoState_t;

void audio_io_init();

bool audio_io_is_connected();

void audio_io_handler();

void audio_io_play_prompt(const uint8_t prompt_idx);

#endif // !__AUDIO_IO__
