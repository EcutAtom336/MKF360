#ifndef __AUDIO_IO__
#define __AUDIO_IO__

#include <stdbool.h>
#include <stddef.h>

typedef enum
{
    AudioIoStateConnected,
    AudioIoStateDisconnected,
} AudioIoState_t;

void audio_io_init();

bool audio_io_is_connected();

void audio_io_handler();

#endif // !__AUDIO_IO__
