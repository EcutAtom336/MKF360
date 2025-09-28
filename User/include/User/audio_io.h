#ifndef __AUDIO_IO__
#define __AUDIO_IO__

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    AudioPathUac,
} AudioPath_t;

typedef void (*OnAudioPathChangeCallback)(const AudioPath_t old_path, const AudioPath_t new_path);
typedef void (*OnAudioInCallback)(int16_t *buffer, const size_t sample_num);
typedef void (*OnAudioOutCallback)(int16_t *buffer, const size_t sample_num);

#endif // !__AUDIO_IO__