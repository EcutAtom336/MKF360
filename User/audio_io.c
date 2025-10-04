#include "User/audio_io.h"

#include "stm32h7xx_hal.h"

#include "User/usb_desc.h"

__attribute__((section(".DTCM"))) OnAudioPathChangeCallback on_audio_path_change_callback = NULL;
__attribute__((section(".DTCM"))) OnAudioInCallback on_audio_in_callback = NULL;
__attribute__((section(".DTCM"))) OnAudioOutCallback on_audio_out_callback = NULL;

void audio_init()
{
}

void register_on_audio_path_change_callback(OnAudioPathChangeCallback callback)
{
    on_audio_path_change_callback = callback;
}

void register_on_audio_in_callback(OnAudioInCallback callback)
{
    on_audio_in_callback = callback;
}

void register_on_audio_out_callback(OnAudioOutCallback callback)
{
    on_audio_out_callback = callback;
}
