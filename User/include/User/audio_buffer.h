#ifndef __AUDIO_BUFFER_H__
#define __AUDIO_BUFFER_H__

#include <stddef.h>
#include <stdint.h>

void audio_buffer_init();

int32_t mic_read(void *const out, const size_t sample_num);
int32_t mic_write(const void *const in, const size_t sample_num);

int32_t speaker_read(void *const out, const size_t sample_num);
int32_t speaker_write(const void *const in, const size_t sample_num);

int32_t interface_in_read(void *const out, const size_t sample_num);
int32_t interface_in_write(const void *const in, const size_t sample_num);

int32_t interface_out_read(void *const out, const size_t sample_num);
int32_t interface_out_write(const void *const in, const size_t sample_num);

void reset_audio_rb();

#endif // !__AUDIO_BUFFER_H__
