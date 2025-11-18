#ifndef __AUDIO_BUFFER_H__
#define __AUDIO_BUFFER_H__

#include <stddef.h>

void audio_buffer_init();

int mic1_read(void *const out, const size_t sample_num);
int mic1_write(const void *const in, const size_t sample_num);

int mic2_read(void *const out, const size_t sample_num);
int mic2_write(const void *const in, const size_t sample_num);

int speaker_read(void *const out, const size_t sample_num);
int speaker_write(const void *const in, const size_t sample_num);

int interface_in_read(void *const out, const size_t sample_num);
int interface_in_write(const void *const in, const size_t sample_num);

int interface_out_read(void *const out, const size_t sample_num);
int interface_out_write(const void *const in, const size_t sample_num);

void reset_audio_rb();

#endif // !__AUDIO_BUFFER_H__
