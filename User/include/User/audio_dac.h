#ifndef __AUDIO_DAC_H__
#define __AUDIO_DAC_H__

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    AudioDacCmdEnableCh1,
    AudioDacCmdDisableCh1,
    AudioDacCmdEnableCh2,
    AudioDacCmdDisableCh2,
} AudioDacCmd_t;

typedef enum
{
    DacCh1,
    DacCh2,
} DacCh_t;

typedef void (*OnDacDmaFrameEmptyIsrCallback)(void *frames, const size_t sample_num);

void audio_dac_init(const OnDacDmaFrameEmptyIsrCallback cb);

void audio_dac_util_write_ch(void *frame, const int16_t *data, const size_t sample_num, const DacCh_t ch);

void audio_dac_ctl(const AudioDacCmd_t cmd);

#endif // !__AUDIO_DAC_H__
