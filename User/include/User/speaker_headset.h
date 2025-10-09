#ifndef __SPEAKER_HEADSET_H__
#define __SPEAKER_HEADSET_H__

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    SpkHdstCmdEnableSpk,
    SpkHdstCmdDisableSpk,

    SpkHdstCmdEnableHdst,
    SpkHdstCmdDisableHdst,
} SpkHdstCmd_t;

typedef void (*OnDacDmaFrameEmptyIsrCallback)(void *frame, const size_t sample_num);

void spk_hdst_init(const OnDacDmaFrameEmptyIsrCallback cb);

void audio_dac_util_write_speaker(void *frame, const int16_t *data, const size_t sample_num);

void audio_dac_util_write_headset(void *frame, const int16_t *data, const size_t sample_num);

void spk_hdst_ctl(const SpkHdstCmd_t cmd);

#endif // !__SPEAKER_HEADSET_H__
