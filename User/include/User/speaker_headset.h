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

    SpkHdstCmdSpkPlay,
    SpkHdstCmdHdstPlay,
} SpkHdstCmd_t;

typedef union {

    struct
    {
        const int16_t *data;
        size_t sample_num;
    } spk_play;

    struct
    {
        const int16_t *data;
        size_t sample_num;
    } hdst_play;

} SpkHdstCtlData_t;

typedef void (*DacBufferResetCallback)();

void spk_hdst_init();

void register_dac_buffer_reset_callback(DacBufferResetCallback callback);

void spk_hdst_ctl(const SpkHdstCmd_t cmd, SpkHdstCtlData_t *data);

#endif // !__SPEAKER_HEADSET_H__
