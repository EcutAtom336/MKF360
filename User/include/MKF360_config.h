#ifndef __MKF360_CONFIG_H__
#define __MKF360_CONFIG_H__

typedef enum
{
    EventGroup1MicDataInterlaceComplete,

    EventGroup1UacConnect,
    EventGroup1Disconnect,

    EventGroup1Tick500Pass,
} EventGroup1Idx_t;

#define EVENT_BIT(event_idx) (1U << (event_idx))

#define MKF360_MIC_SAMPLE_RATE_HZ (16000U)
#define DFSDM_DMA_FRAME_SAMPLE_NUM (MKF360_MIC_SAMPLE_RATE_HZ / 50U)
#define DAC_DMA_FRAME_SAMPLE_NUM (MKF360_MIC_SAMPLE_RATE_HZ / 50U)

#endif // !__MKF360_CONFIG_H__