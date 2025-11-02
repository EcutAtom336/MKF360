#include "User/audio_io.h"

#include "arm_math.h"
#include "lwrb/lwrb.h"
#include "stm32h7xx_hal.h"

#include "MKF360_config.h"
#include "User/audio_adc.h"
#include "User/audio_dac.h"
#include "User/audio_iis.h"
#include "User/event_group.h"
#include "User/usb_desc.h"
#include "main.h"
#include "usbd_core.h"

typedef enum
{
    AudioIoTypeNone,
    AudioIoTypeAux,
    AudioIoTypeBt,
    AudioIoTypeUac,
} AudioIoType_t;

typedef enum
{
    FlagsIdxAuxEnabled,
    FlagsIdxBtEnabled,
    FlagsIdxUacEnabled,
} FlagsIdx_t;

__attribute__((section(".bss.DTCM"))) static lwrb_t capture_rb;
__attribute__((section(".bss.DTCM"))) static lwrb_t playback_rb;
__attribute__((section(".bss.DTCM"))) static uint8_t
    audio_capture_rb_buf[MKF360_DMA_FRAME_SAMPLE_NUM * MKF360_AUDIO_SAMPLE_SIZE * 2U + 1U];
__attribute__((section(".bss.DTCM"))) static uint8_t
    audio_playback_rb_buf[MKF360_DMA_FRAME_SAMPLE_NUM * MKF360_AUDIO_SAMPLE_SIZE * 2U + 1U];

__attribute__((section(".bss.DTCM"))) static uint32_t flags;

__attribute__((section(".bss.DTCM"))) static AudioIoType_t audio_io_type = AudioIoTypeNone;

static inline void aux_enable();
static inline void aux_disable();
static inline void bt_enable();
static inline void bt_disable();
static inline void uac_enable();
static inline void uac_disable();
static void speaker_start();
static void speaker_stop();
static int32_t capture_read(void *const data, const size_t sample_num);
static int32_t playback_write(const void *const data, const size_t sample_num);
static int32_t playback_read(void *const data, const size_t sample_num);
static void reset_audio_rb();

static inline void aux_enable()
{
#warning "Hardware unsupport, only set flag"
    flags |= (1U << FlagsIdxAuxEnabled);
}

static inline void aux_disable()
{
#warning "Hardware unsupport, only clear flag"
    flags &= ~(1U << FlagsIdxAuxEnabled);
}

static inline void bt_enable()
{
    HAL_GPIO_WritePin(BT_DISABLE__GPIO_Port, BT_DISABLE__Pin, GPIO_PIN_SET);
    flags |= (1U << FlagsIdxBtEnabled);
}

static inline void bt_disable()
{
    HAL_GPIO_WritePin(BT_DISABLE__GPIO_Port, BT_DISABLE__Pin, GPIO_PIN_RESET);
    flags &= ~(1U << FlagsIdxBtEnabled);
}

static inline void uac_enable()
{
    // 初始化 USB 协议栈
    usb_init(0, USB_OTG_FS_PERIPH_BASE);
    // 使能 USB 断开检测
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = VBUS_DETECT_Pin,
        .Mode = GPIO_MODE_IT_FALLING,
        .Pull = GPIO_PULLDOWN,
    };
    HAL_GPIO_Init(VBUS_DETECT_GPIO_Port, &GPIO_InitStruct);
    flags |= (1U << FlagsIdxUacEnabled);
}

static inline void uac_disable()
{
    // 反初始化 USB 协议栈
    usbd_deinitialize(0);
    // 禁用 USB 断检测
    HAL_GPIO_DeInit(VBUS_DETECT_GPIO_Port, VBUS_DETECT_Pin);
    flags &= ~(1U << FlagsIdxUacEnabled);
}

static void speaker_start()
{
    HAL_GPIO_WritePin(SPEAKER_EN_GPIO_Port, SPEAKER_EN_Pin, GPIO_PIN_SET);
    audio_dac_ctl(AudioDacCmdEnableCh1);
}

static void speaker_stop()
{
    HAL_GPIO_WritePin(SPEAKER_EN_GPIO_Port, SPEAKER_EN_Pin, GPIO_PIN_RESET);
    audio_dac_ctl(AudioDacCmdDisableCh1);
}

static inline void disable_audio_io_exclue(AudioIoType_t exclude)
{
    if (exclude != AudioIoTypeAux)
    {
        aux_disable();
    }

    if (exclude != AudioIoTypeBt)
    {
        bt_disable();
    }

    if (exclude != AudioIoTypeUac)
    {
        uac_disable();
    }
}

static inline void enable_all_audio_io()
{
    if (!(flags & (1U << FlagsIdxAuxEnabled)))
    {
        aux_enable();
    }

    if (!(flags & (1U << FlagsIdxBtEnabled)))
    {
        bt_enable();
    }

    if (!(flags & (1U << FlagsIdxUacEnabled)))
    {
        uac_enable();
    }
}

static int32_t capture_read(void *const data, const size_t sample_num)
{
    if (lwrb_get_full(&capture_rb) < sample_num * MKF360_AUDIO_SAMPLE_SIZE)
    {
        return -1;
    }
    __disable_irq();
    lwrb_read(&capture_rb, data, sample_num * MKF360_AUDIO_SAMPLE_SIZE);
    __enable_irq();
    return sample_num;
}

static int32_t playback_write(const void *const data, const size_t sample_num)
{
    __disable_irq();
    lwrb_overwrite(&playback_rb, data, sample_num * MKF360_AUDIO_SAMPLE_SIZE);
    __enable_irq();
    return sample_num;
}

static int32_t playback_read(void *const data, const size_t sample_num)
{
    if (lwrb_get_full(&playback_rb) < sample_num * MKF360_AUDIO_SAMPLE_SIZE)
    {
        return -1;
    }
    __disable_irq();
    lwrb_read(&playback_rb, data, sample_num * MKF360_AUDIO_SAMPLE_SIZE);
    __enable_irq();
    return sample_num;
}

static void reset_audio_rb()
{
    __disable_irq();
    lwrb_reset(&capture_rb);
    lwrb_reset(&playback_rb);
    __enable_irq();
}

void audio_io_init()
{
    uint8_t ret_uint8 = 0;

    // 初始化 IO 缓冲区
    ret_uint8 = lwrb_init(&capture_rb, &audio_capture_rb_buf[0], sizeof(audio_capture_rb_buf));
    if (ret_uint8 != 1U)
    {
        Error_Handler();
    }

    ret_uint8 = lwrb_init(&playback_rb, &audio_playback_rb_buf[0], sizeof(audio_playback_rb_buf));
    if (ret_uint8 != 1U)
    {
        Error_Handler();
    }

    enable_all_audio_io();
}

bool audio_io_is_connected()
{
    return audio_io_type != AudioIoTypeNone;
}

void audio_io_write(const void *const data, const size_t sample_num)
{
    __disable_irq();
    lwrb_overwrite(&capture_rb, data, sample_num * MKF360_AUDIO_SAMPLE_SIZE);
    __enable_irq();
}

void audio_io_handler()
{
    // 处理连接事件
    if (event_group_check_event(EventGroup1, EventGroup1AuxConnect, true) && audio_io_type == AudioIoTypeNone)
    {
        disable_audio_io_exclue(AudioIoTypeAux);
        audio_adc_start();
        speaker_start();
        audio_dac_ctl(AudioDacCmdEnableCh2);
        audio_io_type = AudioIoTypeAux;
        event_group_set_event(EventGroup1, EventGroup1AudioIoConnected);
    }
    if (event_group_check_event(EventGroup1, EventGroup1BtConnect, true) && audio_io_type == AudioIoTypeNone)
    {
        disable_audio_io_exclue(AudioIoTypeBt);
        iis_start();
        speaker_start();
        audio_io_type = AudioIoTypeBt;
        event_group_set_event(EventGroup1, EventGroup1AudioIoConnected);
    }
    if (event_group_check_event(EventGroup1, EventGroup1UsbConnect, true) && audio_io_type == AudioIoTypeNone)
    {
        disable_audio_io_exclue(AudioIoTypeUac);
        speaker_start();
        audio_io_type = AudioIoTypeUac;
        event_group_set_event(EventGroup1, EventGroup1AudioIoConnected);
    }

    // 处理断开事件
    bool has_disconnect_event = false;
    if (event_group_check_event(EventGroup1, EventGroup1UsbDisconnect, true) && audio_io_type == AudioIoTypeUac)
    {
        // CherryUSB 不支持断开事件，
        // 重新初始化协议栈避免协议栈内部重复触发挂起事件
        usbd_deinitialize(0);
        usb_init(0, USB_OTG_FS_PERIPH_BASE);
        has_disconnect_event = true;
        event_group_set_event(EventGroup1, EventGroup1AudioIoDisconnected);
    }
    if (event_group_check_event(EventGroup1, EventGroup1BtDisconnect, true) && audio_io_type == AudioIoTypeBt)
    {
        speaker_stop();
        iis_stop();
        event_group_set_event(EventGroup1, EventGroup1AudioIoDisconnected);
    }
    if (event_group_check_event(EventGroup1, EventGroup1AuxDisconnect, true) && audio_io_type == AudioIoTypeAux)
    {
        audio_adc_stop();
        audio_dac_ctl(AudioDacCmdDisableCh2);
        event_group_set_event(EventGroup1, EventGroup1AudioIoDisconnected);
    }
    if (has_disconnect_event == true)
    {
        speaker_stop();
        audio_io_type = AudioIoTypeNone;
        enable_all_audio_io();
        reset_audio_rb();
    }

    int32_t ret_int32 = 0;

    // 路由底层接口数据
    if (event_group_check_event(EventGroup1, EventGroup1IisDmaBufferReady, true))
    {
        if (audio_io_type == AudioIoTypeBt)
        {
            capture_read(iis_get_tx_idle_buffer_address(), IIS_DMA_FRAME_SAMPLE_NUM);
            playback_write(iis_get_rx_idle_buffer_address(), IIS_DMA_FRAME_SAMPLE_NUM);
        }
    }
    if (event_group_check_event(EventGroup1, EventGroup1Adc3DmaBufferReady, true))
    {
        if (audio_io_type == AudioIoTypeAux)
        {
            playback_write(audio_adc_get_data_address(), MKF360_DMA_FRAME_SAMPLE_NUM);
        }
    }
    if (event_group_check_event(EventGroup1, EventGroup1DacDmaBufferReady, true))
    {
        __attribute__((section(".bss.DTCM"))) static int16_t tmp[DAC_DMA_FRAME_SAMPLE_NUM];

        ret_int32 = playback_read(&tmp[0], DAC_DMA_FRAME_SAMPLE_NUM);
        if (ret_int32 == DAC_DMA_FRAME_SAMPLE_NUM)
        {
            audio_dac_write_ch(&tmp[0], DacCh1);
        }

        if (audio_io_type == AudioIoTypeAux)
        {
            ret_int32 = capture_read(&tmp[0], DAC_DMA_FRAME_SAMPLE_NUM);
            if (ret_int32 == DAC_DMA_FRAME_SAMPLE_NUM)
            {
                audio_dac_write_ch(&tmp[0], DacCh2);
            }
        }
    }
    if (event_group_check_event(EventGroup1, EventGroup1UacDataIn, true))
    {
        playback_write(uac_get_read_buffer_address(), MKF360_DMA_FRAME_SAMPLE_NUM);
    }
    if (event_group_check_event(EventGroup1, EventGroup1UacDataOut, true))
    {
        capture_read(uac_get_write_buffer_address(), MKF360_DMA_FRAME_SAMPLE_NUM);
    }
}
