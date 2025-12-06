#include "User/audio_io.h"

#include <stdbool.h>

#include "User/audio_adc.h"
#include "User/audio_buffer.h"
#include "User/audio_dac.h"
#include "User/audio_dfsdm.h"
#include "User/audio_iis.h"
#include "User/audio_processor.h"
#include "User/event_group.h"
#include "User/share_buffer.h"
#include "User/usb_desc.h"
#include "main.h"
#include "usbd_core.h"

/**
 * [Interface] --> input_rb --> (process) --> playback_rb --> [Speaker]
 * [Mic] --> capture_rb --> (process) --> output_rb --> [Interface]
 */

typedef enum
{
    AudioIoTypeNone,
    AudioIoTypeAux,
    AudioIoTypeBt,
    AudioIoTypeUac,
} AudioIoType_t;

const char *AUDIO_IO_TYPE_NAME[] = {
    "None",
    "Aux",
    "BT",
    "UAC",
};

typedef enum
{
    FlagsIdxAuxEnabled,
    FlagsIdxBtEnabled,
    FlagsIdxUacEnabled,
} FlagsIdx_t;

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

void audio_io_init()
{
    audio_buffer_init();
    enable_all_audio_io();
}

bool audio_io_is_connected()
{
    return audio_io_type != AudioIoTypeNone;
}

void audio_io_handler()
{
    // 处理连接事件
    bool has_connect_event = false;
    if (event_group_check_event(EventGroup1, EventGroup1AuxConnect, true))
    {
        printf("AUX connect event coming.\n");
        if (audio_io_type != AudioIoTypeNone)
        {
            printf("Current audio io type is: %s, ignore.\n", AUDIO_IO_TYPE_NAME[audio_io_type]);
        }
        else
        {
            disable_audio_io_exclue(AudioIoTypeAux);
            audio_adc_start();
            audio_dac_ctl(AudioDacCmdEnableCh2);
            audio_processor_set_ifout_ch_num(1);
            audio_io_type = AudioIoTypeAux;
            has_connect_event = true;
        }
    }
    if (event_group_check_event(EventGroup1, EventGroup1BtConnect, true))
    {
        printf("BT connect event coming.\n");
        if (audio_io_type != AudioIoTypeNone)
        {
            printf("Current audio io type is: %s, ignore.\n", AUDIO_IO_TYPE_NAME[audio_io_type]);
        }
        else
        {
            disable_audio_io_exclue(AudioIoTypeBt);
            iis_start();
            audio_processor_set_ifout_ch_num(1);
            audio_io_type = AudioIoTypeBt;
            has_connect_event = true;
        }
    }
    if (event_group_check_event(EventGroup1, EventGroup1UsbConnect, true) && audio_io_type == AudioIoTypeNone)
    {
        printf("USB connect event coming.\n");
        if (audio_io_type != AudioIoTypeNone)
        {
            printf("Current audio io type is: %s, ignore.\n", AUDIO_IO_TYPE_NAME[audio_io_type]);
        }
        else
        {
            disable_audio_io_exclue(AudioIoTypeUac);
            audio_processor_set_ifout_ch_num(2);
            audio_io_type = AudioIoTypeUac;
            has_connect_event = true;
        }
    }
    if (has_connect_event == true)
    {
        speaker_start();
        audio_dfsdm_start();
        event_group_set_event(EventGroup1, EventGroup1AudioIoConnected);
    }

    // 处理断开事件
    bool has_disconnect_event = false;
    if (event_group_check_event(EventGroup1, EventGroup1UsbDisconnect, true))
    {
        printf("UAC disconnect event coming.\n");
        if (audio_io_type != AudioIoTypeUac)
        {
            printf("Current audio io type is: %s, ignore.\n", AUDIO_IO_TYPE_NAME[audio_io_type]);
        }
        else
        {
            // CherryUSB 不支持断开事件，
            // 重新初始化协议栈避免协议栈内部重复触发挂起事件
            usbd_deinitialize(0);
            usb_init(0, USB_OTG_FS_PERIPH_BASE);
            has_disconnect_event = true;
        }
    }
    if (event_group_check_event(EventGroup1, EventGroup1BtDisconnect, true) && audio_io_type == AudioIoTypeBt)
    {
        printf("BT disconnect event coming.\n");
        if (audio_io_type != AudioIoTypeBt)
        {
            printf("Current audio io type is: %s, ignore.\n", AUDIO_IO_TYPE_NAME[audio_io_type]);
        }
        else
        {
            iis_stop();
            has_disconnect_event = true;
        }
    }
    if (event_group_check_event(EventGroup1, EventGroup1AuxDisconnect, true) && audio_io_type == AudioIoTypeAux)
    {
        printf("AUX disconnect event coming.\n");
        if (audio_io_type != AudioIoTypeAux)
        {
            printf("Current audio io type is: %s, ignore.\n", AUDIO_IO_TYPE_NAME[audio_io_type]);
        }
        else
        {
            audio_adc_stop();
            audio_dac_ctl(AudioDacCmdDisableCh2);
            has_disconnect_event = true;
        }
    }
    if (has_disconnect_event == true)
    {
        audio_io_type = AudioIoTypeNone;
        audio_dfsdm_stop();
        speaker_stop();
        enable_all_audio_io();
        event_group_set_event(EventGroup1, EventGroup1AudioIoDisconnected);
    }

    int ret_int = 0;

    int16_t *buffer = &shared_buffer[0];

    // 路由底层接口数据
    if (event_group_check_event(EventGroup1, EventGroup1IisDmaBufferReady, true))
    {
        if (audio_io_type == AudioIoTypeBt)
        {
            interface_out_read(iis_get_tx_idle_buffer_address(),
                               MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_NUM_1MS);
            interface_in_write(iis_get_rx_idle_buffer_address(),
                               MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_NUM_1MS);
        }
    }
    if (event_group_check_event(EventGroup1, EventGroup1Adc3DmaBufferReady, true))
    {
        if (audio_io_type == AudioIoTypeAux)
        {
            interface_in_write(audio_adc_get_data_address(),
                               MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_NUM_1MS);
        }
    }
    if (event_group_check_event(EventGroup1, EventGroup1DacCh1DmaBufferReady, true))
    {
        ret_int = speaker_read(buffer, MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_NUM_1MS);
        if (ret_int != 0)
        {
            memset(buffer, 0,
                   MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_SAMPLE_SIZE);
        }
        else
        {
            audio_dac_write_ch(buffer, DacCh1);
        }
        feedback_write(buffer, MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_NUM_1MS);
    }
    if (event_group_check_event(EventGroup1, EventGroup1DacCh2DmaBufferReady, false))
    {
        if (audio_io_type == AudioIoTypeAux)
        {
            ret_int = interface_out_read(&buffer[0], MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_NUM_1MS);
            if (ret_int == 0)
            {
                audio_dac_write_ch(&buffer[0], DacCh2);
                event_group_check_event(EventGroup1, EventGroup1DacCh2DmaBufferReady, true);
            }
        }
    }
    if (event_group_check_event(EventGroup1, EventGroup1UacDataIn, true))
    {
        ret_int = interface_in_write(uac_get_speaker_buffer_address(),
                                     MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_NUM_1MS);
        if (ret_int == 1)
        {
            printf("Interface in data overwrite.\n");
        }
    }
    if (event_group_check_event(EventGroup1, EventGroup1UacDataOut, true))
    {
        int16_t *uac_mic_buffer = uac_get_mic_buffer_address();
        ret_int =
            interface_out_read(uac_mic_buffer, MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_NUM_1MS * 2);
        if (ret_int != 0)
        {
            memset(uac_mic_buffer, 0, MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * 2 * MKF360_AUDIO_SAMPLE_NUM_1MS);
        }
    }
    if (event_group_check_event(EventGroup1, EventGroup1DfsdmFilter0DmaBufferReady, true))
    {
        ret_int = mic1_write(audio_dfsdm_get_filter0_buffer_address(),
                             MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_NUM_1MS);
        if (ret_int == 1)
        {
            printf("Mic1 data overwrite.\n");
        }
    }
    if (event_group_check_event(EventGroup1, EventGroup1DfsdmFilter1DmaBufferReady, true))
    {
        ret_int = mic2_write(audio_dfsdm_get_filter1_buffer_address(),
                             MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_NUM_1MS);
        if (ret_int == 1)
        {
            printf("Mic2 data overwrite.\n");
        }
    }

    if (event_group_check_event(EventGroup1, EventGroup1DfsdmFilter0DmaError, true))
    {
        printf("DFSDM filter0 DMA error.\n");
    }
    if (event_group_check_event(EventGroup1, EventGroup1DfsdmFilter1DmaError, true))
    {
        printf("DFSDM filter1 DMA error.\n");
    }
}
