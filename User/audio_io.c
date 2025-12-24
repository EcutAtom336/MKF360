#include "User/audio_io.h"

#include <stdbool.h>

#include "arm_math.h"

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

#define IO_STAT_CHANGE_TICK_THRESHOLD (50U)

typedef enum
{
    AudioIoTypeNone,
    AudioIoTypeAux,
    AudioIoTypeBt,
    AudioIoTypeUac,
} AudioIoType_t;

typedef enum
{
    FlagsIdxBtEnabled,
    FlagsIdxUacEnabled,
    FlagsIdxAuxDetected,
    FlagsIdxBtDetected,
    FlagsIdxUacDetected,
} FlagsIdx_t;

__attribute__((section(".bss.DTCM"))) static uint32_t flags;

__attribute__((section(".bss.DTCM"))) static bool aux_changed_detected;
__attribute__((section(".bss.DTCM"))) static bool bt_changed_detected;
__attribute__((section(".bss.DTCM"))) static uint32_t aux_change_detected_tick;
__attribute__((section(".bss.DTCM"))) static uint32_t bt_change_detected_tick;

__attribute__((section(".bss.DTCM"))) static AudioIoType_t audio_io_type = AudioIoTypeNone;

static inline void bt_enable();
static inline void bt_disable();
static inline void uac_enable();
static inline void uac_disable();
static void speaker_start();
static void speaker_stop();
static void hardware_link_detect();
static void aux_detect();
static void bt_detect();
static void uac_detect();
static void software_link_switch();

static inline void bt_enable()
{
    HAL_GPIO_WritePin(BT_ENABLE_GPIO_Port, BT_ENABLE_Pin, GPIO_PIN_SET);
    flags |= (1U << FlagsIdxBtEnabled);
}

static inline void bt_disable()
{
    HAL_GPIO_WritePin(BT_ENABLE_GPIO_Port, BT_ENABLE_Pin, GPIO_PIN_RESET);
    flags &= ~(1U << FlagsIdxBtEnabled);
}

static inline void uac_enable()
{
    // 初始化 USB 协议栈
    usb_init(0, USB_OTG_HS_PERIPH_BASE);
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
    // 检测硬件连接状态
    hardware_link_detect();

    // 切换软件链路
    software_link_switch();

    int ret_int = 0;

    int16_t *buffer = &shared_buffer[0];

    // 路由底层接口数据
    if (event_group_check_event(EventGroup1, EventGroup1IisDmaBufferReady, true))
    {
        if (audio_io_type == AudioIoTypeBt)
        {
            interface_out_read(iis_get_tx_idle_buffer_address(),
                               MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_NUM_1MS);
            int16_t *rx_buffer = iis_get_rx_idle_buffer_address();
            // 修复 IIS 接收偶发的全 0 数据问题
            for (size_t i = 0; i < MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_NUM_1MS - 1; i++)
            {
                if (abs(rx_buffer[i] - rx_buffer[i + 1]) > 10 && rx_buffer[i + 1] == 0)
                {
                    rx_buffer[i + 1] = rx_buffer[i];
                }
            }
            interface_in_write(rx_buffer, MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_NUM_1MS);
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

static void hardware_link_detect()
{
    aux_detect();
    bt_detect();
    uac_detect();
}

static void aux_detect()
{
    const bool aux_detect_stat = READ_BIT(flags, 1U << FlagsIdxAuxDetected);
    const bool current_aux_detect_stat = (HAL_GPIO_ReadPin(AUX1_DET_GPIO_Port, AUX1_DET_Pin) == GPIO_PIN_SET ||
                                          HAL_GPIO_ReadPin(AUX2_DET_GPIO_Port, AUX2_DET_Pin) == GPIO_PIN_SET);
    if (aux_detect_stat == current_aux_detect_stat)
    {
        aux_changed_detected = false;
        return;
    }

    if (aux_changed_detected == false)
    {
        aux_changed_detected = true;
        aux_change_detected_tick = HAL_GetTick();
    }

    if (HAL_GetTick() - aux_change_detected_tick < IO_STAT_CHANGE_TICK_THRESHOLD)
    {
        return;
    }

    if (current_aux_detect_stat)
    {
        printf("Hardware link detected: AUX\n");
        SET_BIT(flags, 1U << FlagsIdxAuxDetected);
    }
    else
    {
        printf("Hardware link removed: AUX\n");
        CLEAR_BIT(flags, 1U << FlagsIdxAuxDetected);
    }
}

static void bt_detect()
{
    const bool bt_detect_stat = READ_BIT(flags, 1U << FlagsIdxBtDetected);
    const bool current_bt_detect_stat = HAL_GPIO_ReadPin(BT_STAT_GPIO_Port, BT_STAT_Pin) == GPIO_PIN_SET;
    if (bt_detect_stat == current_bt_detect_stat)
    {
        bt_changed_detected = false;
        return;
    }

    if (bt_changed_detected == false)
    {
        bt_changed_detected = true;
        bt_change_detected_tick = HAL_GetTick();
    }

    if (HAL_GetTick() - bt_change_detected_tick < IO_STAT_CHANGE_TICK_THRESHOLD)
    {
        return;
    }

    if (current_bt_detect_stat)
    {
        printf("Hardware link detected: BT\n");
        SET_BIT(flags, 1U << FlagsIdxBtDetected);
    }
    else
    {
        printf("Hardware link removed: BT\n");
        CLEAR_BIT(flags, 1U << FlagsIdxBtDetected);
    }
}

static void uac_detect()
{
    const bool uac_detect_stat = READ_BIT(flags, 1U << FlagsIdxUacDetected);
    const bool current_uac_detect_stat =
        (HAL_GPIO_ReadPin(VBUS_DETECT_GPIO_Port, VBUS_DETECT_Pin) == GPIO_PIN_SET && usb_device_is_configured(0));
    if (uac_detect_stat == current_uac_detect_stat)
    {
        return;
    }

    if (current_uac_detect_stat)
    {
        printf("Hardware link detected: UAC\n");
        SET_BIT(flags, 1U << FlagsIdxUacDetected);
    }
    else
    {
        printf("Hardware link removed: UAC\n");
        CLEAR_BIT(flags, 1U << FlagsIdxUacDetected);
    }
}

static void software_link_switch()
{
    const bool uac_detected = READ_BIT(flags, 1U << FlagsIdxUacDetected);
    const bool bt_detected = READ_BIT(flags, 1U << FlagsIdxBtDetected);
    const bool aux_detected = READ_BIT(flags, 1U << FlagsIdxAuxDetected);

    // 硬件链路已移除
    if (uac_detected == false && bt_detected == false && aux_detected == false)
    {
        if (audio_io_type == AudioIoTypeNone)
        {
            return;
        }
        printf("All hardware are removed.\n");
        if (audio_io_type == AudioIoTypeAux)
        {
            printf("Close software link: AUX\n");
            audio_adc_stop();
            audio_dac_ctl(AudioDacCmdDisableCh2);
        }
        else if (audio_io_type == AudioIoTypeBt)
        {
            printf("Close software link: BT\n");
            iis_stop();
        }
        else if (audio_io_type == AudioIoTypeUac)
        {
            printf("Close software link: UAC\n");
            // CherryUSB 不支持断开事件，
            // 重新初始化协议栈避免协议栈内部重复触发挂起事件
            usbd_deinitialize(0);
            usb_init(0, USB_OTG_HS_PERIPH_BASE);
        }
        audio_io_type = AudioIoTypeNone;
        audio_dfsdm_stop();
        speaker_stop();
        enable_all_audio_io();
        event_group_set_event(EventGroup1, EventGroup1AudioIoDisconnected);
        return;
    }

    if (uac_detected)
    {
        // 软件链路和硬件链路已对应
        if (audio_io_type == AudioIoTypeUac)
        {
            return;
        }

        printf("New hardware link detected: UAC.\n");

        // 关闭原软件链路
        if (audio_io_type == AudioIoTypeAux)
        {
            printf("Disable current software link: AUX.\n");
            audio_adc_stop();
            audio_dac_ctl(AudioDacCmdDisableCh2);
        }
        else if (audio_io_type == AudioIoTypeBt)
        {
            printf("Disable current software link: BT.\n");
            iis_stop();
        }

        // 启动新的软件链路
        printf("Switch to new software link: UAC.\n");
        audio_processor_set_ifout_ch_num(2);

        // 通用步骤
        if (audio_io_type == AudioIoTypeNone)
        {
            speaker_start();
            audio_dfsdm_start();
            event_group_set_event(EventGroup1, EventGroup1AudioIoConnected);
        }
        disable_audio_io_exclue(AudioIoTypeUac);
        audio_io_type = AudioIoTypeUac;
    }
    else if (bt_detected)
    {
        // 软件链路和硬件链路已对应
        if (audio_io_type == AudioIoTypeBt)
        {
            return;
        }

        printf("New hardware link detected: BT.\n");

        // 关闭原软件链路
        if (audio_io_type == AudioIoTypeAux)
        {
            printf("Disable current software link: AUX.\n");
            audio_adc_stop();
            audio_dac_ctl(AudioDacCmdDisableCh2);
        }

        // 启动新的软件链路
        printf("Switch to new software link: BT.\n");
        iis_start();
        audio_processor_set_ifout_ch_num(1);

        // 通用步骤
        if (audio_io_type == AudioIoTypeNone)
        {
            speaker_start();
            audio_dfsdm_start();
            event_group_set_event(EventGroup1, EventGroup1AudioIoConnected);
        }

        disable_audio_io_exclue(AudioIoTypeBt);
        audio_io_type = AudioIoTypeBt;
    }
    else if (aux_detected)
    {
        // 软件链路和硬件链路已对应
        if (audio_io_type == AudioIoTypeAux)
        {
            return;
        }

        // 启动新的软件链路
        printf("Switch to new software link: AUX.\n");
        audio_adc_start();
        audio_dac_ctl(AudioDacCmdEnableCh2);
        audio_processor_set_ifout_ch_num(1);

        // 通用步骤
        if (audio_io_type == AudioIoTypeNone)
        {
            speaker_start();
            audio_dfsdm_start();
            event_group_set_event(EventGroup1, EventGroup1AudioIoConnected);
        }

        disable_audio_io_exclue(AudioIoTypeAux);
        audio_io_type = AudioIoTypeAux;
    }
}
