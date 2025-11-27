#include "User/usb_desc.h"

#include "arm_math.h"

#include "usbd_core.h"
//
#include "usbd_audio.h"

#include "User/event_group.h"
#include "main.h"

#ifndef CONFIG_USBDEV_ADVANCE_DESC
#error "Please enable CONFIG_USBDEV_ADVANCE_DESC macro."
#endif

/*
 * 修改描述符注意事项：
 * 1. INTYERFACE_NUM USB_CONFIG_SIZE 任何配置都有，必须与实际描述符长度匹配。
 * 2. 特定配置下的描述符长度宏，如 AUDIO_AC_SIZ 也必须与实际描述符长度匹配。
 */

#define USBD_VID 0xFFFF
#define USBD_PID 0xFFFF
#define USBD_MAX_POWER 100
#define USBD_LANGID_STRING 1033

#define AUDIO_IN_EP 0x81
#define AUDIO_OUT_EP 0x01

#define INTYERFACE_NUM 3
#define UAC_FIRST_INTERFACE 0x00
#define UAC_MIC_INTERFACE 0x01
#define UAC_SPEAKER_INTERFACE 0x02

#define EP_INTERVAL 0x01
#define FEEDBACK_ENDP_PACKET_SIZE 0x03

#define AUDIO_IN_FU_ID 0x02
#define AUDIO_OUT_FU_ID 0x05

#define AUDIO_SPEAKER_FREQ MKF360_AUDIO_SAMPLE_RATE_HZ
#define AUDIO_SPEAKER_FRAME_SIZE_BYTE 2u
#define AUDIO_SPEAKER_RESOLUTION_BIT 16u
#define AUDIO_SPEAKER_CHANNELS 1u
#define AUDIO_MIC_FREQ MKF360_AUDIO_SAMPLE_RATE_HZ
#define AUDIO_MIC_FRAME_SIZE_BYTE 2u
#define AUDIO_MIC_RESOLUTION_BIT 16u
#define AUDIO_MIC_CHANNELS 2u

#define AUDIO_OUT_PACKET                                                                                               \
    ((uint32_t)((AUDIO_SPEAKER_FREQ * AUDIO_SPEAKER_FRAME_SIZE_BYTE * AUDIO_SPEAKER_CHANNELS) / 1000))
#define AUDIO_IN_PACKET ((uint32_t)((AUDIO_MIC_FREQ * AUDIO_MIC_FRAME_SIZE_BYTE * AUDIO_MIC_CHANNELS) / 1000))

// clang-format off

#define USB_CONFIG_SIZE (9 + \
                         AUDIO_AC_DESCRIPTOR_INIT_LEN(2) + \
                         AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC + \
                         AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(AUDIO_MIC_CHANNELS, 1) + \
                         AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC + \
                         AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC + \
                         AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(AUDIO_SPEAKER_CHANNELS, 1) + \
                         AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC + \
                         AUDIO_AS_DESCRIPTOR_INIT_LEN(1) + \
                         AUDIO_AS_DESCRIPTOR_INIT_LEN(1) \
                        )

#define AUDIO_AC_SIZ (AUDIO_SIZEOF_AC_HEADER_DESC(2) + \
                      AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC + \
                      AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(AUDIO_MIC_CHANNELS, 1) + \
                      AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC + \
                      AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC + \
                      AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(AUDIO_SPEAKER_CHANNELS, 1) + \
                      AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC \
                     )

// 用于方便查看描述符大小
static const uint16_t USB_DESC_SIZE = USB_CONFIG_SIZE;
static const uint32_t UAC_AC_SIZE = AUDIO_AC_SIZ;

// clang-format on

static const uint8_t DEVICE_DESCRIPTOR[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0,  // bcdUSB
                               0x00,     // bDeviceClass
                               0x00,     // bDeviceSubClass
                               0x00,     // bDeviceProtocol
                               USBD_VID, // idVendor
                               USBD_PID, // idProduct
                               0x0100,   // bcdDevice
                               0x01      // bNumConfigurations
                               ),
};

static const uint8_t CONFIG_DESCRIPTOR[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE,        // bLength
                               INTYERFACE_NUM,         // bNumInterfaces
                               0x01,                   // bConfigurationValue
                               USB_CONFIG_BUS_POWERED, // bmAttributes
                               USBD_MAX_POWER          // bMaxPower
                               ),
    //
    AUDIO_AC_DESCRIPTOR_INIT(UAC_FIRST_INTERFACE,  // bFirstInterface
                             0x03,                 // bInterfaceCount
                             AUDIO_AC_SIZ,         // wTotalLength
                             0x00,                 // stridx
                             UAC_MIC_INTERFACE,    // 从这里开始是包含的 interface 号，
                             UAC_SPEAKER_INTERFACE // 接口号的顺序要从小到大
                             ),

    AUDIO_AC_INPUT_TERMINAL_DESCRIPTOR_INIT(0x01,               // bTerminalID
                                            AUDIO_INTERM_MIC,   // wTerminalType
                                            AUDIO_MIC_CHANNELS, // bNrChannels
                                            0x0000              // wChannelConfig
                                            ),
    AUDIO_AC_FEATURE_UNIT_DESCRIPTOR_INIT(AUDIO_IN_FU_ID, // bUnitID
                                          0x01,           // bSourceID
                                          0x01,           // bControlSize
                                          0x03,           //
                                          0x00,           //
                                          0x00            //
                                          ),
    AUDIO_AC_OUTPUT_TERMINAL_DESCRIPTOR_INIT(0x03,                     // bTerminalID
                                             AUDIO_TERMINAL_STREAMING, // wTerminalType
                                             AUDIO_IN_FU_ID            // bSourceID
                                             ),

    AUDIO_AC_INPUT_TERMINAL_DESCRIPTOR_INIT(0x04,                     // bTerminalID
                                            AUDIO_TERMINAL_STREAMING, // wTerminalType
                                            AUDIO_SPEAKER_CHANNELS,   // bNrChannels
                                            0x0000                    // wChannelConfig
                                            ),
    AUDIO_AC_FEATURE_UNIT_DESCRIPTOR_INIT(AUDIO_OUT_FU_ID, // bUnitID
                                          0x04,            // bSourceID
                                          0x01,            // bControlSize
                                          0x03,            //
                                          0x00             //
                                          ),
    AUDIO_AC_OUTPUT_TERMINAL_DESCRIPTOR_INIT(0x06,                  // bTerminalID
                                             AUDIO_OUTTERM_SPEAKER, // wTerminalType
                                             AUDIO_OUT_FU_ID        // bSourceID
                                             ),
    // AUDIO_AS_DESCRIPTOR_INIT 的顺序要和 AUDIO_AC_DESCRIPTOR_INIT 里一样
    AUDIO_AS_DESCRIPTOR_INIT(UAC_MIC_INTERFACE,                   // bInterfaceNumber
                             0x03,                                // bTerminalLink
                             AUDIO_MIC_CHANNELS,                  // bNrChannels
                             AUDIO_MIC_FRAME_SIZE_BYTE,           // bSubFrameSize
                             AUDIO_MIC_RESOLUTION_BIT,            // bBitResolution
                             AUDIO_IN_EP,                         // bEndpointAddress
                             0x05,                                // bmAttributes
                             AUDIO_IN_PACKET,                     // wMaxPacketSize
                             EP_INTERVAL,                         // bInterval
                             AUDIO_SAMPLE_FREQ_3B(AUDIO_MIC_FREQ) //
                             ),
    AUDIO_AS_DESCRIPTOR_INIT(UAC_SPEAKER_INTERFACE,                   // bInterfaceNumber
                             0x04,                                    // bTerminalLink
                             AUDIO_SPEAKER_CHANNELS,                  // bNrChannels
                             AUDIO_SPEAKER_FRAME_SIZE_BYTE,           // bSubFrameSize
                             AUDIO_SPEAKER_RESOLUTION_BIT,            // bBitResolution
                             AUDIO_OUT_EP,                            // bEndpointAddress
                             0x09,                                    // bmAttributes
                             AUDIO_OUT_PACKET,                        // wMaxPacketSize
                             EP_INTERVAL,                             // bInterval
                             AUDIO_SAMPLE_FREQ_3B(AUDIO_SPEAKER_FREQ) //
                             ),
};

static const uint8_t DEVICE_QUALITY_DESCRIPTOR[] = {
    0x0a, USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
};

static const char *STRING_DESCRIPTORS[] = {
    (const char[]){0x09, 0x04}, /* Langid */
    "OSHWHub",                  /* Manufacturer */
    "MKF360",                   /* Product */
    "88888888",                 /* Serial Number */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return DEVICE_DESCRIPTOR;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return CONFIG_DESCRIPTOR;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return DEVICE_QUALITY_DESCRIPTOR;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;
    if (index > 3)
    {
        return NULL;
    }
    return STRING_DESCRIPTORS[index];
}

const struct usb_descriptor usb_desc = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
};

// MKF360_AUDIO_PERIPH_DMA_DEST_SAMPLE_NUM 必须是 AUDIO_IN_PACKET 和 AUDIO_OUT_PACKET 的倍数
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t
    uac_speaker_buffer[2][MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t uac_mic_buffer[2][MKF360_AUDIO_SAMPLE_NUM_1MS *
                                                                        MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST *
                                                                        MKF360_AUDIO_SAMPLE_SIZE * AUDIO_MIC_CHANNELS];
__attribute__((section(".bss.DTCM"))) volatile static uint8_t uac_speaker_idle_buffer_idx;
__attribute__((section(".bss.DTCM"))) volatile static uint8_t uac_mic_idle_buffer_idx;
__attribute__((section(".bss.DTCM"))) volatile static uint32_t uac_speaker_buffer_full;
__attribute__((section(".bss.DTCM"))) volatile static uint32_t uac_mic_buffer_sent;

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    (void)busid;
    switch (event)
    {
    case USBD_EVENT_RESET:
        break;
    case USBD_EVENT_CONNECTED:
        break;
    case USBD_EVENT_DISCONNECTED:
        break;
    case USBD_EVENT_RESUME:
        break;
    case USBD_EVENT_SUSPEND:
        break;
    case USBD_EVENT_CONFIGURED:
        event_group_set_event(EventGroup1, EventGroup1UsbConnect);
        break;
    case USBD_EVENT_SET_REMOTE_WAKEUP:
        break;
    case USBD_EVENT_CLR_REMOTE_WAKEUP:
        break;

    default:
        break;
    }
}

volatile uint8_t dtr_enable = 0;

void usbd_audio_open(uint8_t busid, uint8_t intf)
{
    if (intf == UAC_SPEAKER_INTERFACE)
    {
        arm_fill_q15(0, (q15_t *)&uac_speaker_buffer[0][0], sizeof(uac_speaker_buffer) / sizeof(int16_t));
        uac_speaker_buffer_full = 0;
        uac_speaker_idle_buffer_idx = 1;
        usbd_ep_start_read(busid, AUDIO_OUT_EP,
                           &uac_speaker_buffer[uac_speaker_idle_buffer_idx == 0 ? 1 : 0][uac_speaker_buffer_full],
                           AUDIO_OUT_PACKET);
        event_group_set_event(EventGroup1, EventGroup1UacDataIn);
    }
    else if (intf == UAC_MIC_INTERFACE)
    {
        arm_fill_q15(0, (q15_t *)&uac_mic_buffer[0][0], sizeof(uac_mic_buffer) / sizeof(int16_t));
        uac_mic_buffer_sent = 0;
        uac_mic_idle_buffer_idx = 1;
        usbd_ep_start_write(busid, AUDIO_IN_EP,
                            &uac_mic_buffer[uac_mic_idle_buffer_idx = 0 ? 1 : 0][uac_mic_buffer_sent], AUDIO_IN_PACKET);
        event_group_set_event(EventGroup1, EventGroup1UacDataOut);
    }
}

void usbd_audio_close(uint8_t busid, uint8_t intf)
{
    (void)busid;
    if (intf == UAC_SPEAKER_INTERFACE)
    {
    }
    else if (intf == UAC_MIC_INTERFACE)
    {
    }
}

// audio out 代表主机输出，设备端输入
void usbd_audio_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)nbytes;

    uac_speaker_buffer_full += AUDIO_OUT_PACKET;

    if (uac_speaker_buffer_full ==
        MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE)
    {
        uac_speaker_buffer_full = 0;
        uac_speaker_idle_buffer_idx = uac_speaker_idle_buffer_idx == 0 ? 1 : 0;

        event_group_set_event(EventGroup1, EventGroup1UacDataIn);
    }
    else if (uac_speaker_buffer_full >
             MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST * MKF360_AUDIO_SAMPLE_SIZE)
    {
        __disable_irq();
        while (1)
            ;
    }

    usbd_ep_start_read(busid, ep,
                       &uac_speaker_buffer[uac_speaker_idle_buffer_idx == 0 ? 1 : 0][uac_speaker_buffer_full],
                       AUDIO_OUT_PACKET);
}

// audio in 代表主机输入，设备端输出
void usbd_audio_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)nbytes;

    uac_mic_buffer_sent += AUDIO_IN_PACKET;

    if (uac_mic_buffer_sent == MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST *
                                   MKF360_AUDIO_SAMPLE_SIZE * AUDIO_MIC_CHANNELS)
    {
        uac_mic_buffer_sent = 0;
        uac_mic_idle_buffer_idx = uac_mic_idle_buffer_idx == 0 ? 1 : 0;

        event_group_set_event(EventGroup1, EventGroup1UacDataOut);
    }
    else if (uac_mic_buffer_sent > MKF360_AUDIO_SAMPLE_NUM_1MS * MKF360_AUDIO_PERIPH_DMA_MS_PER_DEST *
                                       MKF360_AUDIO_SAMPLE_SIZE * AUDIO_MIC_CHANNELS)
    {
        __disable_irq();
        while (1)
            ;
    }

    usbd_ep_start_write(busid, ep, &uac_mic_buffer[uac_mic_idle_buffer_idx == 0 ? 1 : 0][uac_mic_buffer_sent],
                        AUDIO_IN_PACKET);
}

void *uac_get_mic_buffer_address()
{
    return &uac_mic_buffer[uac_mic_idle_buffer_idx][0];
}

void *uac_get_speaker_buffer_address()
{
    return &uac_speaker_buffer[uac_speaker_idle_buffer_idx][0];
}

static struct usbd_interface intf0;
static struct usbd_interface intf1;
static struct usbd_interface intf2;

static struct usbd_endpoint audio_in_ep = {
    .ep_cb = usbd_audio_in_callback,
    .ep_addr = AUDIO_IN_EP,
};

static struct usbd_endpoint audio_out_ep = {
    .ep_cb = usbd_audio_out_callback,
    .ep_addr = AUDIO_OUT_EP,
};

struct audio_entity_info audio_entity_table[] = {
    {
        .bEntityId = AUDIO_IN_FU_ID,
        .bDescriptorSubtype = AUDIO_CONTROL_FEATURE_UNIT,
        .ep = AUDIO_IN_EP,
    },
    {
        .bEntityId = AUDIO_OUT_FU_ID,
        .bDescriptorSubtype = AUDIO_CONTROL_FEATURE_UNIT,
        .ep = AUDIO_OUT_EP,
    },
};

void usb_init(uint8_t busid, uintptr_t reg_base)
{
    usbd_desc_register(busid, &usb_desc);
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf0, 0x0100, audio_entity_table,
                                                   sizeof(audio_entity_table) / sizeof(struct audio_entity_info)));
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf1, 0x0100, audio_entity_table,
                                                   sizeof(audio_entity_table) / sizeof(struct audio_entity_info)));
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf2, 0x0100, audio_entity_table,
                                                   sizeof(audio_entity_table) / sizeof(struct audio_entity_info)));
    usbd_add_endpoint(busid, &audio_in_ep);
    usbd_add_endpoint(busid, &audio_out_ep);
    usbd_initialize(busid, reg_base, usbd_event_handler);
}
