#include "User/usb_desc.h"

#include "usbd_core.h"
//
#include "usbd_audio.h"
#include "usbd_cdc_acm.h"

#include "audio/PCM_RES.h"

#ifndef CONFIG_USBDEV_ADVANCE_DESC
#error "Please enable CONFIG_USBDEV_ADVANCE_DESC macro."
#endif

#define USBD_VID 0xFFFF
#define USBD_PID 0xFFFF
#define USBD_MAX_POWER 100
#define USBD_LANGID_STRING 1033

#define CDC_IN_EP 0x81
#define CDC_OUT_EP 0x01
#define CDC_INT_EP 0x82
#define AUDIO_IN_EP 0x83
#define AUDIO_OUT_EP 0x03

#define EP_INTERVAL 0x01
#define FEEDBACK_ENDP_PACKET_SIZE 0x03

#define AUDIO_IN_FU_ID 0x02
#define AUDIO_OUT_FU_ID 0x05

#define AUDIO_SPEAKER_FREQ 16000U
#define AUDIO_SPEAKER_FRAME_SIZE_BYTE 2u
#define AUDIO_SPEAKER_RESOLUTION_BIT 16u
#define AUDIO_SPEAKER_CHANNELS 1u
#define AUDIO_MIC_FREQ 16000U
#define AUDIO_MIC_FRAME_SIZE_BYTE 2u
#define AUDIO_MIC_RESOLUTION_BIT 16u
#define AUDIO_MIC_CHANNELS 1u

#define AUDIO_SAMPLE_FREQ(frq) (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

#define AUDIO_SAMPLE_FREQ(frq) (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

#define AUDIO_OUT_PACKET                                                                                               \
    ((uint32_t)((AUDIO_SPEAKER_FREQ * AUDIO_SPEAKER_FRAME_SIZE_BYTE * AUDIO_SPEAKER_CHANNELS) / 1000))
#define AUDIO_IN_PACKET ((uint32_t)((AUDIO_MIC_FREQ * AUDIO_MIC_FRAME_SIZE_BYTE * AUDIO_MIC_CHANNELS) / 1000))

// clang-format off

#define USB_CONFIG_SIZE (9 + \
                         CDC_ACM_DESCRIPTOR_LEN + \
                         AUDIO_AC_DESCRIPTOR_INIT_LEN(2) + \
                         AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC + \
                         AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(AUDIO_MIC_CHANNELS, 1) + \
                         AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC + \
                         AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC + \
                         AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(AUDIO_SPEAKER_CHANNELS, 1) + \
                         AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC + \
                         AUDIO_AS_DESCRIPTOR_INIT_LEN(1) + \
                         AUDIO_AS_DESCRIPTOR_INIT_LEN(1))

const uint16_t usb_desc_size = USB_CONFIG_SIZE;

#define AUDIO_AC_SIZ (AUDIO_SIZEOF_AC_HEADER_DESC(2) + \
                      AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC + \
                      AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(AUDIO_MIC_CHANNELS, 1) + \
                      AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC + \
                      AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC + \
                      AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(AUDIO_SPEAKER_CHANNELS, 1) + \
                      AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC)

// clang-format on

#define CDC_MAX_MPS 64

static const uint8_t device_descriptor[] = {
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

static const uint8_t config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE,        // bLength
                               0x05,                   // bNumInterfaces
                               0x01,                   // bConfigurationValue
                               USB_CONFIG_BUS_POWERED, // bmAttributes
                               USBD_MAX_POWER          // bMaxPower
                               ),
    //
    CDC_ACM_DESCRIPTOR_INIT(0x00,        // bFirstInterface
                            CDC_INT_EP,  // int_ep
                            CDC_OUT_EP,  // out_ep
                            CDC_IN_EP,   // in_ep
                            CDC_MAX_MPS, // wMaxPacketSize
                            0x02         // str_idx
                            ),
    //
    AUDIO_AC_DESCRIPTOR_INIT(0x02,         // bFirstInterface
                             0x03,         // bInterfaceCount
                             AUDIO_AC_SIZ, // wTotalLength
                             0x00,         // stridx
                             0x03,         //
                             0x04          //
                             ),

    AUDIO_AC_INPUT_TERMINAL_DESCRIPTOR_INIT(0x01,               // bTerminalID
                                            AUDIO_INTERM_MIC,   // wTerminalType
                                            AUDIO_MIC_CHANNELS, // bNrChannels
                                            0x0000              // wChannelConfig
                                            ),
    AUDIO_AC_FEATURE_UNIT_DESCRIPTOR_INIT(0x02, // bUnitID
                                          0x01, // bSourceID
                                          0x01, // bControlSize
                                          0x03, //
                                          0x00  //
                                          ),
    AUDIO_AC_OUTPUT_TERMINAL_DESCRIPTOR_INIT(0x03,                     // bTerminalID
                                             AUDIO_TERMINAL_STREAMING, // wTerminalType
                                             0x02                      // bSourceID
                                             ),

    AUDIO_AC_INPUT_TERMINAL_DESCRIPTOR_INIT(0x04,                     // bTerminalID
                                            AUDIO_TERMINAL_STREAMING, // wTerminalType
                                            AUDIO_SPEAKER_CHANNELS,   // bNrChannels
                                            0x0000                    // wChannelConfig
                                            ),
    AUDIO_AC_FEATURE_UNIT_DESCRIPTOR_INIT(0x05, // bUnitID
                                          0x04, // bSourceID
                                          0x01, // bControlSize
                                          0x03, //
                                          0x00  //
                                          ),
    AUDIO_AC_OUTPUT_TERMINAL_DESCRIPTOR_INIT(0x06,                  // bTerminalID
                                             AUDIO_OUTTERM_SPEAKER, // wTerminalType
                                             0x05                   // bSourceID
                                             ),

    AUDIO_AS_DESCRIPTOR_INIT(0x03,                                    // bInterfaceNumber
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
    AUDIO_AS_DESCRIPTOR_INIT(0x04,                                // bInterfaceNumber
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
};

static const uint8_t device_quality_descriptor[] = {
    0x0a, USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
};

static const char *string_descriptors[] = {
    (const char[]){0x09, 0x04}, /* Langid */
    "OSHWHub",                  /* Manufacturer */
    "MKF360",                   /* Product */
    "88888888",                 /* Serial Number */
};

const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_descriptor;
}

const uint8_t *config_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return config_descriptor;
}

const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_quality_descriptor;
}

const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;
    if (index > 3)
    {
        return NULL;
    }
    return string_descriptors[index];
}

const struct usb_descriptor usb_desc = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
};

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_read_buffer[CDC_MAX_MPS];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_write_buffer[CDC_MAX_MPS];

volatile bool cdc_ep_tx_busy_flag = false;
volatile bool uac_ep_tx_busy_flag = false;

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t uac_read_buffer[AUDIO_OUT_PACKET];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t uac_write_buffer[AUDIO_IN_PACKET];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t s_speaker_feedback_buffer[4];

volatile bool tx_flag = 0;
volatile bool rx_flag = 0;
volatile uint32_t s_mic_sample_rate;
volatile uint32_t s_speaker_sample_rate;

void usbd_event_handler(uint8_t busid, uint8_t event)
{
    (void)busid;
    switch (event)
    {
    case USBD_EVENT_RESET:
        break;
    case USBD_EVENT_CONNECTED:
        cdc_ep_tx_busy_flag = false;
        usbd_ep_start_read(busid, CDC_OUT_EP, cdc_read_buffer, CDC_MAX_MPS);
        break;
    case USBD_EVENT_DISCONNECTED:
        break;
    case USBD_EVENT_RESUME:
        break;
    case USBD_EVENT_SUSPEND:
        break;
    case USBD_EVENT_CONFIGURED:
        break;
    case USBD_EVENT_SET_REMOTE_WAKEUP:
        break;
    case USBD_EVENT_CLR_REMOTE_WAKEUP:
        break;

    default:
        break;
    }
}

void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)ep;
    (void)nbytes;
    usbd_ep_start_read(busid, CDC_OUT_EP, cdc_read_buffer, CDC_MAX_MPS);
}

void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes)
    {
        /* send zlp */
        usbd_ep_start_write(busid, CDC_IN_EP, NULL, 0);
    }
    else
    {
        cdc_ep_tx_busy_flag = false;
    }
}

volatile uint8_t dtr_enable = 0;

void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr)
{
    (void)busid;
    (void)intf;
    (void)dtr;
    if (dtr)
    {
        dtr_enable = 1;
    }
    else
    {
        dtr_enable = 0;
    }
}

void usbd_audio_open(uint8_t busid, uint8_t intf)
{
    if (intf == 0x03)
    {
        rx_flag = 1;
        usbd_ep_start_read(busid, AUDIO_OUT_EP, uac_read_buffer, AUDIO_OUT_PACKET);
        printf("OPEN1\r\n");
    }
    else
    {
        tx_flag = 1;
        uac_ep_tx_busy_flag = false;
        usbd_ep_start_write(busid, AUDIO_IN_EP, uac_write_buffer, AUDIO_IN_PACKET);
        printf("OPEN2\r\n");
    }
}

void usbd_audio_close(uint8_t busid, uint8_t intf)
{
    (void)busid;
    if (intf == 1)
    {
        rx_flag = 0;
        printf("CLOSE1\r\n");
    }
    else
    {
        tx_flag = 0;
        uac_ep_tx_busy_flag = false;
        printf("CLOSE2\r\n");
    }
}

void usbd_audio_set_sampling_freq(uint8_t busid, uint8_t ep, uint32_t sampling_freq)
{
    (void)busid;
    if (ep == AUDIO_OUT_EP)
    {
        s_speaker_sample_rate = sampling_freq;
    }
    else if (ep == AUDIO_IN_EP)
    {
        s_mic_sample_rate = sampling_freq;
    }
}

uint32_t usbd_audio_get_sampling_freq(uint8_t busid, uint8_t ep)
{
    (void)busid;

    uint32_t freq = 0;

    if (ep == AUDIO_OUT_EP)
    {
        freq = s_speaker_sample_rate;
    }
    else if (ep == AUDIO_IN_EP)
    {
        freq = s_mic_sample_rate;
    }

    return freq;
}

void usbd_audio_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;
    usbd_ep_start_read(busid, AUDIO_OUT_EP, uac_read_buffer, AUDIO_OUT_PACKET);
}

static uint32_t sent_bytes = 0;
void usbd_audio_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;

    memcpy(uac_write_buffer, (void *)BATTERY_LOW_PCM + sent_bytes, AUDIO_IN_PACKET);
    sent_bytes += AUDIO_IN_PACKET;
    if (sent_bytes + AUDIO_IN_PACKET >= sizeof(BATTERY_LOW_PCM))
    {
        sent_bytes = 0;
    }
    usbd_ep_start_write(busid, AUDIO_IN_EP, uac_write_buffer, AUDIO_IN_PACKET);

    uac_ep_tx_busy_flag = false;
}

static struct usbd_interface intf0;
static struct usbd_interface intf1;
static struct usbd_interface intf2;
static struct usbd_interface intf3;
static struct usbd_interface intf4;

struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out,
};

struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in,
};

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
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf0));
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1));
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf2, 0x0100, audio_entity_table, 2));
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf3, 0x0100, audio_entity_table, 2));
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf4, 0x0100, audio_entity_table, 2));
    usbd_add_endpoint(busid, &cdc_out_ep);
    usbd_add_endpoint(busid, &cdc_in_ep);
    usbd_add_endpoint(busid, &audio_in_ep);
    usbd_add_endpoint(busid, &audio_out_ep);
    usbd_initialize(busid, reg_base, usbd_event_handler);
}
