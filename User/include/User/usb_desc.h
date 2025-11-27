#ifndef __USB_DESC_H__
#define __USB_DESC_H__

#include "usbd_core.h"

#include <stdbool.h>
#include <stdint.h>

extern const struct usb_descriptor usb_desc;

void usb_init(uint8_t busid, uintptr_t reg_base);

void *uac_get_mic_buffer_address();
void *uac_get_speaker_buffer_address();

#endif // !__USB_DESC_H__
