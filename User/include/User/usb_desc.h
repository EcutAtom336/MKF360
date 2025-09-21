#ifndef __USB_DESC_H__
#define __USB_DESC_H__

#include <stdbool.h>
#include <stdint.h>

extern const struct usb_descriptor usb_desc;

void usb_init(uint8_t busid, uintptr_t reg_base);

const uint8_t *device_descriptor_callback(uint8_t speed);
const uint8_t *config_descriptor_callback(uint8_t speed);
const uint8_t *device_quality_descriptor_callback(uint8_t speed);
const char *string_descriptor_callback(uint8_t speed, uint8_t index);

void usbd_event_handler(uint8_t busid, uint8_t event);

void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes);
void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes);
void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr);

#endif // !__USB_DESC_H__
