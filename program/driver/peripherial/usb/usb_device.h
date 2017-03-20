#ifndef __usb_device_H
#define __usb_device_H

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "usbd_def.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

void usb_fs_init(void);
void usb_cdc_send(uint8_t *buf, uint16_t len);

#endif
