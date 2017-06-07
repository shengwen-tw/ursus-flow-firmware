#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

USBD_HandleTypeDef hUsbDeviceFS;

/* Setup fast speed usb and register the class as cdc */
void usb_fs_init(void)
{
	USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
	USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
	USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
	USBD_Start(&hUsbDeviceFS);
}

void OTG_FS_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

void usb_cdc_send(uint8_t *buf, uint16_t len)
{
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, buf, len);
	while(USBD_CDC_TransmitPacket(&hUsbDeviceFS) != 0);
}

int usb_cdc_connected(void)
{
	return USBD_LL_DevConnected(&hUsbDeviceFS);
}
