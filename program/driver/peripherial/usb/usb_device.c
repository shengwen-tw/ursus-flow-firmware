#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include "FreeRTOS.h"
#include "semphr.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

USBD_HandleTypeDef hUsbDeviceFS;

SemaphoreHandle_t usb_tx_semaphore;

/* Setup fast speed usb and register the class as cdc */
void usb_fs_init(void)
{
	/* Create semaphore for fast speed usb resource */
	usb_tx_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(usb_tx_semaphore);

	USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
	USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
	USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
	USBD_Start(&hUsbDeviceFS);
}

void OTG_FS_IRQHandler(void)
{
	long higher_priority_task_woken = pdFALSE;

	HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);

	USBD_CDC_HandleTypeDef *usb_cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
	if(usb_cdc->TxState == 0) {
		xSemaphoreGiveFromISR(usb_tx_semaphore, &higher_priority_task_woken);
		portYIELD_FROM_ISR(higher_priority_task_woken);
	}
}

void usb_cdc_send(uint8_t *buf, uint16_t len)
{
	xSemaphoreTake(usb_tx_semaphore, portMAX_DELAY);

	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, buf, len);
	USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}
