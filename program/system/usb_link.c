#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "usb_device.h"

#include "mpu9250.h"
#include "mt9v034.h"

#include "delay.h"

#include "usb_link.h"

extern uint16_t image_buffer[IMG_WIDTH][IMG_HEIGHT];

extern vector3d_f_t gyro_data;

extern float drift_x;
extern float drift_y;
extern float drift_z;

int send_mode = USB_SEND_IMAGE;

static void usb_send_image(void)
{
#if 0
	uint16_t image_size = sizeof(image_buffer);

	/* send header */
	uint8_t header[5];
	header[0] = '@'; //header[0:2] = start symbols "@uf"
	header[1] = 'u';
	header[2] = 'f';
	header[3] = image_size >> 8; //header[3:4] = image size
	header[4] = image_size;
	usb_cdc_send((uint8_t *)header, sizeof(header));
#endif

	/* send image */
	usb_cdc_send((uint8_t *)image_buffer, sizeof(image_buffer));
}

static void usb_send_gyro(void)
{
	char str[256] = {'\0'};
	sprintf(str,
	        "gyroscope x:%f y:%f z:%f\n\r"
	        "\x1b[H\x1b[2J",
	        gyro_data.x,
	        gyro_data.y,
	        gyro_data.z);

	usb_cdc_send((uint8_t *)str, strlen(str));
}

static void usb_send_gyro_calibrate(void)
{
	char str[256] = {'\0'};
	sprintf(str,
	        "gyroscope drift x:%f y:%f z:%f\n\r"
	        "\x1b[H\x1b[2J",
	        drift_x,
	        drift_y,
	        drift_z);

	usb_cdc_send((uint8_t *)str, strlen(str));
}

void usb_link_task(void)
{
	usb_fs_init();

	while(1) {
		switch(send_mode) {
		case USB_SEND_IMAGE:
			usb_send_image();
			break;
		case USB_SEND_GYRO:
			usb_send_gyro();
			break;
		case USB_SEND_GYRO_CALIB:
			usb_send_gyro_calibrate();
			break;
		}

		vTaskDelay(100);
	}
}
