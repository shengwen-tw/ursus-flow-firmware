#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "usb_device.h"

#include "mpu9250.h"
#include "mt9v034.h"

#include "delay.h"

#include "flow.h"
#include "usb_link.h"

#define HEADER_MSG_SIZE 20

extern flow_t flow;

extern vector3d_f_t gyro_data;
extern uint16_t lidar_distance;

extern bool do_gyro_calibrate;
extern float drift_x;
extern float drift_y;
extern float drift_z;

static void usb_send_header_message(void)
{
	/* The size of the data is too big so need to be seperated into 3 part */
	char header_message[HEADER_MSG_SIZE];
	header_message[0] = '@';
	header_message[1] = 'u';
	header_message[2] = 'f';
	int append_size = 3;

	//lidar distance
	memcpy(header_message + append_size, &lidar_distance, sizeof(uint16_t));
	append_size += sizeof(uint16_t);

	if(do_gyro_calibrate == false) {
		uint8_t gyro_calib_enable = 0;
		memcpy(header_message + append_size, &gyro_calib_enable, sizeof(uint8_t));
		append_size += sizeof(uint8_t);
		//gyro x
		memcpy(header_message + append_size, &gyro_data.x, sizeof(float));
		append_size += sizeof(float);
		//gyro y
		memcpy(header_message + append_size, &gyro_data.y, sizeof(float));
		append_size += sizeof(float);
		//gyro z
		memcpy(header_message + append_size, &gyro_data.z, sizeof(float));
		append_size += sizeof(float);
	} else {
		uint8_t gyro_calib_enable = 0;
		memcpy(header_message + append_size, &gyro_calib_enable, sizeof(uint8_t));
		append_size += sizeof(uint8_t);

		//gyro drift x
		memcpy(header_message + append_size, &drift_x, sizeof(float));
		append_size += sizeof(float);
		//gyro drift y
		memcpy(header_message + append_size, &drift_y, sizeof(float));
		append_size += sizeof(float);
		//gyro drift z
		memcpy(header_message + append_size, &drift_z, sizeof(float));
		append_size += sizeof(float);
	}

	uint8_t image_width = IMG_WIDTH, image_height = IMG_HEIGHT;

	memcpy(header_message + append_size, &image_width, sizeof(uint8_t));
	append_size += sizeof(uint8_t);

	memcpy(header_message + append_size, &image_height, sizeof(uint8_t));
	append_size += sizeof(uint8_t);

	usb_cdc_send((uint8_t *)header_message, HEADER_MSG_SIZE);
}

/* foward sending the image without calculating the optical flow */
void usb_image_foward(void)
{
	/* send header message */
	usb_send_header_message();

	/* send image */
	const size_t half_image_size = sizeof(flow.image[0].frame) / 2;
	usb_cdc_send((uint8_t *)flow.image[0].frame, half_image_size);
	usb_cdc_send((uint8_t *)flow.image[0].frame + half_image_size, half_image_size);
}

#if (DISABLE_USB == 0)
/* send gyroscope data, image and flow flow visualization data */
void usb_send_flow_info(void)
{
	/* send header message */
	usb_send_header_message();

	/* send image */
	size_t send_size = sizeof(flow.image[0].frame);
	usb_cdc_send((uint8_t *)flow.image[0].frame, send_size);

	/* send image */
	send_size = sizeof(flow.match_x);
	usb_cdc_send((uint8_t *)flow.match_x, send_size);

	/* send image */
	send_size = sizeof(flow.match_y);
	usb_cdc_send((uint8_t *)flow.match_y, send_size);
}
#endif

void usb_link_task(void)
{
	usb_fs_init();

	while(1) {
		//usb_send_onboard_info();

		vTaskDelay(MILLI_SECOND_TICK(100));
	}
}
