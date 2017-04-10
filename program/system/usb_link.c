#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "usb_device.h"

#include "mpu9250.h"
#include "mt9v034.h"

#include "delay.h"

#include "usb_link.h"

#define HEADER_MSG_SIZE 18

extern uint16_t image_buffer[IMG_WIDTH][IMG_HEIGHT];

extern vector3d_f_t gyro_data;
extern uint16_t lidar_distance;

extern bool do_gyro_calibrate;
extern float drift_x;
extern float drift_y;
extern float drift_z;

static void usb_send_onboard_info(void)
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

	usb_cdc_send((uint8_t *)header_message, HEADER_MSG_SIZE);

	/* image */
	const size_t half_image_size = sizeof(image_buffer) / 2;
	usb_cdc_send((uint8_t *)image_buffer, half_image_size);
	usb_cdc_send((uint8_t *)image_buffer + half_image_size, half_image_size);
}

void usb_link_task(void)
{
	usb_fs_init();

	while(1) {
		usb_send_onboard_info();

		vTaskDelay(MILLI_SECOND_TICK(100));
	}
}
