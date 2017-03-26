#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "usb_device.h"

#include "mpu9250.h"

#include "delay.h"

#include "usb_link.h"

extern vector3d_f_t gyro_data;

extern float drift_x;
extern float drift_y;
extern float drift_z;

void usb_link_task(void)
{
	usb_fs_init();
	char str[256] = {'\0'};

	while(1) {
#if (GYRO_CALIBRATE == 1)
		sprintf(str,
		        "gyroscope drift x:%f y:%f z:%f\n\r"
		        "\x1b[H\x1b[2J",
		        drift_x,
		        drift_y,
		        drift_z);
#endif

#if (GYRO_PRINT == 1)
		sprintf(str,
		        "gyroscope x:%f y:%f z:%f\n\r"
		        "\x1b[H\x1b[2J",
		        gyro_data.x,
		        gyro_data.y,
		        gyro_data.z);
#endif

		usb_cdc_send((uint8_t *)str, strlen(str));

		vTaskDelay(100);
	}
}
