#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "gpio.h"

#include "mpu9250.h"

#include "delay.h"
#include "imu.h"

#include "usb_link.h"
#include "fcb_link.h"

extern TaskHandle_t fcb_link_task_handle;
extern TaskHandle_t usb_link_task_handle;

vector3d_f_t gyro_data;

#if (GYRO_CALIBRATE == 1)
float drift_x = 0;
float drift_y = 0;
float drift_z = 0;
#endif

void flow_estimate_task(void)
{
	/* wait until the mcu peripherial initialization is finished */
	vTaskDelay(MILLI_SECOND_TICK(5));

	if(mpu9250_init()) {
		while(1); //This is bad
	}

	/* successfully initialized the hardware == */
	gpio_on(LED_3); //red led
	vTaskResume(fcb_link_task_handle);
	vTaskResume(usb_link_task_handle);
	/* ======================================== */

#if (GYRO_CALIBRATE == 1)
	mpu9250_drift_error_estimate(&drift_x, &drift_y, &drift_z);
#endif

	int state = 1;

	while(1) {
		mpu9250_read(&gyro_data);

		if(state == 1) {
			gpio_on(LED_1);
			//gpio_on(LED_2);
		} else {
			gpio_off(LED_1);
			//gpio_off(LED_2);
		}

		state = (state + 1) % 2;

		vTaskDelay(MILLI_SECOND_TICK(100));
	}
}
