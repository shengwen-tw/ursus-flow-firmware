#include <stdbool.h>

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "gpio.h"
#include "timer.h"

#include "mpu9250.h"
#include "mt9v034.h"
#include "lidar.h"

#include "delay.h"
#include "imu.h"

#include "flow.h"
#include "usb_link.h"
#include "fcb_link.h"
#include "system_time.h"

extern TaskHandle_t fcb_link_task_handle;
extern TaskHandle_t usb_link_task_handle;

SemaphoreHandle_t flow_task_semaphore;

flow_t flow;

vector3d_f_t gyro_data;
uint16_t lidar_distance;

bool do_gyro_calibrate = false; //set true to eanble the calibration function

float drift_x = 0;
float drift_y = 0;
float drift_z = 0;

void flow_estimate_task(void)
{
	/* wait until the mcu peripherial initialization is finished */
	vTaskDelay(MILLI_SECOND_TICK(5));

	if(mpu9250_init()) {
		while(1); //This is bad
	}

	if(mt9v034_init()) {
		while(1); //This is bad
	}

	lidar_init(&lidar_distance);

	/* successfully initialized the hardware == */
	gpio_on(LED_3); //red led
	vTaskResume(fcb_link_task_handle);
	//vTaskResume(usb_link_task_handle); //XXX: remember to unblock me later
	/* ======================================== */

	if(do_gyro_calibrate == true) {
		mpu9250_drift_error_estimate(&drift_x, &drift_y, &drift_z);
	}

	timer_init();

	/* XXX: usb direct camera output */
	usb_fs_init();

	float current_time;
	float previous_time;
	float delta_t;
	float fps;

	int next = 0;

	mt9v034_start_capture_image((uint32_t)flow.image[0].frame);
	mt9v034_wait_finish();
	previous_time = get_time_sec();

	mt9v034_start_capture_image((uint32_t)flow.image[1].frame);

	while(1) {
		//gpio_off(LED_1);

		mpu9250_read(&gyro_data);

		/* wait until image finished capturing */
		mt9v034_wait_finish();

		current_time = get_time_sec();
		delta_t = current_time - previous_time;
		previous_time = current_time;
		fps = 1.0f / delta_t;

		//flow_estimate()

#if 0
		mt9v034_start_capture_image((uint32_t)image[next].frame);
#else
		mt9v034_start_capture_image((uint32_t)flow.image[0].frame);
		usb_send_onboard_info();
#endif
		next = (next + 1) % 2;

		send_flow_to_fcb(lidar_distance, 0.0f, 0.0f, current_time, delta_t, fps);

		//gpio_on(LED_1);
		gpio_toggle(LED_1);
	}
}
