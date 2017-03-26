#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "core.h"
#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "usb_device.h"

#include "mpu9250.h"

#include "delay.h"
#include "imu.h"

#include "usb_link.h"
#include "fcb_link.h"

TaskHandle_t fcb_link_task_handle;
TaskHandle_t usb_link_task_handle;

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

	/* successfully initialize the hardware == */
	gpio_on(LED_3); //red led
	vTaskResume(fcb_link_task_handle);
	vTaskResume(usb_link_task_handle);
	/* ======================================= */

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

int main(void)
{
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	mpu_init();
	cpu_cache_enable();
	system_clock_init();

	gpio_init();
	uart_init();
	spi_init();

	xTaskCreate((TaskFunction_t)flow_estimate_task, "flow estimate task",
	            1024, (void *)0, tskIDLE_PRIORITY + 3, NULL);

	xTaskCreate((TaskFunction_t)flight_ctrl_board_link_task,
	            "flight control board link task",
	            1024, (void *)0, tskIDLE_PRIORITY + 2, &fcb_link_task_handle);
	vTaskSuspend(fcb_link_task_handle);

	xTaskCreate((TaskFunction_t)usb_link_task, "usb link task",
	            1024, (void *)0, tskIDLE_PRIORITY + 1, &usb_link_task_handle);
	vTaskSuspend(usb_link_task_handle);

	vTaskStartScheduler();

	return 0;
}
