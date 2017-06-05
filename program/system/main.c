#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "core.h"
#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "timer.h"
#include "pwm.h"
#include "i2c.h"
#include "dcmi.h"

#include "mpu9250.h"
#include "mt9v034.h"

#include "flow.h"
#include "usb_link.h"
#include "fcb_link.h"

TaskHandle_t fcb_link_task_handle;
TaskHandle_t usb_link_task_handle;

int main(void)
{
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	mpu_init();
	cpu_cache_enable();
	system_clock_init();

	gpio_init();
	uart_init();
	spi_init();
	pwm_init();
	i2c_init();
	dcmi_init();

	xTaskCreate((TaskFunction_t)flow_estimate_task, "flow estimate task",
	            1024, (void *)0, tskIDLE_PRIORITY + 3, NULL);

	vTaskStartScheduler();

	return 0;
}
