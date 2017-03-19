#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "core.h"
#include "gpio.h"
#include "uart.h"
#include "usb_device.h"

#include "delay.h"

void test_task1(void)
{
	int state = 1;

	while(1) {
		if(state == 1) {
			gpio_on(LED_1);
			gpio_on(LED_2);
			gpio_on(LED_3);
		} else {
			gpio_off(LED_1);
			gpio_off(LED_2);
			gpio_off(LED_3);
		}

		state = (state + 1) % 2;

		vTaskDelay(MILLI_SECOND_TICK(500));
	}
}

void test_task2(void)
{
	while(1) {
		uart2_puts("Hello World\n\r");

		vTaskDelay(MILLI_SECOND_TICK(500));
	}
}


int main(void)
{
	mpu_init();
	cpu_cache_enable();
	HAL_Init();
	system_clock_init();

	gpio_init();
	uart_init();

	usb_fs_init();

	xTaskCreate((TaskFunction_t)test_task1, "blinky1",
		1024, (void *)0, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate((TaskFunction_t)test_task2, "print",
		1024, (void *)0, tskIDLE_PRIORITY + 2, NULL);
		
	vTaskStartScheduler();

	return 0;
}
