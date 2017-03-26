#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "uart.h"

#include "delay.h"

void flight_ctrl_board_link_task(void)
{
	while(1) {
		uart2_puts("Hello World\n\r");

		vTaskDelay(MILLI_SECOND_TICK(500));
	}
}

