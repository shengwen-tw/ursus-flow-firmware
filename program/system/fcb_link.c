#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "uart.h"

#include "delay.h"

extern uint16_t lidar_distance;

void flight_ctrl_board_link_task(void)
{
	char buffer[256] = {'\0'};

	while(1) {
		sprintf(buffer, "%d\n\r", lidar_distance);

		uart2_puts(buffer);

		vTaskDelay(MILLI_SECOND_TICK(10));
	}
}

