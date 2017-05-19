#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "uart.h"

#include "fcb_link.h"

#include "delay.h"

extern uint16_t lidar_distance;

QueueHandle_t fcb_link_queue_handle;

void send_flow_to_fcb(uint16_t _lidar_distance, float _flow_vx, float _flow_vy)
{
	fcb_data_t fcb_data = {
		.lidar_distance = _lidar_distance,
		.flow_vx = _flow_vx,
		.flow_vy = _flow_vy
	};

	xQueueSendToBack(fcb_link_queue_handle, &fcb_data, 0);
}

void flight_ctrl_board_link_task(void)
{
	fcb_link_queue_handle = xQueueCreate(16, sizeof(fcb_link_queue_handle));

	char buffer[256] = {'\0'};

	while(1) {
		sprintf(buffer, "%d\n\r", lidar_distance);

		uart2_puts(buffer);

		vTaskDelay(MILLI_SECOND_TICK(10));
	}
}

