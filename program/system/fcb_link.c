#include <string.h>

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "uart.h"

#include "fcb_link.h"

#include "delay.h"

#define PACKET_SIZE 23

extern uint16_t lidar_distance;

QueueHandle_t fcb_link_queue_handle;

fcb_data_t fcb_data;

void fcb_link_data_queue_init(void)
{
	fcb_link_queue_handle = xQueueCreate(16, sizeof(fcb_data_t));
}

void send_flow_to_fcb(uint16_t _lidar_distance, float _flow_vx, float _flow_vy,
                      float _time, float _period, float _frequency)
{
	fcb_data_t _fcb_data = {
		.lidar_distance = _lidar_distance,
		.flow_vx = _flow_vx,
		.flow_vy = _flow_vy,
		.time = _time,
		.period = _period,
		.frequency = _frequency
	};

	xQueueSendToBack(fcb_link_queue_handle, &_fcb_data, 0);
}

__attribute__((unused)) static void send_debug_message(void)
{
	char buffer[256] = {'\0'};

	/* debug mode */
	int size = sprintf(buffer, "lidar:%3d, vx:%+2.3f, vy:%+2.3f, time:%.1f, fps:%.1f\n\r",
	                   fcb_data.lidar_distance,
	                   fcb_data.flow_vx,
	                   fcb_data.flow_vy,
	                   fcb_data.time,
	                   fcb_data.frequency
	                  );

	uart2_puts(buffer, size);
}

static void send_onboard_params(void)
{
	char start_byte = '$';
	char message[PACKET_SIZE] = {0};
	int append_size = 0;

	memcpy(message + append_size, &start_byte, sizeof(char));
	append_size += sizeof(char);

	memcpy(message + append_size, &fcb_data.lidar_distance, sizeof(uint16_t));
	append_size += sizeof(uint16_t);

	memcpy(message + append_size, &fcb_data.flow_vx, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, &fcb_data.flow_vy, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, &fcb_data.time, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, &fcb_data.period, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, &fcb_data.frequency, sizeof(float));
	append_size += sizeof(float);

	uart2_puts(message, append_size + 1);
}

void flight_ctrl_board_link_task(void)
{
	while(1) {
		if(xQueueReceive(fcb_link_queue_handle, &fcb_data, portMAX_DELAY) == pdTRUE) {
			send_onboard_params();
		}
	}
}

