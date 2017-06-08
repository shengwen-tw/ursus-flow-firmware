#include <string.h>

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "uart.h"

#include "fcb_link.h"

#define PACKET_SIZE 23

void send_flow_to_fcb(uint16_t *lidar_distance, float *flow_vx, float *flow_vy,
                      float *time, float *period, float *frequency)
{
	/* pack message */
	char start_byte = '$';
	char message[PACKET_SIZE] = {0};
	int append_size = 0;

	memcpy(message + append_size, &start_byte, sizeof(char));
	append_size += sizeof(char);

	memcpy(message + append_size, lidar_distance, sizeof(uint16_t));
	append_size += sizeof(uint16_t);

	memcpy(message + append_size, flow_vx, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, flow_vy, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, time, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, period, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, frequency, sizeof(float));
	append_size += sizeof(float);

	uart2_puts(message, append_size + 1);
}
