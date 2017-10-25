#include <string.h>

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "uart.h"

#include "fcb_link.h"

#define PACKET_SIZE 46

static void calculate_checksum(uint8_t *checksum, uint8_t *data, int size)
{
	int i;
	for(i = 0; i < size; i++) {
		*checksum ^= data[i];
	}
}

__attribute__((section(".itcmtext")))
void send_flow_to_fcb(float *lidar_distance, float *lidar_velocity,
		      float *flow_vx, float *flow_vy,
		      float *accel_x, float *accel_y, float *accel_z,
                      float *time, float *period, float *frequency,
		      float *quality_vx, float *quality_vy)
{
	/* pack message */
	char start_byte = '+';
	char message[PACKET_SIZE] = {0};
	int append_size = 0;

	uint8_t checksum = 19; //initial checksum seed

	memcpy(message + append_size, &start_byte, sizeof(char));
	append_size += sizeof(char);

	memcpy(message + append_size, lidar_distance, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, lidar_velocity, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, flow_vx, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, flow_vy, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, accel_x, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, accel_y, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, accel_z, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, time, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, period, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, frequency, sizeof(float));
	append_size += sizeof(float);

	memcpy(message + append_size, quality_vx, sizeof(float));
	append_size += sizeof(float);

//	memcpy(message + append_size, quality_vy, sizeof(float));
//	append_size += sizeof(float);

	calculate_checksum(&checksum, (uint8_t *)message, PACKET_SIZE - 1);
	memcpy(message + append_size, &checksum, sizeof(uint8_t));

	uart2_puts(message, PACKET_SIZE);
}
