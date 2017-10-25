#ifndef __FCB_LINK_H__
#define __FCB_LINK_H__

#include "uart.h"

#define send_debug_message(s, ...) \
	char buffer[256] = {'\0'}; \
	int size = sprintf(buffer, s, ##__VA_ARGS__); \
	uart2_puts(buffer, size); \


void send_flow_to_fcb(float *lidar_distance, float *lidar_velocity,
                      float *flow_vx, float *flow_vy,
                      float *accel_x, float *accel_y, float *accel_z,
                      float *time, float *period, float *frequency,
		      float *quality_vx, float *quality_vy);

#endif
