#ifndef __FCB_LINK_H__
#define __FCB_LINK_H__

#include "uart.h"

#define send_debug_message(s, ...) \
	char buffer[256] = {'\0'}; \
	int size = sprintf(buffer, s, ##__VA_ARGS__); \
	SCB_CleanDCache_by_Addr((uint32_t *)buffer, 256); \
	uart2_puts(buffer, size); \

void send_flow_to_fcb(uint16_t *lidar_distance, float *flow_vx,
                      float *flow_vy, float *time, float *period, float *frequency);

#endif
