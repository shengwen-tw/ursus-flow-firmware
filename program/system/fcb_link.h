#ifndef __FCB_LINK_H__
#define __FCB_LINK_H__

typedef struct {
	uint16_t lidar_distance;

	float flow_vx;
	float flow_vy;
} fcb_data_t; 

void flight_ctrl_board_link_task(void);

#endif
