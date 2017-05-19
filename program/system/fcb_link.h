#ifndef __FCB_LINK_H__
#define __FCB_LINK_H__

typedef struct {
	float flow_vx;
	float flow_vy;

	uint16_t lidar_distance;
} fcb_data_t; 

void flight_ctrl_board_link_task(void);
void fcb_link_data_queue_init(void);

void send_flow_to_fcb(uint16_t _lidar_distance, float _flow_vx, float _flow_vy);

#endif
