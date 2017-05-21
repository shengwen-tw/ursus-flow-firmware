#ifndef __LINK_H__
#define __LINK_H__

typedef struct {
	float flow_vx;
	float flow_vy;

	float time;
	float period;
	float frequency;

	uint16_t lidar_distance;
} fcb_data_t;

#endif
