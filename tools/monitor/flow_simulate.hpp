#ifndef __FLOW_SIMULATE_H__
#define  __FLOW_SIMULATE_H__

#define FLOW_IMG_WIDTH  72
#define FLOW_IMG_HEIGHT 72
#define FLOW_IMG_SIZE   (FLOW_IMG_WIDTH * FLOW_IMG_HEIGHT)

typedef struct {
	uint16_t frame[FLOW_IMG_WIDTH][FLOW_IMG_HEIGHT];
} image_t;

typedef struct {
	image_t image[2];
} flow_t;

void simulate_opical_flow_on_pc();

#endif
