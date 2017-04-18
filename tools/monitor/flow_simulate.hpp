#ifndef __FLOW_SIMULATE_H__
#define  __FLOW_SIMULATE_H__

#define FLOW_IMG_SIZE 72 //calculate optical flow for 64x64 points requires 72x72 pixels

#define FLOW_COUNT          64 //number of optical flow point to be calculated
#define TEMPLATE_SIZE       8  //sum of absolute difference template size
#define SEARCH_SUBAREA_SIZE 16 //only match the template in near -4 ~ +4 range

#define TEMPLATE_MIDPOINT_OFFSET        3 //distance from template edge to its midpoint (8x8 template)
#define TEMPLATE_SEARCH_SUBAREA_OFFSET  3 //distance from search subarea edge to the middlemost template

#define FLOW_THRESHOLD 300 //64x64 = 4096

typedef struct {
	uint16_t frame[FLOW_IMG_SIZE][FLOW_IMG_SIZE];
	uint8_t match_point[FLOW_IMG_SIZE][FLOW_IMG_SIZE];
} image_t;

typedef struct {
	image_t image[2];

	uint8_t match_x[FLOW_IMG_SIZE][FLOW_IMG_SIZE];
	uint8_t match_y[FLOW_IMG_SIZE][FLOW_IMG_SIZE];

	uint8_t distance[FLOW_IMG_SIZE][FLOW_IMG_SIZE];
} flow_t;

void simulate_opical_flow_on_pc();

#endif
