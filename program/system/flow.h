#ifndef __FLOW_H__
#define __FLOW_H__

#include "mt9v034.h"

/* disable the optical flow calcuation and directly sending the image to the PC
 * by setting this flag to 1 */
#define IMAGE_FOWARD_NO_FLOW 0

#define EDGE_PRESERVE_SIZE 4 //the offset from origin to middlemost template in the first subarea

#define FLOW_COUNT 54 //number of optical flow point to be calculated

//calculate optical flow for 54x54 points requires 62x62 pixels
#define FLOW_IMG_SIZE (FLOW_COUNT + 2 * EDGE_PRESERVE_SIZE)

#define FOCAL_LENGTH_PX 93.6f //[pixel size / cm]

#define TEMPLATE_SIZE       8  //sum of absolute difference template size
#define SEARCH_SUBAREA_SIZE 16 //only match the template in near -4 ~ +4 range
#define SUBAREA_SSD_SIZE (SEARCH_SUBAREA_SIZE - TEMPLATE_SIZE + 1) //9x9

#define TEMPLATE_MIDPOINT_OFFSET        3 //distance from template edge to its midpoint (8x8 template)
#define TEMPLATE_SEARCH_SUBAREA_OFFSET  3 //distance from search subarea edge to the middlemost template
//the offset from origin to first flow point
#define FLOW_MIDPOINT_OFFSET (TEMPLATE_MIDPOINT_OFFSET + TEMPLATE_SEARCH_SUBAREA_OFFSET)

#define HISTOGRAM_THRESHOLD 500 //max = 54 * 54 = 2916

/* since we only search the near -4 ~ +4 pixels for flow,
   there are only 9x9 matching possibilites */
#define FLOW_DISP_SIZE (SEARCH_SUBAREA_SIZE - TEMPLATE_SIZE + 1) //flow displacement possibilty size

typedef struct {
	uint16_t frame[FLOW_IMG_SIZE][FLOW_IMG_SIZE];
	uint8_t match_point[FLOW_IMG_SIZE][FLOW_IMG_SIZE];
} image_t;

typedef struct {
	uint32_t subarea_ssd_column_start[SUBAREA_SSD_SIZE][SUBAREA_SSD_SIZE];
	uint32_t subarea_ssd_last[SUBAREA_SSD_SIZE][SUBAREA_SSD_SIZE];

	image_t image[3];

	uint8_t match_x[FLOW_IMG_SIZE][FLOW_IMG_SIZE];
	uint8_t match_y[FLOW_IMG_SIZE][FLOW_IMG_SIZE];

	uint8_t distance[FLOW_IMG_SIZE][FLOW_IMG_SIZE];
} flow_t;

void flow_estimate_task(void);

#endif
