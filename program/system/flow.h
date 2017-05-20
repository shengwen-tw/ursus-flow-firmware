#ifndef __FLOW_H__
#define __FLOW_H__

#include "mt9v034.h"

typedef struct {
	uint16_t frame[IMG_WIDTH][IMG_HEIGHT];
} image_t;

typedef struct {
	image_t image[2];
} flow_t;

void flow_estimate_task(void);

#endif
