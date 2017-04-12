#ifndef __FLOW_H__
#define __FLOW_H__

#include "mt9v034.h"

typedef struct {
	uint16_t frame[IMG_WIDTH][IMG_HEIGHT];
} image_t;

void flow_estimate_task(void);

void give_flow_task_semaphore_from_isr(void);

#endif
