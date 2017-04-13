#ifndef __DCMI_H__
#define __DCMI_H__

#include <stdbool.h>

void dcmi_init(void);

void dcmi_dma_config(uint32_t buffer_address, uint32_t image_width, uint32_t image_height);
void dcmi_wait_finish(void);

#endif
