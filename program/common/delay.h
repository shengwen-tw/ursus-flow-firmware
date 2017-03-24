#ifndef __DELAY_H__
#define __DELAY_H__

#include "FreeRTOS.h"

#define MILLI_SECOND_TICK(t) (t / portTICK_PERIOD_MS)

#define DELAY_MS_TWEAK 0

void block_delay_ms(volatile uint32_t t_ms);


#endif
