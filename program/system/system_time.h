#ifndef __SYSTEM_TIMER_H__
#define __SYSTEM_TIMER_H__

#include <stdint.h>

#define TICK_FREQUENCY 10000
#define TICK_PERIOD    0.0001

typedef struct {
	uint32_t sec;
	float sec_remainder;

	uint32_t tick_number;
} system_time_t;

void update_system_time(void);
float get_time_sec(void);

void delay_ms(float ms);

#endif
