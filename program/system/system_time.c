#include "system_time.h"

system_time_t system_time = {
	.sec = 0,
	.sec_remainder = 0.0f,
	.tick_number = 0
};

void update_system_time(void)
{
	system_time.tick_number++;
	system_time.sec_remainder = (float)system_time.tick_number * TICK_PERIOD;

	if(system_time.tick_number == TICK_FREQUENCY) {
		system_time.tick_number = 0;
		system_time.sec_remainder = 0.0f;
		system_time.sec++;
	}
}

float get_time_sec(void)
{
	return (float)system_time.sec + system_time.sec_remainder;
}
