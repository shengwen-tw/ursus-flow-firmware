#include "delay.h"

#define STM32_CLOCK_HZ 216000000UL
#define DELAY_TWEAK 10000

void block_delay_ms(uint32_t ms)
{
	ms *= STM32_CLOCK_HZ / DELAY_TWEAK;

	asm volatile(" mov r0, %[ms] \n\t"
	             "1: subs r0, #1 \n\t"
	             " bhi 1b \n\t"
	             :
	             : [ms] "r" (ms)
	             : "r0");
}
