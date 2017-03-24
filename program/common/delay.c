#include "delay.h"

void block_delay_ms(uint32_t t_ms)
{
#if 0
	volatile uint32_t cnt = t_ms * DELAY_MS_TWEAK;

	while(cnt--) {
		__asm("nop");
	}
#endif
}
