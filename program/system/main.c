#include "stm32f7xx.h"

#include "core.h"
#include "gpio.h"
#include "uart.h"

void delay(uint32_t count)
{
        while(count--) {
		__asm volatile("nop");
	}
}

int main(void)
{
	gpio_init();
	uart_init();

	int state = 1;

	while(1) {
		if(state == 1) {
			gpio_on(LED_1);
			gpio_on(LED_2);
			gpio_on(LED_3);
		} else {
			gpio_off(LED_1);
			gpio_off(LED_2);
			gpio_off(LED_3);
		}

		uart3_puts("Hello World\n\r");

		delay(1000000);

		state = (state + 1) % 2;
	}

	return 0;
}
