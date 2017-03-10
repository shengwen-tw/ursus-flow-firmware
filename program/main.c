#include "stm32f7xx.h"

#include "uart.h"

void led_on(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
}


void led_off(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
}

void gpio_init(void)
{
	__HAL_RCC_GPIOD_CLK_ENABLE();

	GPIO_InitTypeDef gpio_init_struct = {
		.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6,
		.Mode = GPIO_MODE_OUTPUT_PP,
		.Pull = GPIO_PULLUP,
		.Speed = GPIO_SPEED_HIGH
	};

	HAL_GPIO_Init(GPIOD, &gpio_init_struct);

	led_off();
}

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
			led_on();
		} else {
			led_off();
		}

		uart3_puts("Hello World\n\r");

		delay(1000000);

		state = (state + 1) % 2;
	}

	return 0;
}
