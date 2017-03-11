#include "stm32f7xx.h"

#include "gpio.h"

static void led_init(void);

void gpio_init(void)
{
	led_init();
}

static void led_init(void)
{
	__HAL_RCC_GPIOD_CLK_ENABLE();

	GPIO_InitTypeDef gpio_init_struct = {
		.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6,
		.Mode = GPIO_MODE_OUTPUT_PP,
		.Pull = GPIO_PULLUP,
		.Speed = GPIO_SPEED_HIGH
	};

	HAL_GPIO_Init(GPIOD, &gpio_init_struct);

	gpio_off(LED_1);
	gpio_off(LED_2);
	gpio_off(LED_3);
}
