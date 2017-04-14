#include "stm32f7xx.h"

#include "gpio.h"
#include "interrupt.h"

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

void exti3_init(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef gpio_exit3 = {
		.Mode = GPIO_MODE_IT_FALLING,
		.Pull = GPIO_NOPULL,
		.Pin = GPIO_PIN_3
	};

	HAL_GPIO_Init(GPIOC, &gpio_exit3);

	HAL_NVIC_SetPriority(EXTI3_IRQn, EXTI3_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}
