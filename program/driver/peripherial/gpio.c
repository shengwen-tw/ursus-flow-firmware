#include "stm32f7xx.h"

#include "gpio.h"
#include "interrupt.h"

static void led_init(void);
static void exti3_init(void);

void gpio_init(void)
{
	led_init();
	exti3_init();
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

static void exti3_init(void)
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

void EXTI3_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);

		gpio_on(LED_2);
	}
}
