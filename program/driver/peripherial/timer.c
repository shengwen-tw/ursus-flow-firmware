#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"

#include "interrupt.h"

#include "flow.h"
#include "system_time.h"

static void timer10_init(void);

TIM_HandleTypeDef timer10;

void timer_init(void)
{
	timer10_init();
}

/* timer period: 100us */
static void timer10_init(void)
{
	__HAL_RCC_TIM10_CLK_ENABLE();

	timer10.Instance = TIM10;
	timer10.Init.Prescaler = 216 - 1;
	timer10.Init.CounterMode = TIM_COUNTERMODE_UP;
	timer10.Init.Period = 100 - 1;
	timer10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&timer10) != HAL_OK) {
		//Error_Handler();
	}

	HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, TIMER10_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	HAL_TIM_Base_Start_IT(&timer10);
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&timer10);

	update_system_time();
}
