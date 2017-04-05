#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"

#include "flow.h"

static void timer10_init(void);

TIM_HandleTypeDef timer10;

void timer_init(void)
{
	timer10_init();
}

static void timer10_init(void)
{
	__HAL_RCC_TIM10_CLK_ENABLE();

	timer10.Instance = TIM10;
	timer10.Init.Prescaler = 432 - 1;
	timer10.Init.CounterMode = TIM_COUNTERMODE_UP;
	timer10.Init.Period = 1000 - 1;
	timer10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&timer10) != HAL_OK) {
		//Error_Handler();
	}

	HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	HAL_TIM_Base_Start_IT(&timer10);
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&timer10);

	give_flow_task_semaphore_from_isr();
}
