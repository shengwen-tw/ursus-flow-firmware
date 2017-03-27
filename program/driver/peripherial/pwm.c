#include "stm32f7xx.h"

static void timer8_pwm_init(void);

TIM_HandleTypeDef pwm_timer8;

void pwm_init(void)
{
	timer8_pwm_init();
}

static void timer8_pwm_init(void)
{
	__HAL_RCC_TIM8_CLK_ENABLE();

	pwm_timer8.Instance = TIM8;
	pwm_timer8.Init.Prescaler = 1080 - 1;
	pwm_timer8.Init.CounterMode = TIM_COUNTERMODE_UP;
	pwm_timer8.Init.Period = 2000 - 1;
	pwm_timer8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
	pwm_timer8.Init.RepetitionCounter = 0;

	if(HAL_TIM_PWM_Init(&pwm_timer8) != HAL_OK) {
		//Error_Handler();
	}

	TIM_MasterConfigTypeDef pwm_master_config = {
		.MasterOutputTrigger = TIM_TRGO_RESET,
		.MasterOutputTrigger2 = TIM_TRGO2_RESET,
		.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE
	};

	if(HAL_TIMEx_MasterConfigSynchronization(&pwm_timer8, &pwm_master_config) != HAL_OK) {
		//Error_Handler();
	}

	TIM_BreakDeadTimeConfigTypeDef pwm_break_dead_config = {
		.OffStateRunMode = TIM_OSSR_DISABLE,
		.OffStateIDLEMode = TIM_OSSI_DISABLE,
		.LockLevel = TIM_LOCKLEVEL_OFF,
		.DeadTime = 0,
		.BreakState = TIM_BREAK_DISABLE,
		.BreakPolarity = TIM_BREAKPOLARITY_HIGH,
		.BreakFilter = 0,
		.Break2State = TIM_BREAK2_DISABLE,
		.Break2Polarity = TIM_BREAK2POLARITY_HIGH,
		.Break2Filter = 0,
		.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE
	};

	if(HAL_TIMEx_ConfigBreakDeadTime(&pwm_timer8, &pwm_break_dead_config) != HAL_OK) {
		//Error_Handler();
	}

	TIM_OC_InitTypeDef pwm_oc_config = {
		.OCMode = TIM_OCMODE_PWM1,
		.Pulse = 2000 - 1,
		.OCPolarity = TIM_OCPOLARITY_HIGH,
		.OCNPolarity = TIM_OCNPOLARITY_HIGH,
		.OCFastMode = TIM_OCFAST_DISABLE,
		.OCIdleState = TIM_OCIDLESTATE_RESET,
		.OCNIdleState = TIM_OCNIDLESTATE_RESET,
	};

	if(HAL_TIM_PWM_ConfigChannel(&pwm_timer8, &pwm_oc_config, TIM_CHANNEL_2) != HAL_OK) {
		//Error_Handler();
	}

	HAL_TIM_PWM_Start(&pwm_timer8, TIM_CHANNEL_2);
}
