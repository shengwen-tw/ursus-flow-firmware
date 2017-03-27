#include "stm32f7xx.h"

static void timer3_pwm_init(void);

TIM_HandleTypeDef pwm_timer3;

void pwm_init(void)
{
	timer3_pwm_init();
}

/*
 * Timer3 channel2 - PWM
 * Generate clock signal for MT9V034
 * Frequency   : 18Mhz
 * High period : 50%
 * Low period  : 50%
 */
static void timer3_pwm_init(void)
{
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef pwm_gpio = {
		.Pin = GPIO_PIN_7,
		.Mode = GPIO_MODE_AF_PP,
		.Pull = GPIO_NOPULL,
		.Speed = GPIO_SPEED_FREQ_VERY_HIGH,
		.Alternate = GPIO_AF2_TIM3
	};
	HAL_GPIO_Init(GPIOC, &pwm_gpio);

	pwm_timer3.Instance = TIM3;
	pwm_timer3.Init.Prescaler = 1 - 1;
	pwm_timer3.Init.CounterMode = TIM_COUNTERMODE_UP;
	pwm_timer3.Init.Period = 6 - 1;
	pwm_timer3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	if(HAL_TIM_PWM_Init(&pwm_timer3) != HAL_OK) {
		//Error_Handler();
	}

	TIM_MasterConfigTypeDef pwm_master_config = {
		.MasterOutputTrigger = TIM_TRGO_RESET,
		.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE
	};

	if(HAL_TIMEx_MasterConfigSynchronization(&pwm_timer3, &pwm_master_config) != HAL_OK) {
		//Error_Handler();
	}

	TIM_OC_InitTypeDef pwm_oc_config = {
		.OCMode = TIM_OCMODE_PWM1,
		.Pulse = 4 - 1,
		.OCPolarity = TIM_OCPOLARITY_HIGH,
		.OCFastMode = TIM_OCFAST_ENABLE
	};

	if(HAL_TIM_PWM_ConfigChannel(&pwm_timer3, &pwm_oc_config, TIM_CHANNEL_2) != HAL_OK) {
		//Error_Handler();
	}

	HAL_TIM_PWM_Start(&pwm_timer3, TIM_CHANNEL_2);
}
