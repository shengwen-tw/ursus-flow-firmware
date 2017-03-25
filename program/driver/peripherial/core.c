#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "delay.h"

/* System Clock source            = PLL (HSE)
 * SYSCLK(Hz)                     = 216000000
 * HCLK(Hz)                       = 216000000
 * AHB Prescaler                  = 1
 * APB1 Prescaler                 = 4
 * APB2 Prescaler                 = 2
 * HSE Frequency(Hz)              = 8000000
 * PLL_M                          = 4
 * PLL_N                          = 216
 * PLL_P                          = 6
 * PLL_Q                          = 9
 * PLL_R                          = X
 * VDD(V)                         = 3.3
 * Main regulator output voltage  = Scale1 mode
 * Flash Latency(WS)              = 7
 */
void system_clock_init(void)
{
	/* Enable the power control clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/* The voltage scaling indicate the power consumption */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable hse oscillator and activate pll with hse as source */
	RCC_OscInitTypeDef RCC_OscInitStruct = {
		.OscillatorType = RCC_OSCILLATORTYPE_HSE,
		.HSEState = RCC_HSE_ON,
		.PLL.PLLState = RCC_PLL_ON,
		.PLL.PLLSource = RCC_PLLSOURCE_HSE,
		.PLL.PLLM = 4,
		.PLL.PLLN = 216,
		.PLL.PLLP = RCC_PLLP_DIV2,
		.PLL.PLLQ = 9,
	};

	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		//Error_Handler();
	}

	/* Activate the over drive to reach the 216MHz frequency */
	if(HAL_PWREx_EnableOverDrive() != HAL_OK) {
		//Error_Handler();
	}

	/* Select pll as system clock source and configure the hclk, pclk1 and pclk2 clocks dividers */
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {
		.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
		RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2,
		.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK,
		.AHBCLKDivider = RCC_SYSCLK_DIV1,
		.APB1CLKDivider = RCC_HCLK_DIV4,
		.APB2CLKDivider = RCC_HCLK_DIV2
	};

	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
		//Error_Handler();
	}
}

void mpu_init(void)
{
	HAL_MPU_Disable();

	/* Configure the MPU attributes as WT for SRAM */
	MPU_Region_InitTypeDef MPU_InitStruct = {
		.Enable = MPU_REGION_ENABLE,
		.BaseAddress = 0x20010000,
		.Size = MPU_REGION_SIZE_256KB,
		.AccessPermission = MPU_REGION_FULL_ACCESS,
		.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE,
		.IsCacheable = MPU_ACCESS_CACHEABLE,
		.IsShareable = MPU_ACCESS_SHAREABLE,
		.Number = MPU_REGION_NUMBER0,
		.TypeExtField = MPU_TEX_LEVEL0,
		.SubRegionDisable = 0x00,
		.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE
	};

	HAL_MPU_ConfigRegion(&MPU_InitStruct);

	/* Enable the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void cpu_cache_enable(void)
{
	/* Enable i-cache and d-cache */
	SCB_EnableICache();
	SCB_EnableDCache();
}

/* Override HAL delay function */
void HAL_Delay(volatile uint32_t millis)
{
	vTaskDelay(MILLI_SECOND_TICK(millis));
}

/* The HAL_InitTick() should be override to prevent it initialize
 * the systick  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
}
