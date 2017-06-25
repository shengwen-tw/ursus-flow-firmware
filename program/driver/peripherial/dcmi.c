#include <stdbool.h>

#include "stm32f7xx_hal.h"

#include "gpio.h"
#include "interrupt.h"

DMA_HandleTypeDef dcmi_dma;
DCMI_HandleTypeDef dcmi;

volatile bool frame_captured = false;

/* DCMI_D4     = PE4
 * DCMI_D6     = PE5
 * DCMI_D7     = PE6
 * DCMI_HSYNC  = PA4
 * DCMI_PIXCLK = PA6
 * DCMI_D0     = PC6
 * DCMI_D2     = PC8
 * DCMI_D1     = PA10
 * DCMI_D8     = PC10
 * DCMI_D9     = PC12
 * DCMI_D5     = PD3
 * DCMI_VSYNC  = PB7
 * DCMI_D3     = PE1
 */
void dcmi_init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_DCMI_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	GPIO_InitTypeDef dcmi_gpio = {
		.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_1,
		.Mode = GPIO_MODE_AF_PP,
		.Pull = GPIO_NOPULL,
		.Speed = GPIO_SPEED_FREQ_LOW,
		.Alternate = GPIO_AF13_DCMI
	};
	HAL_GPIO_Init(GPIOE, &dcmi_gpio);

	dcmi_gpio.Pin = GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_10;
	HAL_GPIO_Init(GPIOA, &dcmi_gpio);

	dcmi_gpio.Pin = GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_12;
	HAL_GPIO_Init(GPIOC, &dcmi_gpio);

	dcmi_gpio.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOD, &dcmi_gpio);

	dcmi_gpio.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOB, &dcmi_gpio);

	dcmi_dma.Instance = DMA2_Stream1;
	dcmi_dma.Init.Channel = DMA_CHANNEL_1;
	dcmi_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
	dcmi_dma.Init.PeriphInc = DMA_PINC_DISABLE;
	dcmi_dma.Init.MemInc = DMA_MINC_ENABLE;
	dcmi_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	dcmi_dma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	dcmi_dma.Init.Mode = DMA_CIRCULAR;
	dcmi_dma.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	dcmi_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

	HAL_DMA_Init(&dcmi_dma);

	dcmi.Instance = DCMI;
	dcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
	dcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
	dcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
	dcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
	dcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
	dcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_10B;
	dcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
	dcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
	dcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
	dcmi.Init.LineSelectMode = DCMI_LSM_ALL;
	dcmi.Init.LineSelectStart = DCMI_OELS_ODD;

	HAL_DCMI_Init(&dcmi);

	__HAL_LINKDMA(&dcmi,DMA_Handle,dcmi_dma);

	HAL_NVIC_SetPriority(DCMI_IRQn, DCMI_PRIORITY, 1);
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, DCMI_PRIORITY, 1);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	HAL_NVIC_EnableIRQ(DCMI_IRQn);
}

__attribute__((section(".itcmtext")))
void dcmi_dma_config(uint32_t buffer_address, uint32_t image_width, uint32_t image_height)
{
	frame_captured = false;
	HAL_DMA_Init(&dcmi_dma);
	HAL_DCMI_Start_DMA(&dcmi, DCMI_MODE_SNAPSHOT, buffer_address, image_width * image_height / 2);
}

__attribute__((section(".itcmtext")))
void dcmi_wait_finish(void)
{
	while(frame_captured == false);
}

__attribute__((section(".itcmtext")))
void DCMI_IRQHandler(void)
{
	HAL_DCMI_IRQHandler(&dcmi);
}

__attribute__((section(".itcmtext")))
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *dcmi)
{
	frame_captured = true;
}

__attribute__((section(".itcmtext")))
void DMA2_Stream1_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&dcmi_dma);
}
