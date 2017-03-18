#include <string.h>
#include "stm32f7xx_hal.h"

void uart2_init(int baudrate);

UART_HandleTypeDef uart2;

void uart_init(void)
{
	uart2_init(57600);
}

void uart2_init(int baudrate)
{
	__HAL_RCC_USART2_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();

	uart2.Instance = USART2;
	uart2.Init.BaudRate = baudrate;
	uart2.Init.WordLength = UART_WORDLENGTH_8B;
	uart2.Init.StopBits = UART_STOPBITS_1;
	uart2.Init.Parity = UART_PARITY_NONE;
	uart2.Init.Mode = UART_MODE_TX_RX;
	uart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart2.Init.OverSampling = UART_OVERSAMPLING_16;
	uart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	uart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	HAL_UART_Init(&uart2);

	GPIO_InitTypeDef gpio = {
		.Pin = GPIO_PIN_2 | GPIO_PIN_3,
		.Mode = GPIO_MODE_AF_PP,
		.Pull = GPIO_PULLUP,
		.Speed = GPIO_SPEED_FREQ_VERY_HIGH,
		.Alternate = GPIO_AF7_USART2
	};

	HAL_GPIO_Init(GPIOA, &gpio);

	/* DMA1 channel4 stream6 for UART2 tx */
	DMA_HandleTypeDef uart2_tx_dma = {
		.Instance = DMA1_Stream6, 
		.Init.Channel = DMA_CHANNEL_4,
		.Init.Direction = DMA_MEMORY_TO_PERIPH,
		.Init.PeriphInc = DMA_PINC_DISABLE,
		.Init.MemInc = DMA_MINC_ENABLE,
		.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
		.Init.MemDataAlignment = DMA_PDATAALIGN_BYTE,
		.Init.Mode = DMA_NORMAL,
		.Init.Priority = DMA_PRIORITY_LOW,
	};

	HAL_DMA_Init(&uart2_tx_dma);
	__HAL_LINKDMA(&uart2, hdmatx, uart2_tx_dma);
}

void uart2_puts(char *str)
{
	//HAL_UART_Transmit_DMA(&uart2, (uint8_t*)str, strlen(str));
	HAL_UART_Transmit(&uart2, (uint8_t*)str, strlen(str), 100);
}
