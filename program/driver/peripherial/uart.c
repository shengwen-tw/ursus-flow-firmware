#include <string.h>

#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "interrupt.h"

static void uart2_init(int baudrate);

UART_HandleTypeDef uart2;
DMA_HandleTypeDef uart2_tx_dma;

SemaphoreHandle_t uart2_tx_semaphore;

void uart_init(void)
{
	uart2_init(57600);
}

static void uart2_init(int baudrate)
{
	/* Create semaphore for serial resource */
	uart2_tx_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(uart2_tx_semaphore);

	/* Hardware initialization */
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();

	uart2.Instance = USART2;
	uart2.Init.BaudRate = baudrate;
	uart2.Init.WordLength = UART_WORDLENGTH_8B;
	uart2.Init.StopBits = UART_STOPBITS_1;
	uart2.Init.Parity = UART_PARITY_NONE;
	uart2.Init.Mode = UART_MODE_TX;
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
	uart2_tx_dma.Instance = DMA1_Stream6;
	uart2_tx_dma.Init.Channel = DMA_CHANNEL_4;
	uart2_tx_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
	uart2_tx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
	uart2_tx_dma.Init.MemInc = DMA_MINC_ENABLE;
	uart2_tx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	uart2_tx_dma.Init.MemDataAlignment = DMA_PDATAALIGN_BYTE;
	uart2_tx_dma.Init.Mode = DMA_NORMAL;
	uart2_tx_dma.Init.Priority = DMA_PRIORITY_LOW;

	HAL_DMA_Init(&uart2_tx_dma);
	__HAL_LINKDMA(&uart2, hdmatx, uart2_tx_dma);

	HAL_NVIC_SetPriority(USART2_IRQn, UART2_PRIORITY, 1);
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, UART2_PRIORITY, 1);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQHandler(void)
{
	long higher_priority_task_woken = pdFALSE;

	HAL_UART_IRQHandler(&uart2);

	if(HAL_UART_GetState(&uart2) == HAL_UART_STATE_READY) {
		xSemaphoreGiveFromISR(uart2_tx_semaphore, &higher_priority_task_woken);
		portYIELD_FROM_ISR(higher_priority_task_woken);
	}
}

void DMA1_Stream6_IRQHandler(void)
{
	HAL_DMA_IRQHandler(uart2.hdmatx);
}

void uart2_puts(char *str, int size)
{
	xSemaphoreTake(uart2_tx_semaphore, portMAX_DELAY);

	HAL_UART_Transmit_DMA(&uart2, (uint8_t*)str, size);
}
