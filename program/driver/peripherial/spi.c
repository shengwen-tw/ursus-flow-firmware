#include "stm32f7xx_hal.h"

#include "gpio.h"
#include "spi.h"
#include "interrupt.h"

static void spi1_init(void);

SPI_HandleTypeDef spi1;
DMA_HandleTypeDef spi1_rx_dma;

void spi_init(void)
{
	spi1_init();
}

/* SPI1 Clock            = PA5
 * SPI1 MOSI             = PA7
 * SPI1 MISO             = PB4
 * MPU9250 Chip Selector = PC4
 * Clock rate            = 843.75khz
 * Clock polarity (idle) = high
 * Clock phase (sample)  = low
 */
static void spi1_init(void)
{
	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();

	/* init gpio for spi miso/mosi/sck  */
	GPIO_InitTypeDef spi_gpio = {
		.Pin = GPIO_PIN_5 | GPIO_PIN_7,
		.Mode = GPIO_MODE_AF_PP,
		.Pull = GPIO_PULLUP,
		.Speed = GPIO_SPEED_FREQ_VERY_HIGH,
		.Alternate = GPIO_AF5_SPI1
	};
	HAL_GPIO_Init(GPIOA, &spi_gpio);

	spi_gpio.Pin = GPIO_PIN_4;
	HAL_GPIO_Init(GPIOB, &spi_gpio);

	/* init gpio for spi cs */
	GPIO_InitTypeDef mpu9250_cs_gpio = {
		.Pin = GPIO_PIN_4,
		.Mode = GPIO_MODE_OUTPUT_PP,
		.Pull = GPIO_PULLUP,
		.Speed = GPIO_SPEED_HIGH
	};
	HAL_GPIO_Init(GPIOC, &mpu9250_cs_gpio);

	gpio_on(MPU9250_CHIP_SELECTOR);

	spi1.Instance = SPI1;
	spi1.Init.Mode = SPI_MODE_MASTER;
	spi1.Init.Direction = SPI_DIRECTION_2LINES;
	spi1.Init.DataSize = SPI_DATASIZE_8BIT;
	spi1.Init.CLKPolarity = SPI_POLARITY_HIGH; //idle when logic high
	spi1.Init.CLKPhase = SPI_PHASE_2EDGE;      //falling edge
	spi1.Init.NSS = SPI_NSS_SOFT;
	spi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;  //843.75khz
	spi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	spi1.Init.TIMode = SPI_TIMODE_DISABLE;
	spi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spi1.Init.CRCPolynomial = 7;
	spi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	//spi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	HAL_SPI_Init(&spi1);

	spi1_rx_dma.Instance = DMA2_Stream0;
	spi1_rx_dma.Init.Channel = DMA_CHANNEL_3;
	spi1_rx_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
	spi1_rx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
	spi1_rx_dma.Init.MemInc = DMA_MINC_ENABLE;
	spi1_rx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	spi1_rx_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	spi1_rx_dma.Init.Mode = DMA_NORMAL;
	spi1_rx_dma.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	spi1_rx_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	//HAL_DMA_Init(&spi1_rx_dma);

	//__HAL_LINKDMA(&spi1, hdmarx, spi1_rx_dma);

	HAL_NVIC_SetPriority(SPI1_IRQn, SPI1_PRIORITY, 1);
	//HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, SPI1_PRIORITY, 1);
	//HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

void SPI1_IRQHandler(void)
{
	HAL_SPI_IRQHandler(&spi1);
}

void DMA2_Stream0_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&spi1_rx_dma);
}

void spi1_write_byte(uint8_t data)
{
	//TODO:check transmit result!
	HAL_SPI_Transmit(&spi1, &data, 1, UINT32_MAX);
}

uint8_t spi1_read_byte(void)
{
	//TODO:check receive result!
	uint8_t received_data = 0;
	HAL_SPI_Receive(&spi1, &received_data, 1, UINT32_MAX);

	return received_data;
}

void spi1_read(uint8_t *data, int size)
{
	HAL_SPI_Receive_IT(&spi1, data, size);
	while(HAL_SPI_GetState(&spi1) != HAL_SPI_STATE_READY);
}
