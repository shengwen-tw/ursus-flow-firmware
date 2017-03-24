#include "stm32f7xx_hal.h"

#include "gpio.h"
#include "spi.h"

static void spi1_init(void);

SPI_HandleTypeDef spi1;

void spi_init(void)
{
	spi1_init();
}

/* SPI1 Clock            = PA5
 * SPI1 MOSI             = PA7
 * SPI1 MISO             = PB4
 * MPU9250 Chip Selector = PC4
 * Clock rate            = 13.5Mhz
 * Clock polarity (idle) = high
 * Clock phase (sample)  = low
 */
static void spi1_init(void)
{
	__HAL_RCC_SPI1_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();

	/* init gpio for spi miso/mosi/sck  */
	GPIO_InitTypeDef spi_gpio = {
		.Pin = GPIO_PIN_5 | GPIO_PIN_7,
		.Mode = GPIO_MODE_AF_PP,
		.Pull = GPIO_NOPULL,
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
	spi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;  //13.5Mhz
	spi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	spi1.Init.TIMode = SPI_TIMODE_DISABLE;
	spi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spi1.Init.CRCPolynomial = 7;
	spi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	//spi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;

	if (HAL_SPI_Init(&spi1) != HAL_OK) {
		//Error_Handler();
	}
}
