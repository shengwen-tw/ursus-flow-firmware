#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f7xx_hal.h"

#define MPU9250_CHIP_SELECTOR GPIOC, GPIO_PIN_4

#define MPU9250_SPI &spi1

void spi_init(void);

#endif
