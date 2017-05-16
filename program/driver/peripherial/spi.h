#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f7xx_hal.h"

#define MPU9250_CHIP_SELECTOR GPIOC, GPIO_PIN_4

void spi_init(void);

void spi1_write_byte(uint8_t data);
void spi1_read_byte(uint8_t *received_data);

void spi1_read(uint8_t *data, int size);

#endif
