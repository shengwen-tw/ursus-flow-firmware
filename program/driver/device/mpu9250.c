#include <stdint.h>

#include "stm32f7xx_hal.h"

#include "gpio.h"
#include "spi.h"

#include "mpu9250.h"

/* mpu9250 is low active */
inline void mpu9250_select(void)
{
	gpio_off(MPU9250_CHIP_SELECTOR);
}

/* mpu9250 is low active */
inline void mpu9250_deselect(void)
{
	gpio_on(MPU9250_CHIP_SELECTOR);
}

uint8_t mpu9250_read_byte(uint8_t address)
{
	uint8_t buffer = '\0';
	address |= 0x80;

	HAL_SPI_Transmit(MPU9250_SPI, &address, 1, 1000);
	HAL_SPI_Receive(MPU9250_SPI, &buffer, 1, 1000);

	return buffer;
}

uint8_t mpu9250_read_who_am_i(void)
{
	return mpu9250_read_byte(MPU9250_WHO_AM_I);
}

int mpu9250_init(void)
{
	mpu9250_select();

	if(mpu9250_read_who_am_i() != 0x71) {return 1;}

	mpu9250_deselect();

	return 0;
}
