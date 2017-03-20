#include <stdint.h>

#include "stm32f7xx_hal.h"

#include "gpio.h"
#include "spi.h"

#include "mpu9250.h"

inline void mpu9250_select(void)
{
	gpio_on(MPU9250_CHIP_SELECTOR);
}

inline void mpu9250_deselect(void)
{
	gpio_off(MPU9250_CHIP_SELECTOR);
}

uint8_t mpu9250_read_byte(uint8_t address)
{
	uint8_t buffer;
	address |= 0x80;

	HAL_SPI_Transmit(MPU9250_SPI, &address, 1, 100);
	HAL_SPI_Receive(MPU9250_SPI, &buffer, 1, 100);

	return buffer;
}

uint8_t mpu9250_read_who_am_i(void)
{
	return mpu9250_read_byte(MPU9250_WHO_AM_I);
}

void mpu9250_init(void)
{
	mpu9250_select();
	uint8_t id = mpu9250_read_who_am_i();
	mpu9250_deselect();
}
