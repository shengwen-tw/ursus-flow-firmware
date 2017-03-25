#include <stdint.h>

#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "gpio.h"
#include "spi.h"

#include "mpu9250.h"

#include "delay.h"

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
	mpu9250_select();

	uint8_t buffer = '\0';
	address |= 0x80;

	HAL_SPI_Transmit(MPU9250_SPI, &address, 1, 1000);
	HAL_SPI_Receive(MPU9250_SPI, &buffer, 1, 1000);

	mpu9250_deselect();

	return buffer;
}

void mpu9250_write_byte(uint8_t address, uint8_t data)
{
	mpu9250_select();

	HAL_SPI_Transmit(MPU9250_SPI, &address, 1, UINT32_MAX);
	HAL_SPI_Transmit(MPU9250_SPI, &data, 1, UINT32_MAX);

	mpu9250_deselect();
}

uint8_t mpu9250_read_who_am_i(void)
{
	return mpu9250_read_byte(MPU9250_WHO_AM_I);
}

int mpu9250_init(void)
{
	if(mpu9250_read_who_am_i() != 0x71) {
		return 1;
	}
	vTaskDelay(MILLI_SECOND_TICK(50));	

	mpu9250_write_byte(MPU9250_PWR_MGMT_1, 0x80);   //reset command     = 0x80
	vTaskDelay(MILLI_SECOND_TICK(50));
	mpu9250_write_byte(MPU9250_GYRO_CONFIG, 0x10);  //full scale 1000Hz = 0x10
	vTaskDelay(MILLI_SECOND_TICK(50));
	mpu9250_write_byte(MPU9250_ACCEL_CONFIG, 0x10); //full scale 8g     = 0x10
	vTaskDelay(MILLI_SECOND_TICK(50));

	return 0;
}
