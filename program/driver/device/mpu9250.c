#include <stdint.h>

#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "gpio.h"
#include "spi.h"

#include "mpu9250.h"

#include "delay.h"
#include "imu.h"

/* mpu9250 is low active */
__inline__ void mpu9250_select(void)
{
	gpio_off(MPU9250_CHIP_SELECTOR);
}

/* mpu9250 is low active */
__inline__ void mpu9250_deselect(void)
{
	gpio_on(MPU9250_CHIP_SELECTOR);
}

uint8_t mpu9250_read_byte(uint8_t address)
{
	mpu9250_select();

	uint8_t buffer = '\0';
	address |= 0x80;

	spi1_write_byte(address);
	buffer = spi1_read_byte();

	mpu9250_deselect();

	return buffer;
}

void mpu9250_write_byte(uint8_t address, uint8_t data)
{
	mpu9250_select();

	spi1_write_byte(address);
	spi1_write_byte(data);

	mpu9250_deselect();
}

uint8_t mpu9250_read_who_am_i(void)
{
	return mpu9250_read_byte(MPU9250_WHO_AM_I);
}

void mpu9250_read_unscaled_gyro(vector3d_16_t *unscaled_gyro_data)
{
	mpu9250_select();

	uint8_t buffer[6] = {0};

	spi1_write_byte(MPU9250_GYRO_XOUT_H | 0x80);	
	buffer[0] = spi1_read_byte();
	buffer[1] = spi1_read_byte();
	buffer[2] = spi1_read_byte();
	buffer[3] = spi1_read_byte();
	buffer[4] = spi1_read_byte();
	buffer[5] = spi1_read_byte();

	unscaled_gyro_data->x = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
	unscaled_gyro_data->y = ((uint16_t)buffer[2] << 8) | (uint16_t)buffer[3];
	unscaled_gyro_data->z = ((uint16_t)buffer[4] << 8) | (uint16_t)buffer[5];

	mpu9250_deselect();
}

int mpu9250_init(void)
{
	if(mpu9250_read_who_am_i() != 0x71) {
		vTaskDelay(MILLI_SECOND_TICK(50));
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
