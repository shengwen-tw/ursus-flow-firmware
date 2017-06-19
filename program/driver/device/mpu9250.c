#include <stdint.h>

#include "stm32f7xx_hal.h"

#include "gpio.h"
#include "spi.h"

#include "mpu9250.h"

#include "imu.h"

#include "system_time.h"

__inline__ static void mpu9250_select(void);
__inline__ static void mpu9250_deselect(void);

/* mpu9250 is low active */
static void mpu9250_select(void)
{
	gpio_off(MPU9250_CHIP_SELECTOR);
}

/* mpu9250 is low active */
static void mpu9250_deselect(void)
{
	gpio_on(MPU9250_CHIP_SELECTOR);
}

static uint8_t mpu9250_read_byte(uint8_t address)
{
	mpu9250_select();

	uint8_t buffer = '\0';
	address |= 0x80;

	spi1_write_byte(address);
	spi1_read_byte(&buffer);

	mpu9250_deselect();

	return buffer;
}

static void mpu9250_write_byte(uint8_t address, uint8_t data)
{
	mpu9250_select();

	spi1_write_byte(address);
	spi1_write_byte(data);

	mpu9250_deselect();
}

static uint8_t mpu9250_read_who_am_i(void)
{
	return mpu9250_read_byte(MPU9250_WHO_AM_I);
}

static void mpu9250_read_unscaled_data(vector3d_16_t *unscaled_gyro_data,
				       vector3d_16_t *unscaled_accel_data)
{
	mpu9250_select();

	uint8_t buffer[14] = {0};

	spi1_write_byte(MPU9250_ACCEL_XOUT_H | 0x80);

#if 0
	spi1_read(buffer, 13);
#else
	spi1_read_byte(&buffer[0]);
	spi1_read_byte(&buffer[1]);
	spi1_read_byte(&buffer[2]);
	spi1_read_byte(&buffer[3]);
	spi1_read_byte(&buffer[4]);
	spi1_read_byte(&buffer[5]);
	spi1_read_byte(&buffer[6]);
	spi1_read_byte(&buffer[7]);
	spi1_read_byte(&buffer[8]);
	spi1_read_byte(&buffer[9]);
	spi1_read_byte(&buffer[10]);
	spi1_read_byte(&buffer[11]);
	spi1_read_byte(&buffer[12]);
	spi1_read_byte(&buffer[13]);
#endif
	unscaled_accel_data->x = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
	unscaled_accel_data->y = ((uint16_t)buffer[2] << 8) | (uint16_t)buffer[3];
	unscaled_accel_data->z = ((uint16_t)buffer[4] << 8) | (uint16_t)buffer[5];

	unscaled_gyro_data->x = ((uint16_t)buffer[8] << 8) | (uint16_t)buffer[9];
	unscaled_gyro_data->y = ((uint16_t)buffer[10] << 8) | (uint16_t)buffer[11];
	unscaled_gyro_data->z = ((uint16_t)buffer[12] << 8) | (uint16_t)buffer[13];

	mpu9250_deselect();
}

static void mpu9250_convert_to_scale(vector3d_16_t *unscaled_gyro_data, vector3d_f_t *scaled_gyro_data)
{
	scaled_gyro_data->x = -unscaled_gyro_data->y * MPU9250G_1000dps - MPU9250_OFFSET_X;
	scaled_gyro_data->y = -unscaled_gyro_data->x * MPU9250G_1000dps - MPU9250_OFFSET_Y;
	scaled_gyro_data->z = -unscaled_gyro_data->z * MPU9250G_1000dps - MPU9250_OFFSET_Z;
}

void mpu9250_read(vector3d_f_t *gyro_data)
{
	vector3d_16_t unscaled_gyro_data;
	vector3d_16_t unscaled_accel_data;

	mpu9250_read_unscaled_data(&unscaled_gyro_data, &unscaled_accel_data);
	mpu9250_convert_to_scale(&unscaled_gyro_data, gyro_data);
}

void mpu9250_bias_error_estimate(vector3d_f_t *gyro_bias)
{
	vector3d_f_t gyro_data;

	gyro_bias->x = 0;
	gyro_bias->y = 0;
	gyro_bias->z = 0;

	int n = 10000;
	int count = n;

	float led_blink_time_previous = 0;

	while(count--) {
		float current_time = get_time_sec();

		if((current_time - led_blink_time_previous) > 0.1) {
			gpio_toggle(LED_1);
			gpio_toggle(LED_2);
			gpio_toggle(LED_3);

			led_blink_time_previous = current_time;
		}

		mpu9250_read(&gyro_data);
		gyro_bias->x += gyro_data.x / n;
		gyro_bias->y += gyro_data.y / n;
		gyro_bias->z += gyro_data.z / n;

		delay_ms(1);
	}
}

int mpu9250_init(void)
{
	if(mpu9250_read_who_am_i() != 0x71) {
		delay_ms(50);
		return 1;
	}
	delay_ms(50);

	mpu9250_write_byte(MPU9250_PWR_MGMT_1, 0x80);   //reset command     = 0x80
	delay_ms(50);
	mpu9250_write_byte(MPU9250_GYRO_CONFIG, 0x10);  //full scale 1000Hz = 0x10
	delay_ms(50);
	mpu9250_write_byte(MPU9250_ACCEL_CONFIG, 0x10); //full scale 8g     = 0x10
	delay_ms(50);

	return 0;
}
