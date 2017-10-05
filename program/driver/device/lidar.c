#include <stdint.h>
#include <stdbool.h>

#include "stm32f7xx.h"

#include "gpio.h"
#include "i2c.h"

#include "lidar.h"
#include "mpu9250.h"

#include "imu.h"
#include "system_time.h"
#include "low_pass_filter.h"

#define MEDIAN_FILTER_SIZE 20

extern vector3d_f_t gyro_data;
extern vector3d_f_t accel_data;

const uint8_t lidar_dev_address = 0x62 << 1;

int lidar_read_mode = 0;

uint8_t lidar_distance_buffer[2] = {0};
int8_t lidar_velocity_buffer = 0;

float *lidar_distance_ptr;
float *lidar_velocity_ptr;

/* median filter */
float median_buffer[MEDIAN_FILTER_SIZE];
int median_counter = 0;

#if 0
static uint8_t lidar_read_byte(uint8_t address)
{
	uint8_t result = 0;

	i2c2_read_memory(LIDAR_DEV_ADDRESS, address, &result, 1);

	return result;
}
#endif

#if 0
__attribute__((section(".itcmtext")))
static void lidar_read_byte(uint8_t address, uint8_t *data)
{
	i2c2_write(LIDAR_DEV_ADDRESS, &address, 1);
	i2c2_read(LIDAR_DEV_ADDRESS, (uint8_t *)data, 1);
}
#endif

__attribute__((section(".itcmtext")))
static void lidar_read_half_word(uint8_t address, uint16_t *data)
{
	i2c2_write(LIDAR_DEV_ADDRESS, &address, 1);
	i2c2_read(LIDAR_DEV_ADDRESS, (uint8_t *)data, 2);
}

__attribute__((section(".itcmtext")))
void lidar_write_byte(uint8_t address, uint8_t data)
{
	i2c2_write_memory(LIDAR_DEV_ADDRESS, address, &data, 1);
}

__attribute__((section(".itcmtext")))
void EXTI3_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);

		lidar_read_half_word(LIDAR_DISTANCE_REG, (uint16_t *)lidar_distance_buffer);

#if 0
		if(lidar_read_mode == READ_LIDAR_DISTANCE) {
			lidar_read_half_word(LIDAR_DISTANCE_REG, (uint16_t *)lidar_distance_buffer);
		} else if(lidar_read_mode == READ_LIDAR_VELOCITY) {
			lidar_read_byte(LIDAR_VELOCITY_REG, (uint8_t *)&lidar_velocity_buffer);
		}

		lidar_read_mode = (lidar_read_mode + 1) % 2;
#endif

		//disable the interrupt until transaction is finished
		HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	}
}

__attribute__((section(".itcmtext")))
static float median_filter(float *buffer)
{
	/* bubble sort */
	int i, j;
	for(i = 0; i < MEDIAN_FILTER_SIZE; i++) {
		for(j = 0; j < MEDIAN_FILTER_SIZE - 1 - i; j++) {
			if(buffer[j] < buffer[j + 1]) {
				uint16_t tmp;
				tmp = buffer[j];
				buffer[j] = buffer[j + 1];
				buffer[j + 1] = tmp;
			}
		}
	}

	/* pick the median value */
	return (buffer[MEDIAN_FILTER_SIZE / 2] + buffer[MEDIAN_FILTER_SIZE / 2 - 1]) / 2;
}

extern I2C_HandleTypeDef i2c2;

__attribute__((section(".itcmtext")))
void I2C2_EV_IRQHandler(void)
{
	HAL_I2C_EV_IRQHandler(&i2c2);
}

__attribute__((section(".itcmtext")))
static void lidar_receive_distance_handler(I2C_HandleTypeDef *i2c)
{
	float _lidar_distance;

	//low pass filter for lidar
	static float previous_velocity = 0;
	static float last_distance = 0.0f;

	static int velocity_prescaler = 4;

	if(i2c == &i2c2) {
		/* fill the median filter buffer */
		median_buffer[median_counter] =
			(float)(lidar_distance_buffer[0] << 8 | lidar_distance_buffer[1]);
		median_counter++;

		/* buffer is full, aply the median filter */
		if(median_counter == MEDIAN_FILTER_SIZE) {
			/* slide the filter window */
			_lidar_distance = median_filter(median_buffer);

			if(_lidar_distance != 0) {
				*lidar_distance_ptr = _lidar_distance;
			}

			median_counter = 0; //reset filter

#if 1 //calculate velocity
			if(velocity_prescaler == 0) {
				const float delta_t = 0.005f;
				float velocity = (_lidar_distance - last_distance) / delta_t;
				*lidar_velocity_ptr = low_pass_filter(velocity, previous_velocity, 0.01);
				last_distance = _lidar_distance;

				velocity_prescaler = 4;
			}
			velocity_prescaler--;
#endif
		}

		/* enable the interrupt and start a new transaction */
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	}

}

#if 0
__attribute__((section(".itcmtext")))
static void lidar_receive_velocity_handler(I2C_HandleTypeDef *i2c)
{
	static float previous_velocity = 0;

	if(i2c == &i2c2) {
		float new_velocity = (float)lidar_velocity_buffer;
		*lidar_velocity_ptr = low_pass_filter(new_velocity, previous_velocity, 0.1);

		/* enable the interrupt and start a new transaction */
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	}
}
#endif

__attribute__((section(".itcmtext")))
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *i2c)
{
	mpu9250_read(&gyro_data, &accel_data);
	lidar_receive_distance_handler(i2c);

#if 0
	if(lidar_read_mode == READ_LIDAR_DISTANCE) {
		lidar_receive_distance_handler(i2c);
	} else if(lidar_read_mode == READ_LIDAR_VELOCITY) {
		lidar_receive_velocity_handler(i2c);
	}
#endif
}

void lidar_init(float *_lidar_distance_ptr, float *_lidar_velocity_ptr)
{
	lidar_distance_ptr = _lidar_distance_ptr;
	lidar_velocity_ptr = _lidar_velocity_ptr;

	/* reset lidar */
	lidar_write_byte(LIDAR_ACQ_COMMAND_REG, 0x00);
	delay_ms(1000);

	/* continuous reading mode */
	lidar_write_byte(LIDAR_MEASURE_COUNT_REG, 0xff);
	delay_ms(10);

	/* measurement rate */
	lidar_write_byte(LIDAR_MEASURE_DELAY_REG, 0x02);
	delay_ms(10);

	/* fast reading mode */
	lidar_write_byte(LIDAR_ACQ_CONFIG_REG, 0x21);
	delay_ms(10);

	/* start distance measurement */
	lidar_write_byte(LIDAR_ACQ_COMMAND_REG, 0x04);
	delay_ms(10);

	exti3_init();
	delay_ms(1000);
}
