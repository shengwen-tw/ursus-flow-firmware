#include <stdint.h>

#include "stm32f7xx.h"

#include "gpio.h"
#include "i2c.h"

#include "lidar.h"

#include "system_time.h"

#define MEDIAN_FILTER_SIZE 20

const uint8_t lidar_dev_address = 0x62 << 1;

uint8_t lidar_buffer[2] = {0};
uint16_t *lidar_distance_ptr;

/* median filter */
uint16_t median_buffer[MEDIAN_FILTER_SIZE];
int median_counter = 0;

#if 0
static uint8_t lidar_read_byte(uint8_t address)
{
	uint8_t result = 0;

	i2c2_read_memory(LIDAR_DEV_ADDRESS, address, &result, 1);

	return result;
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

		//start receiving lidar distance (interrupt mode, non-blocking code)
		lidar_read_half_word(0x8f, (uint16_t *)lidar_buffer);

		//disable the interrupt until transaction is finished
		HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	}
}

__attribute__((section(".itcmtext")))
static uint16_t median_filter(uint16_t *buffer)
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
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *i2c)
{
	if(i2c == &i2c2) {
		//gpio_off(LED_2);

		/* fill the buffer */
		median_buffer[median_counter] = (lidar_buffer[0] << 8 | lidar_buffer[1]);
		median_counter++;

		/* buffer is full, ready to aply the filter */
		if(median_counter == MEDIAN_FILTER_SIZE) {
			/* slide the filter window */
			*lidar_distance_ptr = median_filter(median_buffer);
			median_counter = 0; //reset filter
		}

		/* enable the interrupt and start a new transaction */
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	}
}

void lidar_init(uint16_t *_lidar_distance_ptr)
{
	lidar_distance_ptr = _lidar_distance_ptr;

	/* reset lidar */
	lidar_write_byte(LIDAR_ACQ_COMMAND, 0x00);
	delay_ms(1000);

	/* continuous reading mode */
	lidar_write_byte(0x11, 0xff);
	delay_ms(10);

	/* measurement rate */
	lidar_write_byte(0x45, 0x02);
	delay_ms(10);

	/* fast reading mode */
	lidar_write_byte(0x04, 0x21);
	delay_ms(10);

	/* start distance measurement */
	lidar_write_byte(LIDAR_ACQ_COMMAND, 0x04);
	delay_ms(10);

	exti3_init();
	delay_ms(1000);
}
