#include <stdint.h>

#include "stm32f7xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "gpio.h"
#include "i2c.h"

#include "lidar.h"

#include "delay.h"

#include "system_time.h"

const uint8_t lidar_dev_address = 0x62 << 1;

uint8_t lidar_buffer[2] = {0};
uint16_t *lidar_distance_ptr;

#if 0
static uint8_t lidar_read_byte(uint8_t address)
{
	uint8_t result = 0;

	i2c2_read_memory(LIDAR_DEV_ADDRESS, address, &result, 1);

	return result;
}
#endif

static void lidar_read_half_word(uint8_t address, uint16_t *data)
{
	i2c2_write(LIDAR_DEV_ADDRESS, &address, 1);
	i2c2_read(LIDAR_DEV_ADDRESS, (uint8_t *)data, 2);
}

void lidar_write_byte(uint8_t address, uint8_t data)
{
	i2c2_write_memory(LIDAR_DEV_ADDRESS, address, &data, 1);
}

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

extern I2C_HandleTypeDef i2c2;

void I2C2_EV_IRQHandler(void)
{
	HAL_I2C_EV_IRQHandler(&i2c2);
	lidar_write_byte(0x11, 0xff);
	if(HAL_I2C_GetState(&i2c2) == HAL_I2C_STATE_READY) {
		//gpio_off(LED_2);

		*lidar_distance_ptr = lidar_buffer[0] << 8 | lidar_buffer[1];

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

	lidar_write_byte(0x11, 0xff);
	delay_ms(10);

	/* measurement rate */
	lidar_write_byte(0x45, 0x02);
	delay_ms(10);

	/* continuous reading mode */
	lidar_write_byte(0x04, 0x21);
	delay_ms(10);

	/* start distance measurement */
	lidar_write_byte(LIDAR_ACQ_COMMAND, 0x04);
	delay_ms(10);

	exti3_init();
}
