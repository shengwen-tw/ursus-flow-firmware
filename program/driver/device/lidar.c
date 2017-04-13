#include <stdint.h>

#include "stm32f7xx.h"

#include "gpio.h"
#include "i2c.h"

#include "lidar.h"

const uint8_t lidar_dev_address = 0x62 << 1;

uint8_t lidar_buffer[2] = {0};

static uint8_t lidar_read_byte(uint8_t address)
{
	uint8_t result = 0;

	i2c2_read_memory(LIDAR_DEV_ADDRESS, address, &result, 1);

	return result;
}

static void lidar_read_half_word(uint8_t address, uint16_t *data)
{
	i2c2_write(LIDAR_DEV_ADDRESS, &address, 1);
	i2c2_read(LIDAR_DEV_ADDRESS, (uint8_t *)data, 2);
}

void lidar_write_byte(uint8_t address, uint8_t data)
{
	i2c2_write_memory(LIDAR_DEV_ADDRESS, address, &data, 1);
}

void lidar_read_distance(uint16_t *distance)
{
	//convert received data from big endian to little endian
	uint16_t result = lidar_buffer[0] << 8 | lidar_buffer[1];

	*distance = result;
}

void EXTI3_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);

		gpio_on(LED_2);

		//send distance measurement command
		lidar_write_byte(LIDAR_ACQ_COMMAND, 0x04);

		lidar_read_half_word(0x8f, (uint16_t *)lidar_buffer);
	}
}

void lidar_init(void)
{
}
