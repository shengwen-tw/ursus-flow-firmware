#include <stdint.h>

#include "stm32f7xx.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

#include "gpio.h"
#include "i2c.h"

#include "lidar.h"

QueueHandle_t lidar_queue_handle;

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
	gpio_on(LED_2);

	while(xQueueReceive(lidar_queue_handle, distance, portMAX_DELAY) == pdFALSE);

	/* enable the interrupt and start a new transaction */
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void EXTI3_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);

		//send distance measurement command
		lidar_write_byte(LIDAR_ACQ_COMMAND, 0x04);

		//start receiving lidar distance (interrupt mode, non-blocking code)
		lidar_read_half_word(0x8f, (uint16_t *)lidar_buffer);

		//disable the interrupt until transaction is finished
		HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	}
}

extern I2C_HandleTypeDef i2c2;

void I2C2_EV_IRQHandler(void)
{
	long higher_priority_task_woken = pdFALSE;

	HAL_I2C_EV_IRQHandler(&i2c2);

	if(HAL_I2C_GetState(&i2c2) == HAL_I2C_STATE_READY) {
		gpio_off(LED_2);

		//convert received data from big endian to little endian
		uint16_t lidar_distance = lidar_buffer[0] << 8 | lidar_buffer[1];

		/* put new lidar distance into the queue */
		xQueueSendToBackFromISR(lidar_queue_handle, &lidar_distance,
		                        &higher_priority_task_woken);
	}

	portYIELD_FROM_ISR(higher_priority_task_woken);
}

void lidar_init(void)
{
	lidar_queue_handle = xQueueCreate(16, sizeof(uint16_t));

	lidar_write_byte(LIDAR_ACQ_COMMAND, 0x00); //reset lidar

	/* balance performance mode */
	lidar_write_byte(0x02, 0x80);
	lidar_write_byte(0x04, 0x08);
	lidar_write_byte(0x1c, 0x00);

	exti3_init();
}
