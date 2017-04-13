#include "stm32f7xx_hal.h"

#include "gpio.h"
#include "interrupt.h"

static void i2c1_init(void);
static void i2c2_init(void);

I2C_HandleTypeDef i2c1;
I2C_HandleTypeDef i2c2;

void i2c_init(void)
{
	i2c1_init(); //mt9v034
	i2c2_init(); //lidar
}

/*
 * MT9V034 control = I2C1
 * Clock frequency = 40khz
 * Mode            = 7-bit master
 */
static void i2c1_init(void)
{
	__HAL_RCC_I2C1_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef i2c_gpio = {
		.Pin = GPIO_PIN_6 | GPIO_PIN_9,
		.Mode = GPIO_MODE_AF_OD,
		.Pull = GPIO_PULLUP,
		.Speed = GPIO_SPEED_FREQ_VERY_HIGH,
		.Alternate = GPIO_AF4_I2C1
	};
	HAL_GPIO_Init(GPIOB, &i2c_gpio);

	i2c1.Instance = I2C1;
	i2c1.Init.Timing = 0x2040beff;
	i2c1.Init.OwnAddress1 = 0;
	i2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	i2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2c1.Init.OwnAddress2 = 0;
	i2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	i2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	HAL_I2C_Init(&i2c1); 
	HAL_I2CEx_ConfigAnalogFilter(&i2c1, I2C_ANALOGFILTER_ENABLE);
}

void i2c1_write(uint16_t device_address, uint8_t *data, uint16_t size)
{
	HAL_I2C_Master_Transmit(&i2c1, device_address, data, size, UINT32_MAX);
}

void i2c1_read(uint16_t device_address, uint8_t *data, uint16_t size)
{
	HAL_I2C_Master_Receive(&i2c1, device_address, data, size, UINT32_MAX);
}

/*
 * MT9V034 control = I2C2
 * Clock frequency = 70khz
 * Mode            = 7-bit master
 */
static void i2c2_init(void)
{
	__HAL_RCC_I2C2_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef i2c_gpio = {
		.Pin = GPIO_PIN_10 | GPIO_PIN_11,
		.Mode = GPIO_MODE_AF_OD,
		.Pull = GPIO_PULLUP,
		.Speed = GPIO_SPEED_FREQ_VERY_HIGH,
		.Alternate = GPIO_AF4_I2C2
	};

	HAL_GPIO_Init(GPIOB, &i2c_gpio);

	i2c2.Instance = I2C2;
	i2c2.Init.Timing = 0x10207cff;
	i2c2.Init.OwnAddress1 = 0;
	i2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	i2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2c2.Init.OwnAddress2 = 0;
	i2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	i2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	HAL_I2C_Init(&i2c2);
	HAL_I2CEx_ConfigAnalogFilter(&i2c2, I2C_ANALOGFILTER_ENABLE);

	HAL_NVIC_SetPriority(I2C2_EV_IRQn, I2C2_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
}

void I2C2_EV_IRQHandler(void)
{
	HAL_I2C_EV_IRQHandler(&i2c2);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *i2c2)
{
}

void i2c2_write(uint16_t device_address, uint8_t *data, uint16_t size)
{
	HAL_I2C_Master_Transmit(&i2c2, device_address, data, size, UINT32_MAX);
}

void i2c2_read(uint16_t device_address, uint8_t *data, uint16_t size)
{
	HAL_I2C_Master_Receive_IT(&i2c2, device_address, data, size);

	while(HAL_I2C_GetState(&i2c2) != HAL_I2C_STATE_READY);
}

void i2c2_write_memory(uint16_t device_address, uint8_t memory_address, uint8_t *data, uint16_t size)
{
	HAL_I2C_Mem_Write(&i2c2, device_address, memory_address, 1, data, size, UINT32_MAX);
}

void i2c2_read_memory(uint16_t device_address, uint8_t memory_address, uint8_t *data, uint16_t size)
{
	HAL_I2C_Mem_Read(&i2c2, device_address, memory_address, 1, data, size, UINT32_MAX);
}
