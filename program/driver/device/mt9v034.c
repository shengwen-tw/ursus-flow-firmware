#include <stdint.h>

#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "i2c.h"

#include "mt9v034.h"

#include "delay.h"

static uint16_t mt9v034_read_half_word(uint8_t address)
{
	uint16_t buffer = 0;

	i2c1_write(MT9V034_DEV_ADDRESS, &address, 1);
	i2c1_read(MT9V034_DEV_ADDRESS, (uint8_t *)&buffer, 2);

	//convert received data from big endian to little endian
	uint16_t result = buffer >> 8 | buffer << 8;

	return result;
}

static void mt9v034_write_half_word(uint8_t address, uint16_t data)
{
	uint8_t buffer[3];
	buffer[0] = address;
	/* remeber converting data from little endian to big endian */
	buffer[1] = ((uint8_t *)&data)[1]; //low 8 bit
	buffer[2] = ((uint8_t *)&data)[0]; //high 8 bit

	i2c1_write(MT9V034_DEV_ADDRESS, buffer, 3);
}

static void mt9v034_context_config(void)
{
	uint16_t context = 0x0188; //Just for test!
	mt9v034_write_half_word(MT9V032_CHIP_CONTROL, context);
}

int mt9v034_init(void)
{
	uint16_t chip_version = mt9v034_read_half_word(MT9V034_CHIP_VERSION);

	if(chip_version != MT9V034_CHIP_ID_REV1 &&
	    chip_version != MT9V034_CHIP_ID_REV2 &&
	    chip_version != MT9V034_CHIP_ID_REV3) {
		return 1;
	}

	vTaskDelay(MILLI_SECOND_TICK(1));

	mt9v034_context_config();

	return 0;
}
