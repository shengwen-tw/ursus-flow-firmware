#include <stdint.h>

#include "i2c.h"

#include "mt9v034.h"

uint16_t mt9v034_read_half_word(uint8_t address)
{
	uint16_t buffer = 0;

	i2c1_write(MT9V034_DEV_ADDRESS, &address, 1);
	i2c1_read(MT9V034_DEV_ADDRESS, (uint16_t *)&buffer, 2);

	return buffer;
}

void mt9v034_init(void)
{
	uint16_t chip_version = mt9v034_read_half_word(MT9V034_DEV_ADDRESS);
}
