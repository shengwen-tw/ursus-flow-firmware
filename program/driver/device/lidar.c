#include <stdint.h>

#include "i2c.h"

#include "lidar.h"

uint16_t lidar_read_half_word(uint8_t address)
{
        uint16_t buffer = 0;

        i2c2_write(LIDAR_DEV_ADDRESS, &address, 1);
        i2c2_read(LIDAR_DEV_ADDRESS, (uint8_t *)&buffer, 2);

        //convert received data from big endian to little endian
        uint16_t result = buffer >> 8 | buffer << 8;

	

	return result;
}

void lidar_write_byte(uint8_t address, uint8_t data)
{
	i2c2_write(LIDAR_DEV_ADDRESS, &address, 1);
	i2c2_read(LIDAR_DEV_ADDRESS, &data, 1);
}

uint16_t lidar_read_distance(void)
{
	lidar_write_byte(LIDAR_ACQ_COMMAND, 0x00);

	uint16_t distance; //unit: cm
	distance = lidar_read_half_word(0x8f);

	return distance;
}

void lidar_init(void)
{
}
