#include <stdint.h>

#include "i2c.h"

#include "lidar.h"

const uint8_t lidar_dev_address = 0x62 << 1;

static uint8_t lidar_read_byte(uint8_t address)
{
        uint8_t result = 0;

        i2c2_write(LIDAR_DEV_ADDRESS, &address, 1);
        i2c2_read(LIDAR_DEV_ADDRESS, &result, 1);

	return result;
}

static uint16_t lidar_read_half_word(uint8_t address)
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
	uint8_t lidar_busy_flag = 1; //device busy
	uint16_t trial = 65535;

	/* wait until lidar is not busy */
	while(trial-- || lidar_busy_flag) {
		//read lidar status in byte and apply bit mask to get bit 0 only
		lidar_busy_flag = lidar_read_byte(LIDAR_STATUS) & 0x01;
	}

	if(trial == 0) {
		//while(1); //XXX:test only
	}

	//send distance measurement command
	lidar_write_byte(LIDAR_ACQ_COMMAND, 0x03);

	uint16_t distance; //unit: cm
	distance = lidar_read_half_word(0x8f);

	return distance;
}

void lidar_init(void)
{
}
