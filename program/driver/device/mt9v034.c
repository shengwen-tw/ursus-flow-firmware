#include <stdint.h>
#include <stdbool.h>

#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "i2c.h"

#include "mt9v034.h"

#include "delay.h"

bool calibration_enabled = true;

static uint16_t mt9v034_read_half_word(uint8_t address)
{
	uint16_t buffer = 0;

	i2c1_write(MT9V034_DEV_ADDRESS, &address, 1);
	i2c1_read(MT9V034_DEV_ADDRESS, (uint8_t *)&buffer, 2);

	//convert received data from big endian to little endian
	uint16_t result = buffer >> 8 | buffer << 8;

	vTaskDelay(MILLI_SECOND_TICK(1));

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

	vTaskDelay(MILLI_SECOND_TICK(1));
}

static void mt9v034_context_config(void)
{
	/* context a : optical flow mode (4x image binning) */

	/* context b : calibration mode (full size image and no binning) */
	mt9v034_write_half_word(MT9V034_COLUMN_START_B, 1);
	mt9v034_write_half_word(MT9V034_ROW_START_B, 4);
	mt9v034_write_half_word(MT9V034_WINDOW_HEIGHT_B, 480);
	mt9v034_write_half_word(MT9V034_WINDOW_WIDTH_B, 752);
	//mt9v034_write_half_word(MT9V034_HORIZONTAL_BLANKING_B, );
	//mt9v034_write_half_word(MT9V034_VERTICAL_BLANKING_B, );
	//mt9v034_write_half_word(MT9V034_COARSE_SW_1_B, );
	//mt9v034_write_half_word(MT9V034_COARSE_SW_2_B, );
	//mt9v034_write_half_word(MT9V034_COARSE_SW_CTRL_B, );
	//mt9v034_write_half_word(MT9V034_COARSE_SW_TOTAL_B, );
	mt9v034_write_half_word(MT9V034_READ_MODE_B, 0x300); //[9:8] reserved

	/* chip control register [16-bits]:
	 * [2:0] : scan mode = progressive scan (0) 
	 * [4:3] : sensor operating mode = master mode (1)
	 * [5]   : stereoscopy mode = disabled (0)
	 * [6]   : stereoscopic master/slave mode = not used (0)
	 * [7]   : parallel output enabled = enabled (1)
	 * [8]   : simultaneous/sequential mode = simultaneous mode (1)
	 * [9]   : reserved = (0) according to datasheet
	 * [15]  : context a/b select = a (0) / b (1)
	 */
	if(calibration_enabled == true) {
		mt9v034_write_half_word(MT9V032_CHIP_CONTROL, 0x8188);
	} else {
		mt9v034_write_half_word(MT9V032_CHIP_CONTROL, 0x0188);
	}
}

int mt9v034_init(void)
{
	uint16_t chip_version = mt9v034_read_half_word(MT9V034_CHIP_VERSION);

	if(chip_version != MT9V034_CHIP_ID_REV3) {
		return 1;
	}

	mt9v034_context_config();

	mt9v034_write_half_word(MT9V034_RESET, 0x01); //reset mt9v034

	return 0;
}

bool mt9v034_calibration_is_on(void)
{
	return calibration_enabled;
}
