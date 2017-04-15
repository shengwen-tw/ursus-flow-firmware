#include <stdint.h>
#include <stdbool.h>

#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "i2c.h"
#include "dcmi.h"

#include "mt9v034.h"

#include "delay.h"

#define CALIBRATION_ENABLED 1

__inline__ void mt9v034_wait_finish(void);

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
	/* general setting */
	mt9v034_write_half_word(MT9V034_HIGH_DYNAMIC_ENABLE, 0x0000);   //disable HDR
	mt9v034_write_half_word(MT9V034_NOISE_CORRECTION_CTRL, 0x0303);
	mt9v034_write_half_word(MT9V034_AEC_AGC_ENABLE, 0x0101);  //enale AEC, diable AGC
	mt9v034_write_half_word(MT9V034_AEC_AGC_DESIRED_BIN, 50); //light regulation
	mt9v034_write_half_word(MT9V034_AEC_LOW_PASS_FILTER, 0x0002);

	/* reserved register (read mt9v034 rev 7.1 datasheet table 8) */
	mt9v034_write_half_word(0x13, 0x2d2e);
	mt9v034_write_half_word(0x20, 0x03c7);
	mt9v034_write_half_word(0x24, 0x001b);
	mt9v034_write_half_word(0x2b, 0x0003);
	mt9v034_write_half_word(0x2f, 0x0003);

	/* context a : optical flow mode (4x image binning) */

	/* context b : calibration mode (full size image and 4x binning) */
	mt9v034_write_half_word(
	        MT9V034_COLUMN_START_B,
	        (MT9V034_WINDOW_WIDTH_MAX - CALIB_IMG_WIDTH * IMAGE_BINNING) / 2 + MT9V034_COLUMN_START_MIN
	);
	mt9v034_write_half_word(
	        MT9V034_ROW_START_B,
	        (MT9V034_WINDOW_HEIGHT_MAX - CALIB_IMG_HEIGHT * IMAGE_BINNING) / 2 + MT9V034_ROW_START_MIN
	);
	mt9v034_write_half_word(MT9V034_WINDOW_HEIGHT_B, CALIB_IMG_HEIGHT * IMAGE_BINNING); //120
	mt9v034_write_half_word(MT9V034_WINDOW_WIDTH_B, CALIB_IMG_WIDTH * IMAGE_BINNING);   //188
	mt9v034_write_half_word(MT9V034_HORIZONTAL_BLANKING_B, 709 + MT9V034_HORIZONTAL_BLANKING_MIN);
	mt9v034_write_half_word(MT9V034_VERTICAL_BLANKING_B, 10);
	mt9v034_write_half_word(MT9V034_COARSE_SW_1_B, 443);       //default value
	mt9v034_write_half_word(MT9V034_COARSE_SW_2_B, 473);       //default value
	mt9v034_write_half_word(MT9V034_COARSE_SW_CTRL_B, 0x0164); //default value
	mt9v034_write_half_word(MT9V034_COARSE_SW_TOTAL_B, 480);   //default value
	mt9v034_write_half_word(MT9V034_READ_MODE_B, 0x030a);      //enable 4x pixel binning

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
#if (CALIBRATION_ENABLED == 1)
	mt9v034_write_half_word(MT9V032_CHIP_CONTROL, 0x8188);
#else
	mt9v034_write_half_word(MT9V032_CHIP_CONTROL, 0x0188);
#endif
}

void mt9v034_enable_test_pattern(void)
{
	//mt9v034_write_half_word(MT9V034_TEST_PATTERN, 0x2800); //shade in horizontal direction
	//mt9v034_write_half_word(MT9V034_TEST_PATTERN, 0x3000); //shade in vertical direction
	mt9v034_write_half_word(MT9V034_TEST_PATTERN, 0x3800); //shade in diagonal direction
}

int mt9v034_init(void)
{
	uint16_t chip_version = mt9v034_read_half_word(MT9V034_CHIP_VERSION);

	if(chip_version != MT9V034_CHIP_ID_REV3) {
		return 1;
	}

	mt9v034_context_config();

	//mt9v034_enable_test_pattern();

	mt9v034_write_half_word(MT9V034_RESET, 0x01); //reset mt9v034

	return 0;
}

void mt9v034_start_capture_image(uint32_t image_buffer_address)
{
	dcmi_dma_config(image_buffer_address, IMG_WIDTH, IMG_HEIGHT);
}

void mt9v034_wait_finish(void)
{
	dcmi_wait_finish();
}

bool mt9v034_calibration_is_on(void)
{
	return CALIBRATION_ENABLED;
}
