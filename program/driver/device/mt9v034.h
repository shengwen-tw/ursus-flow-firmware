#ifndef __MT9V034_H__
#define __MT9V034_H__

#include "stdbool.h"

/* ================ camera setting ================ */
#define IMAGE_BINNING          4
#define FLOW_IMG_WIDTH         79
#define FLOW_IMG_HEIGHT        79
#define CALIB_IMG_WIDTH        188
#define CALIB_IMG_HEIGHT       120

#if (CALIBRATION_ENABLED == 0)
#define IMG_HEIGHT FLOW_IMG_HEIGHT
#define IMG_WIDTH  FLOW_IMG_WIDTH
#else
#define IMG_HEIGHT CALIB_IMG_HEIGHT
#define IMG_WIDTH  CALIB_IMG_WIDTH
#endif

/* =========== mt9v034 i2c device address =========== */
#define MT9V034_DEV_ADDRESS                       0x90

/* ============== mt9v034 register map ============== */
#define MT9V034_CHIP_VERSION                      0x00
#define 	MT9V034_CHIP_ID_REV3              0x1324

#define MT9V032_CHIP_CONTROL                      0x07
#define MT9V034_RESET                             0x0c

#define MT9V034_TEST_PATTERN                      0x7f

#define MT9V034_HIGH_DYNAMIC_ENABLE               0x0f
#define MT9V034_NOISE_CORRECTION_CTRL             0x70
#define MT9V034_AEC_AGC_ENABLE                    0xaf
#define MT9V034_AEC_AGC_DESIRED_BIN               0xa5
#define MT9V034_AEC_LOW_PASS_FILTER               0xa8
#define MT9V034_AGC_AEC_PIXEL_CNT                 0xb0
#define MT9V034_MAX_ANALOG_GAIN                   0xab
#define MT9V034_MIN_COARSE_SHUTTER_WIDTH          0xac
#define MT9V034_MAX_COARSE_SHUTTER_WIDTH          0xad

#define MT9V034_COLUMN_START_A                    0x01
#define		MT9V034_COLUMN_START_MIN          1
#define		MT9V034_COLUMN_START_MAX          752
#define MT9V034_ROW_START_A                       0x02
#define		MT9V034_ROW_START_MIN             4
#define		MT9V034_ROW_START_MAX             482
#define MT9V034_WINDOW_HEIGHT_A                   0x03
#define		MT9V034_WINDOW_HEIGHT_MIN         1
#define		MT9V034_WINDOW_HEIGHT_MAX         480
#define MT9V034_WINDOW_WIDTH_A                    0x04
#define		MT9V034_WINDOW_WIDTH_MIN          1
#define		MT9V034_WINDOW_WIDTH_MAX          752
#define MT9V034_HORIZONTAL_BLANKING_A             0x05
#define		MT9V034_HORIZONTAL_BLANKING_MIN   91
#define		MT9V034_HORIZONTAL_BLANKING_MAX   1023
#define MT9V034_VERTICAL_BLANKING_A               0x06
#define		MT9V034_VERTICAL_BLANKING_MIN     2
#define		MT9V034_VERTICAL_BLANKING_MAX     32288
#define MT9V034_COARSE_SW_1_A                     0x08
#define MT9V034_COARSE_SW_2_A                     0x09
#define MT9V034_COARSE_SW_CTRL_A                  0x0a
#define MT9V034_COARSE_SW_TOTAL_A                 0x0b
#define MT9V034_FINE_SW_1_A                       0xd3
#define MT9V034_FINE_SW_2_A                       0xd4
#define MT9V034_FINE_SW_TOTAL_A                   0xd5
#define MT9V034_READ_MODE_A                       0x0d
#define MT9V034_V1_CTRL_A                         0x31
#define MT9V034_V2_CTRL_A                         0x32
#define MT9V034_V3_CTRL_A                         0x33
#define MT9V034_V4_CTRL_A                         0x34
#define MT9V034_ANALOG_GAIN_CTRL_A                0x35

#define MT9V034_COLUMN_START_B                    0xc9
#define MT9V034_ROW_START_B                       0xca
#define MT9V034_WINDOW_HEIGHT_B                   0xcb
#define MT9V034_WINDOW_WIDTH_B                    0xcc
#define MT9V034_HORIZONTAL_BLANKING_B             0xcd
#define MT9V034_VERTICAL_BLANKING_B               0xce
#define MT9V034_COARSE_SW_1_B                     0xcf
#define MT9V034_COARSE_SW_2_B                     0xd0
#define MT9V034_COARSE_SW_CTRL_B                  0xd1
#define MT9V034_COARSE_SW_TOTAL_B                 0xd2
#define MT9V034_FINE_SW_1_B                       0xd6
#define MT9V034_FINE_SW_2_B                       0xd7
#define MT9V034_FINE_SW_TOTAL_B                   0xd8
#define MT9V034_READ_MODE_B                       0x0e
#define MT9V034_V1_CTRL_B                         0x39
#define MT9V034_V2_CTRL_B                         0x3a
#define MT9V034_V3_CTRL_B                         0x3b
#define MT9V034_V4_CTRL_B                         0x3c
#define MT9V034_ANALOG_GAIN_CTRL_B                0x36

int mt9v034_init(void);

void mt9v034_start_capture_image(uint32_t image_buffer_address);
void mt9v034_wait_finish(void);

bool mt9v034_calibration_is_on(void);

#endif
