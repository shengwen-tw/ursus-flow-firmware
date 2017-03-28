#ifndef __MT9V034_H__
#define __MT9V034_H__

#define MT9V034_DEV_ADDRESS  0x90

#define MT9V034_CHIP_VERSION          0x00
#define 	MT9V034_CHIP_ID_REV1  0x1311
#define 	MT9V034_CHIP_ID_REV2  0x1313
#define 	MT9V034_CHIP_ID_REV3  0x1324
#define MT9V032_CHIP_CONTROL          0x07

int mt9v034_init(void);

#endif
