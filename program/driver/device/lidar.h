#ifndef __LIDAR_H__
#define __LIDAR_H__

#define LIDAR_DEV_ADDRESS 0xc4

#define LIDAR_ACQ_COMMAND 0x00

uint16_t lidar_read_distance(void);

#endif
