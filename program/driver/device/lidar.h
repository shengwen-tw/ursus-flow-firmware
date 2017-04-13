#ifndef __LIDAR_H__
#define __LIDAR_H__

#define LIDAR_DEV_ADDRESS (0x62 << 1)

#define LIDAR_ACQ_COMMAND 0x00
#define LIDAR_STATUS      0x01

void lidar_read_distance(uint16_t *distance);

#endif
