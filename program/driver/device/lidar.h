#ifndef __LIDAR_H__
#define __LIDAR_H__

#define LIDAR_DEV_ADDRESS (0x62 << 1)

#define LIDAR_ACQ_COMMAND 0x00
#define LIDAR_STATUS      0x01

enum {
	READ_LIDAR_DISTANCE,
	READ_LIDAR_VELOCITY		
} LIDAR_MODE;

void lidar_init(uint16_t *_lidar_distance_ptr, int8_t *_lidar_velocity_ptr);

#endif
