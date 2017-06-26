#ifndef __MPU9250_H__
#define __MPU9250_H__

#include "imu.h"

#define DO_IMU_CALIBRATION 0

#define ACCEL_1G 4096.0f
#define ACCEL_X_SCALE (ACCEL_1G / 4105.0f) //calibrate this
#define ACCEL_Y_SCALE (ACCEL_1G / 4104.0f)
#define ACCEL_Z_SCALE (ACCEL_1G / 4133.0f)

#define ACCEL_X_OFFSET +82.0f //calibrate this
#define ACCEL_Y_OFFSET +27.0f
#define ACCEL_Z_OFFSET -60.0f

/* calibration */
#define MPU9250_OFFSET_X +0.137f //calibrate this
#define MPU9250_OFFSET_Y -2.338f
#define MPU9250_OFFSET_Z +0.254f

/* mpu9250 register map */
#define MPU9250_SMPLRT_DIV     ((uint8_t)0x19)
#define MPU9250_CONFIG         ((uint8_t)0x1A)
#define MPU9250_GYRO_CONFIG    ((uint8_t)0x1B)
#define MPU9250_ACCEL_CONFIG   ((uint8_t)0x1C)
#define MPU9250_MOT_THR        ((uint8_t)0x1F)
#define MPU9250_FIFO_EN        ((uint8_t)0x23)

#define MPU9250_INT_ENABLE     ((uint8_t)0x38)
#define MPU9250_INT_STATUS     ((uint8_t)0x3A)
#define MPU9250_ACCEL_XOUT_H   ((uint8_t)0x3B)
#define MPU9250_ACCEL_XOUT_L   ((uint8_t)0x3C)
#define MPU9250_ACCEL_YOUT_H   ((uint8_t)0x3D)
#define MPU9250_ACCEL_YOUT_L   ((uint8_t)0x3E)
#define MPU9250_ACCEL_ZOUT_H   ((uint8_t)0x3F)
#define MPU9250_ACCEL_ZOUT_L   ((uint8_t)0x40)
#define MPU9250_TEMP_OUT_H     ((uint8_t)0x41)
#define MPU9250_TEMP_OUT_L     ((uint8_t)0x42)
#define MPU9250_GYRO_XOUT_H    ((uint8_t)0x43)
#define MPU9250_GYRO_XOUT_L    ((uint8_t)0x44)
#define MPU9250_GYRO_YOUT_H    ((uint8_t)0x45)
#define MPU9250_GYRO_YOUT_L    ((uint8_t)0x46)
#define MPU9250_GYRO_ZOUT_H    ((uint8_t)0x47)
#define MPU9250_GYRO_ZOUT_L    ((uint8_t)0x48)

#define MPU9250_USER_CTRL      ((uint8_t)0x6A)
#define MPU9250_PWR_MGMT_1     ((uint8_t)0x6B)
#define MPU9250_PWR_MGMT_2     ((uint8_t)0x6C)
#define MPU9250_FIFO_COUNTH    ((uint8_t)0x72)
#define MPU9250_FIFO_COUNTL    ((uint8_t)0x73)
#define MPU9250_FIFO_R_W       ((uint8_t)0x74)
#define MPU9250_WHO_AM_I       ((uint8_t)0x75)

#define MPU9250A_2g            ((float)0.0000610352f)    //0.0000610352 g/LSB
#define MPU9250A_4g            ((float)0.0001220703f)    //0.0001220703 g/LSB
#define MPU9250A_8g            ((float)0.0002441406f)    //0.0002441406 g/LSB
#define MPU9250A_16g           ((float)0.0004882813f)    //0.0004882813 g/LSB

#define MPU9250G_250dps        ((float)0.007633587786f)  //0.007633587786 dps/LSB
#define MPU9250G_500dps        ((float)0.015267175572f)  //0.015267175572 dps/LSB
#define MPU9250G_1000dps       ((float)0.030487804878f)  //0.030487804878 dps/LSB
#define MPU9250G_2000dps       ((float)0.060975609756f)  //0.060975609756 dps/LSB

#define MPU9250T_85degC        ((float)0.00294f)         //0.00294 degC/LSB

int mpu9250_init(void);
void mpu9250_bias_error_estimate(vector3d_f_t *gyro_bias);

void mpu9250_read(vector3d_f_t *gyro_data, vector3d_f_t *accel_data);

#endif
