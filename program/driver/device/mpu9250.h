#ifndef __MPU9250_H__
#define __MPU9250_H__

#define MPU9250_CHIP_SELECTOR GPIOC, GPIO_PIN_4

#define MPU9250_SPI &spi1

#define MPU9250_WHO_AM_I (uint8_t)0x75

extern SPI_HandleTypeDef spi1;

void mpu9250_init(void);

#endif
