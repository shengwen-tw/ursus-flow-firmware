#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "core.h"
#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "timer.h"
#include "pwm.h"
#include "i2c.h"
#include "dcmi.h"
#include "usb_device.h"

#include "mpu9250.h"
#include "mt9v034.h"
#include "lidar.h"

#include "flow.h"
#include "usb_link.h"
#include "fcb_link.h"
#include "system_time.h"

extern uint16_t lidar_distance;

vector3d_f_t gyro_bias;

int main(void)
{
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	mpu_init();
	cpu_cache_enable();
	system_clock_init();

	timer_init();
	gpio_init();
	uart_init();
	spi_init();
	pwm_init();
	i2c_init();
	dcmi_init();

	/* wait until the mcu peripherial initialization is finished */
	delay_ms(5);

	if(mpu9250_init()) {
		while(1); //This is bad
	}

	if(mt9v034_init()) {
		while(1); //This is bad
	}

	lidar_init(&lidar_distance);

	#if (DO_IMU_CALIBRATION != 0)
		mpu9250_bias_error_estimate(&gyro_bias);
	#endif

	//successfully initialized the hardware
	gpio_on(LED_3); //red led

	usb_fs_init();

	flow_estimate_task();

	return 0;
}
