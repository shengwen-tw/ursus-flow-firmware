#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

#include "FreeRTOS.h"

#define DRIVER_MAX_PRIORITY configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY //equals 5

/* smaller number means higher priority, valid range: 0 ~ 15 */
#define DCMI_PRIORITY     (DRIVER_MAX_PRIORITY - 2)
#define TIMER10_PRIORITY  (DRIVER_MAX_PRIORITY - 1)
#define SPI1_PRIORITY     (DRIVER_MAX_PRIORITY + 1)
#define EXTI3_PRIORITY    (DRIVER_MAX_PRIORITY + 2)
#define I2C2_PRIORITY     (DRIVER_MAX_PRIORITY + 3)

#define UART2_PRIORITY    (DRIVER_MAX_PRIORITY + 5)
#define USB_FS_PRIORITY   (DRIVER_MAX_PRIORITY + 6)

#endif
