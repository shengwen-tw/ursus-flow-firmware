#ifndef __USB_LINK_H__
#define __USB_LINK_H__

enum usb_send_mode {
	USB_SEND_IMAGE,
	USB_SEND_GYRO,
	USB_SEND_GYRO_CALIB,
	USB_SEND_LIDAR
};

void usb_link_task(void);

#endif
