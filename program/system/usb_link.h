#ifndef __USB_LINK_H__
#define __USB_LINK_H__

/* USB should only be enabled during the development stage,
   otherwise it may slow down low flow estimation process */
#define DISABLE_USB 1

void usb_link_task(void);

void usb_image_foward(void);

#if (DISABLE_USB == 0)
void usb_send_flow_info(void);
#else
#define usb_send_flow_info(...)
#endif

#endif
