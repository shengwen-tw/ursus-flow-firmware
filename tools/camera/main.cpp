#include <stdio.h>
#include <stdlib.h>
#include <usb.h>
#include <libusb-1.0/libusb.h>

#define VENDOR_ID  0x0483
#define PRODUCT_ID 0x5740

/* Use lsusb -v to find the correspond values */
static int ep_in_addr  = 0x82;
static int ep_out_addr = 0x01;

struct usb_device *find_usb()
{
	struct usb_bus *busses, *bus;
	struct usb_device *dev;
	struct usb_device_descriptor *desc;

	usb_init();
	usb_find_busses();
	usb_find_devices();

	busses = usb_get_busses();

	for(bus = busses; bus; bus = bus->next) {
		for(dev = bus->devices; dev; dev = dev->next) {
			desc = &(dev->descriptor);
			if((desc->idVendor == VENDOR_ID) && (desc->idProduct == PRODUCT_ID)) {
				return dev;
			}
		}
	}

	return NULL;
}

int main()
{
	struct usb_device *dev;
	struct usb_device_descriptor *desc;

	/* Find usb device with vendor id and product id */
	dev = find_usb();
	desc = &(dev->descriptor);

	if (dev == NULL) {
		printf("Can not find the usb device with provided vendor id and product id!\n");
		return 0;
	}

	//printf("vendor/product id: %04x:%04x\n", desc->idVendor, desc->idProduct);

	/* Open the device and get the token */
	struct usb_dev_handle *dev_handle = usb_open(dev);

	if(dev_handle == NULL) {
		printf("failed to open the device!\n");
		return 0;
	}

	usb_close(dev_handle);
}
