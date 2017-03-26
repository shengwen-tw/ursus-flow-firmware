#include <stdio.h>
#include <stdlib.h>
#include <usb.h>
#include <libusb-1.0/libusb.h>

#define VENDOR_ID  0x0483
#define PRODUCT_ID 0x5740

/* Use lsusb -v to find the correspond values */
static int ep_in_address  = 0x81;
static int ep_out_address = 0x01;

static struct libusb_device_handle *dev_handle = NULL;

int usb_read(uint8_t *buffer, int size, int timeout)
{
	int received_len;
	int rc = libusb_bulk_transfer(dev_handle, ep_in_address,
	                              buffer, size, &received_len, timeout);

	buffer[received_len] = '\0';

	if (rc == LIBUSB_ERROR_TIMEOUT) {
		return 0;
	} else if (rc < 0) {
		return -1;
	}

	return received_len;
}

int main()
{
	int rc;

	/* initialize libusb */
	rc = libusb_init(NULL);
	if (rc < 0) {
		printf("Failed to initialize libusb!\n");
		return 0;
	}

	libusb_set_debug(NULL, 3);

	/* open device to get the token */
	dev_handle = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
	if(!dev_handle) {
		printf("error: device not found (please check vendor id and product id).\n");
		return 0;
	}

	/* claiming interface */
	for(int if_num = 0; if_num < 2; if_num++) {
		if (libusb_kernel_driver_active(dev_handle, if_num)) {
			libusb_detach_kernel_driver(dev_handle, if_num);
		}

		rc = libusb_claim_interface(dev_handle, if_num);
		if (rc < 0) {
			printf("error: failed to claim the usb interface.\n");
			return false;
		}
	}

	/* receive data from usb */
	char buffer[65] = {'\0'};
	while(1) {
		int received_len = usb_read((uint8_t *)buffer, 64, 1000);
		if(received_len > 0) {
			printf("%s", buffer);
		} else if(received_len == 0) {
			printf("usb reception timeout.\n");	
		} else {
			printf("\n[usb disconnected]\n");
			return 0;
		}
	}

	/* close and exit */
	libusb_close(dev_handle);
	libusb_exit(NULL);
}
