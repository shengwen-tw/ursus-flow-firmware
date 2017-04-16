#include <stdio.h>
#include <stdlib.h>
#include <usb.h>
#include <libusb-1.0/libusb.h>
#include "opencv2/opencv.hpp"

#define VENDOR_ID  0x0483
#define PRODUCT_ID 0x5740

#define MAX_IMAGE_SIZE (188 * 120)
#define BUFFER_SIZE (MAX_IMAGE_SIZE + 512)

#define PACKET_HEADER_SIZE 20

int image_width = 81;
int image_height = 81;

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
	uint16_t *buffer = (uint16_t *)malloc(sizeof(uint16_t) * BUFFER_SIZE);
	int size_to_receive = MAX_IMAGE_SIZE * sizeof(buffer[0]);
	int received_len;
	cv::Mat cv_image;

	uint16_t lidar_distance = 0;
	float gyro_x = 0, gyro_y = 0, gyro_z = 0;
	uint8_t gyro_calib_enable = 0;

	while(1) {
		/* wait for header message */
		received_len = usb_read((uint8_t *)buffer, size_to_receive, 1000);

		if(received_len == 0 || received_len == -1) {
			printf("\n[usb disconnected]\n");
			break;
		} else if(received_len != PACKET_HEADER_SIZE) {
			continue;
		}

		/* Lidar distance */
		int append_size = 3;

		//unpack header - lidar distance
		memcpy(&lidar_distance, (uint8_t *)buffer + append_size, sizeof(uint16_t));
		append_size += sizeof(uint16_t);

		memcpy(&gyro_calib_enable, (uint8_t *)buffer + append_size, sizeof(uint8_t));
		append_size += sizeof(uint8_t);

		//unpack header - gyro x
		memcpy(&gyro_x, (uint8_t *)buffer + append_size, sizeof(float));
		append_size += sizeof(float);

		//unpack header - gyro y
		memcpy(&gyro_y, (uint8_t *)buffer + append_size, sizeof(float));
		append_size += sizeof(float);

		//unpack header -  gyro z
		memcpy(&gyro_z, (uint8_t *)buffer + append_size, sizeof(float));
		append_size += sizeof(float);

		//unpack header - image width
		memcpy(&image_width, (uint8_t *)buffer + append_size, sizeof(uint8_t));
		append_size += sizeof(uint8_t);

		//unpack header -  image_height
		memcpy(&image_height, (uint8_t *)buffer + append_size, sizeof(uint8_t));
		append_size += sizeof(uint8_t);

		if(gyro_calib_enable == 0) {
			printf("image size: %dx%d\n"
			       "lidar distance: %dcm\n"
			       "gyro_x: %+.3f\n"
			       "gyro_y: %+.3f\n"
			       "gyro_z: %+.3f\n"
			       "\033[2J\033[1;1H",
			       image_width,
			       image_height,
			       lidar_distance,
			       gyro_x,
			       gyro_y,
			       gyro_z);
		} else {
			printf("[gyroscope bias calibration]\n"
			       "bias x: %+.3f\n"
			       "bias y: %+.3f\n"
			       "bias z: %+.3f\n"
			       "\033[2J\033[1;1H",
			       gyro_x,
			       gyro_y,
			       gyro_z);
		}

		/* receive camera image in two parts */
		received_len = usb_read((uint8_t *)buffer, size_to_receive, 1000);
		received_len = usb_read((uint8_t *)buffer + received_len, size_to_receive, 1000);

		if(received_len > 0 && gyro_calib_enable == 0) {
			//printf("received new image, size = %d bytes\n", received_len);

			for(int i = 0; i < image_width * image_height; i++) {
				buffer[i] = buffer[i] << 6;
			}

			//printf("%d\n", buffer[0]);

			cv_image = cv::Mat(image_height, image_width, CV_16UC1, buffer);
			cv::resize(cv_image, cv_image, cv::Size(image_width * 4, image_height * 4));
			cv::cvtColor(cv_image, cv_image, CV_GRAY2BGR); 

			/* 4x downsample visualization */
			int sample_rate = 4;
			for(int x = 0; x < 64; x += sample_rate) {
				for(int y = 0; y < 64; y += sample_rate) {
					cv::circle(cv_image, cv::Point((x + 8) * 4, (y + 8) * 4),
					           1, cv::Scalar(0, 65535, 0),
					           1, CV_AA, 0);
				}
			}

			cv::imshow("ursus-flow camera", cv_image);
			cv::waitKey(1);
		} else if(received_len == 0 || received_len == -1) {
			printf("\n[usb disconnected]\n");
			break;
		}
	}

	/* close and exit */
	free(buffer);
	libusb_close(dev_handle);
	libusb_exit(NULL);
}
