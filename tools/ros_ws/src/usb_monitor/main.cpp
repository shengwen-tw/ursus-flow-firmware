#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include <usb.h>
#include <libusb-1.0/libusb.h>
#include "opencv2/opencv.hpp"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#define VENDOR_ID  0x0483
#define PRODUCT_ID 0x5740

#define MAX_IMAGE_SIZE (188 * 120)
#define BUFFER_SIZE (MAX_IMAGE_SIZE + 512)

#define PACKET_HEADER_SIZE 20

#define FLOW_IMG_SIZE 40
#define FLOW_COUNT 32
#define FLOW_MIDPOINT_OFFSET 6
#define FLOW_DISP_SIZE 9

/* Use lsusb -v to find the correspond values */
static int ep_in_address  = 0x81;
static int ep_out_address = 0x01;

static struct libusb_device_handle *dev_handle = NULL;

cv::Mat cv_image;
int image_width = 40;  //Default
int image_height = 40; //Default

/* on-board info */
uint16_t lidar_distance = 0;
float gyro_x = 0, gyro_y = 0, gyro_z = 0;
uint8_t gyro_calib_enable = 0;

uint8_t match_x[FLOW_IMG_SIZE][FLOW_IMG_SIZE];
uint8_t match_y[FLOW_IMG_SIZE][FLOW_IMG_SIZE];

bool _usb_init()
{
	int rc;

	/* initialize libusb */
	rc = libusb_init(NULL);
	if (rc < 0) {
		printf("Failed to initialize libusb!\n");
		return false;
	}

	libusb_set_debug(NULL, 3);

	/* open device to get the token */
	dev_handle = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
	if(!dev_handle) {
		printf("error: device not found (please check vendor id and product id).\n");
		return false;
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

	return true;
}

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

bool usb_receive_onboard_info(uint16_t *buffer)
{
	int received_len;
	int size_to_receive = 6400;
 
	/* wait for header message */
	while(1) {
		received_len = usb_read((uint8_t *)buffer, size_to_receive, 1000);

		if(received_len == 0 || received_len == -1) {
			return false;
		} else if(received_len == PACKET_HEADER_SIZE) {
			break;
		}

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

	/* receive camera image in two parts */
	received_len = usb_read((uint8_t *)buffer, size_to_receive, 1000);

	for(int r = 0; r < FLOW_IMG_SIZE; r++) {
		for(int c = 0; c < FLOW_IMG_SIZE; c++) {
			buffer[r * FLOW_IMG_SIZE + c] = buffer[r * FLOW_IMG_SIZE + c] << 6;
		}
	}

	cv_image = cv::Mat(FLOW_IMG_SIZE, FLOW_IMG_SIZE, CV_16UC1, buffer);
	cv::cvtColor(cv_image, cv_image, CV_GRAY2BGR);
	cv::resize(cv_image, cv_image, cv::Size(FLOW_IMG_SIZE * 4, FLOW_IMG_SIZE * 4));

	//unpack match point
	memcpy((uint8_t *)match_x, (uint8_t *)buffer + 3200, sizeof(uint8_t) * 1600);
	memcpy((uint8_t *)match_y, (uint8_t *)buffer + 4800, sizeof(uint8_t) * 1600);

	if(received_len > 0 && gyro_calib_enable == 0) {
		//printf("received new image, size = %d bytes\n", received_len);
	} else if(received_len == 0 || received_len == -1) {
		return false;
	}

	return true;
}

void flow_visualize()
{
	/* 4x downsampling visualization */
	int sample_rate = 4; //only visualize 1/4 flow on the image
	int flow_start = FLOW_MIDPOINT_OFFSET;
	for(int x = 0; x < FLOW_COUNT; x += sample_rate) {
		for(int y = 0; y < FLOW_COUNT; y += sample_rate) {
			cv::Point start((flow_start + y) * 4, (flow_start + x) * 4);
			cv::Point end(match_y[x][y] * 4, match_x[x][y] * 4);

			cv::circle(cv_image, start, 1, cv::Scalar(0, 0, 65535), 1, CV_AA, 0);

			cv::line(cv_image, start, end, cv::Scalar(0, 65535, 0), 1, 8);
		}
	}
}

int main(int argc, char **argv)
{
	/* initialize ROS */
	ros::init(argc, argv, "ursusflow_usb_sim");
	ros::Time::init();

	ros::NodeHandle node;
	ros::Publisher ros_image_publisher =
	        node.advertise<sensor_msgs::Image>("ursusflow_usb/flow_image", 10);

	if(_usb_init() == false) {
		return 0;
	}

	uint16_t buffer[10000];

	double current_time = 0;
	double previous_time = 0;
	double delta_t = 0;

	while(1) {
		if(usb_receive_onboard_info(buffer) == false) {
			printf("[usb disconnected]\n");
			break;
		}

		current_time = ros::Time::now().toSec();
		delta_t = current_time - previous_time; //calculate delta_t
		previous_time = current_time; //update timer

		flow_visualize();

		/* convert image to ros message and send it */
		cv::Mat cv_image_8u3;
		cv_image.convertTo(cv_image_8u3, CV_8UC3, 1.0 / 256.0); //ros only support this format
		sensor_msgs::ImagePtr ros_image_msg =
			cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image_8u3).toImageMsg();

		ros_image_publisher.publish(ros_image_msg);

		if(gyro_calib_enable == 0) {
			printf("image size: %dx%d\n"
			       "lidar distance: %dcm\n"
			       "gyro_x: %+.3f\n"
			       "gyro_y: %+.3f\n"
			       "gyro_z: %+.3f\n"
			       "fps: %f\n"
			       "delta t: %f\n"
			       "\033[2J\033[1;1H",
			       image_width,
			       image_height,
			       lidar_distance,
			       gyro_x,
			       gyro_y,
			       gyro_z,
			       1.0f / delta_t,
			       delta_t
			      );
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
	}

	printf("[leave]\n");

	/* close and exit */
	cv_image.release();
	libusb_close(dev_handle);
	libusb_exit(NULL);
}
