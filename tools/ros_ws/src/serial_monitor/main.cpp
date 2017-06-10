#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "serial/serial.h"
#include "opencv2/opencv.hpp"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include "link.hpp"

#define PACKET_SIZE 23

using namespace std;

serial::Serial *_serial;

fcb_data_t link_data = {
	.flow_vx = 0.0f,
	.flow_vy = 0.0f,
	.time = 0.0f,
	.period = 0.0f,
	.frequency = 0.0f,
	.lidar_distance = 0
};

float position_x = 0.0f;
float position_y = 0.0f;
float position_z = 0.0f;

string port = "/dev/ttyUSB0";
int baudrate = 57600;

bool serial_open(string port, int baudrate)
{
	_serial = new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(serial::Timeout::max()));

	if(_serial->isOpen()) {
		return true;
	} else {
		return false;
	}
}

void serial_gets(uint8_t *s, int size)
{
	_serial->read(s, size);
}

void receive_onboard_params()
{
	uint8_t buffer[PACKET_SIZE];
	int append_size = 1;

	/* wait for start byte */
	do {
		serial_gets(buffer, 1);
	} while(buffer[0] != '$');

	for(int i = 0; i < PACKET_SIZE - 1; i++) {
		serial_gets(buffer + i + 1, 1);
	}

	memcpy(&link_data.lidar_distance, &buffer[append_size], sizeof(uint16_t));
	append_size += sizeof(uint16_t);

	memcpy(&link_data.flow_vx, &buffer[append_size], sizeof(float));
	append_size += sizeof(float);

	memcpy(&link_data.flow_vy, &buffer[append_size], sizeof(float));
	append_size += sizeof(float);

	memcpy(&link_data.time, &buffer[append_size], sizeof(float));
	append_size += sizeof(float);

	memcpy(&link_data.period, &buffer[append_size], sizeof(float));
	append_size += sizeof(float);

	memcpy(&link_data.frequency, &buffer[append_size], sizeof(float));
	append_size += sizeof(float);
}

int main(int argc, char **argv)
{
	/* initialize ROS */
	ros::init(argc, argv, "ursusflow_serial");
	ros::Time::init();

	ros::NodeHandle node;
	ros::Publisher flow_vx_publisher = node.advertise<std_msgs::Float32>("/ursusflow_serial/flow_vx", 10);
	ros::Publisher flow_vy_publisher = node.advertise<std_msgs::Float32>("/ursusflow_serial/flow_vy", 10);
	ros::Publisher lidar_distance_publisher =
	        node.advertise<std_msgs::Float32>("/ursusflow_serial/lidar_distance", 10);

	tf::TransformBroadcaster tf_broadcaster;
	tf::Transform transform;

	if(serial_open(port, baudrate) == false) {
		printf("failed to open the serial port.\n");
		return 0;
	}

	ROS_INFO("\033[2J\033[1;1H"
	         "\033[33m" //yellow
	         "Connection established, waiting for message...");

	while(1) {
		while(!_serial->available());

		receive_onboard_params();

		ROS_INFO("\033[32m" //green
		         "lidar:%3d, vx:%+3.3f, vy:%+3.3f, time:%.1f, delta_t: %.2f, fps:%.1f",
		         link_data.lidar_distance,
		         link_data.flow_vx,
		         link_data.flow_vy,
		         link_data.time,
		         link_data.period,
		         link_data.frequency
		        );
#if 1
		/* send flow velocity message */
		std_msgs::Float32 flow_vx_msg;
		std_msgs::Float32 flow_vy_msg;
		std_msgs::Float32 lidar_distance_msg;
		flow_vx_msg.data = link_data.flow_vx;
		flow_vy_msg.data = link_data.flow_vy;
		lidar_distance_msg.data = (float)link_data.lidar_distance;

		flow_vx_publisher.publish(flow_vx_msg);
		flow_vy_publisher.publish(flow_vy_msg);
		lidar_distance_publisher.publish(lidar_distance_msg);

		position_x += link_data.flow_vx * link_data.period;
		position_y += link_data.flow_vy * link_data.period;
		position_z = (float)link_data.lidar_distance / 100.0f; //convert unit from cm to m

		transform.setOrigin(tf::Vector3(position_x, position_y, position_z));
		transform.setRotation(tf::Quaternion(0, 0, 0, 1));

		tf_broadcaster.sendTransform(
		        tf::StampedTransform(transform, ros::Time::now(),"origin", "quadrotor"));
#endif
	}

	return 0;
}
