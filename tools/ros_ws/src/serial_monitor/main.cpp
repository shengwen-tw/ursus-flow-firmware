#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "serial/serial.h"
#include "opencv2/opencv.hpp"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

using namespace std;

serial::Serial *_serial;

/* on-board info */
uint16_t lidar_distance = 0;
float gyro_x = 0, gyro_y = 0, gyro_z = 0;
float flow_vx = 0, flow_vy = 0;
float position_x = 0, position_y = 0, position_z = 0;

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
	_serial->read(&s, size);
}

int main(int argc, char **argv)
{
	/* initialize ROS */
	ros::init(argc, argv, "ursusflow_serial");
	ros::Time::init();

	ros::NodeHandle node;
	ros::Publisher flow_vx_publisher = node.advertise<std_msgs::Float32>("/ursusflow/flow_vx", 10);
	ros::Publisher flow_vy_publisher = node.advertise<std_msgs::Float32>("/ursusflow/flow_vy", 10);
	ros::Publisher lidar_distance_publisher =
	        node.advertise<std_msgs::Float32>("/ursusflow/lidar_distance", 10);

	tf::TransformBroadcaster tf_broadcaster;
	tf::Transform transform;

	if(serial_open(port, baudrate) == false) {
		printf("failed to open the serial port.\n");
		return 0;
	}

	while(1) {
		if(_serial->available()) {
			uint8_t buffer;
			serial_gets(&buffer, 1);
			printf("%c", buffer);	
		}

#if 0
		/* send flow velocity message */
		std_msgs::Float32 flow_vx_msg;
		std_msgs::Float32 flow_vy_msg;
		std_msgs::Float32 lidar_distance_msg;
		flow_vx_msg.data = flow_vx;
		flow_vy_msg.data = flow_vy;
		lidar_distance_msg.data = (float)lidar_distance;

		flow_vx_publisher.publish(flow_vx_msg);
		flow_vy_publisher.publish(flow_vy_msg);
		lidar_distance_publisher.publish(lidar_distance_msg);

		position_x += flow_vx * delta_t;
		position_y += flow_vy * delta_t;
		position_z = (float)lidar_distance / 100.0f; //convert unit from cm to m

		transform.setOrigin(tf::Vector3(position_x, position_y, position_z));
		transform.setRotation(tf::Quaternion(0, 0, 0, 1));

		tf_broadcaster.sendTransform(
		        tf::StampedTransform(transform, ros::Time::now(),"origin", "quadrotor"));
#endif
	}

	return 0;
}
