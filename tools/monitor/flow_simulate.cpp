#include <stdio.h>
#include "opencv2/opencv.hpp"

#include "flow_simulate.hpp"

extern cv::Mat cv_image;

extern flow_t flow;

void simulate_opical_flow_on_pc()
{
	cv::cvtColor(cv_image, cv_image, CV_GRAY2BGR);

	/* 4x downsample visualization */
	int sample_rate = 4;
	for(int x = 0; x < 64 + 1; x += sample_rate) {
		for(int y = 0; y < 64 + 1; y += sample_rate) {
			cv::circle(cv_image, cv::Point((x + 4) * 4, (y + 4) * 4),
			           1, cv::Scalar(0, 0, 65535),
			           1, CV_AA, 0);
		}
	}
}
