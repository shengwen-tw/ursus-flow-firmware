#include <stdio.h>
#include <stdint.h>
#include "opencv2/opencv.hpp"

#include "flow_simulate.hpp"
#include "distance_weighting.hpp"

extern cv::Mat cv_image;

extern flow_t flow;

extern int now;
uint16_t *previous_image;
uint16_t *current_image;

uint32_t calculate_sad16(uint16_t *template_image, uint16_t *search_image)
{
	uint32_t sad = 0;

	int i, j;
	for(i = 0; i < TEMPLATE_SIZE; i++) {
		for(j = 0; j < TEMPLATE_SIZE; j++) {
			sad += abs(template_image[i * FLOW_IMG_SIZE + j] -
			           search_image[i * FLOW_IMG_SIZE + j]);
		}
	}

	/* sad minimum value is 1 since later will do the distance weighting
	   and required not to be 0 */
	return sad + 1;
}

/* Find the matching point on two images in local -4 ~ +4 pixels */
void match_point_local_area(uint16_t *previos_image, uint16_t *current_image,
                            int8_t *match_x, int8_t *match_y)
{
	int8_t sad_min_x = -4, sad_min_y = -4;
	uint32_t sad_min_value = UINT32_MAX;
	uint32_t current_sad;

	int8_t x, y;
	for(x = -4; x <= +4; x++) {
		for(y = -4; y <= +4; y++) {
			current_sad =
			        calculate_sad16(&previos_image[0], &current_image[x * FLOW_IMG_SIZE + y]);

			/* distance weighting */
			current_sad *= distance_weighting_table[x + 4][y + 4];

			if(current_sad < sad_min_value) {
				sad_min_x = x;
				sad_min_y = y;
				sad_min_value = current_sad;
			}
		}
	}

	*match_x = sad_min_x;
	*match_y = sad_min_y;
}

void flow_estimate(uint16_t *previos_image, uint16_t *current_image)
{
	/* convert the 72x72 start address into 64x64 address */
	int offset = TEMPLATE_MIDPOINT_OFFSET + TEMPLATE_SEARCH_SUBAREA_OFFSET;
	int start_x, start_y;
	uint16_t *frame1;
	uint16_t *frame2;

	int8_t match_x = 0, match_y = 0; //match point relative to the local flow position

	float mean_displacement = 0.0f;
	int valid_count = 0;

	/* calculate the flow for every 64x64 points */
	int x, y;
	for(x = 0; x < FLOW_COUNT; x++) {
		for(y = 0; y < FLOW_COUNT; y++) {
			start_x = x + offset;
			start_y = y + offset;
			frame1 = &previos_image[start_x * FLOW_IMG_SIZE + start_y];
			frame2 = &current_image[start_x * FLOW_IMG_SIZE + start_y];
			match_point_local_area(frame1, frame2, &match_x, &match_y);

			/* convert the position relative the full image */
			flow.match_x[x][y] = match_x + start_x;
			flow.match_y[x][y] = match_y + start_y;

			float displcement = sqrt(match_x * match_x + match_y * match_y);

			if(displcement > 0.0f) {
				valid_count++;
				mean_displacement += displcement;
			}
		}
	}

	if(valid_count > FLOW_THRESHOLD) {
		mean_displacement /= valid_count;
		printf("mean displacement: %f pixels\n", mean_displacement);
	}
}

void simulate_opical_flow_on_pc()
{
	int last = (now + 1) % 2;

	current_image = (uint16_t *)flow.image[now].frame;
	previous_image = (uint16_t *)flow.image[last].frame;

	/* flow estimation */
	flow_estimate(previous_image, current_image);

	/* visualization */
	cv::cvtColor(cv_image, cv_image, CV_GRAY2BGR);

	/* 4x downsampling visualization */
	int sample_rate = 4; //only visualize 1/4 flow on the image
	int flow_start = TEMPLATE_MIDPOINT_OFFSET + TEMPLATE_SEARCH_SUBAREA_OFFSET;
	for(int x = 0; x < 64; x += sample_rate) {
		for(int y = 0; y < 64; y += sample_rate) {
			cv::Point start((flow_start + y) * 4, (flow_start + x) * 4);
			cv::Point end(flow.match_y[x][y] * 4, flow.match_x[x][y] * 4);

			cv::circle(cv_image, start, 1, cv::Scalar(0, 0, 65535), 1, CV_AA, 0);

			cv::line(cv_image, start, end, cv::Scalar(0, 65535, 0), 1, 8);
		}
	}
}
