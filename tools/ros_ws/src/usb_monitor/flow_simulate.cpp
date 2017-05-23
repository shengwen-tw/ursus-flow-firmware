#include <stdio.h>
#include <stdint.h>
#include "opencv2/opencv.hpp"

#include "flow_simulate.hpp"
#include "distance_weighting.hpp"

extern cv::Mat cv_image;

extern uint16_t lidar_distance;
extern float gyro_x, gyro_y, gyro_z;
extern flow_t flow;

extern int now;
uint16_t *previous_image;
uint16_t *current_image;

float focal_length_mm = FOCAL_LENGTH_PX * (1.0f / RETINA_SIZE); //f_mm = f_px * m

/* calculate sum of absoulte difference for 10-bits image */
uint32_t calculate_sad16(uint16_t *template_image, uint16_t *search_image)
{
	/* sad minimum value is 1 since later will do the distance weighting
	   and required not to be 0 */
	uint32_t sad = 1;

	int i, j;
	for(i = 0; i < TEMPLATE_SIZE; i++) {
		for(j = 0; j < TEMPLATE_SIZE; j++) {
			sad += abs(template_image[i * FLOW_IMG_SIZE + j] -
			           search_image[i * FLOW_IMG_SIZE + j]);
		}
	}

	return sad;
}

uint32_t calculate_sad16_row(uint16_t *template_image, uint16_t *search_image, int row_number)
{
	uint16_t *_template = template_image;
	uint16_t *_search = search_image;

	uint32_t ssd = 0;
	int16_t diff;

	_template += row_number * TEMPLATE_SIZE;
	_search += row_number * TEMPLATE_SIZE;

	for(int i = 0; i < TEMPLATE_SIZE; i++) {
			diff = *_template - *_search;
			ssd += diff * diff;

			_template++;
			_search++;
	}

	return ssd;
}

uint32_t calculate_sad16_column(uint16_t *template_image, uint16_t *search_image, int column_number)
{
	uint16_t *_template = template_image;
	uint16_t *_search = search_image;

	uint32_t ssd = 0;
	int16_t diff;

	_template += column_number;
	_search += column_number;

	for(int j = 0; j < TEMPLATE_SIZE; j++) {
			diff = *_template - *_search;
			ssd += diff * diff;

			_template += TEMPLATE_SIZE;
			_search += TEMPLATE_SIZE;
	}

	return ssd;
}

/* calculate sum of squared difference for 10-bits image */
uint32_t calculate_ssd16(uint16_t *template_image, uint16_t *search_image)
{
	/* ssd minimum value is 1 since later will do the distance weighting
	   and required not to be 0 */
	uint32_t ssd = 1;

	uint16_t *_template = template_image;
	uint16_t *_search = search_image;

	int i, j;
	for(i = 0; i < TEMPLATE_SIZE; i++) {
		_template += FLOW_IMG_SIZE;
		_search += FLOW_IMG_SIZE;

		for(j = 0; j < TEMPLATE_SIZE; j++) {
			int16_t diff = _template[j] - _search[j];

			ssd += diff * diff;
		}
	}

	/* ssd minimum value is 1 since later will do the distance weighting
	   and required not to be 0 */
	return ssd;
}

/* Find the matching point on two images in local -4 ~ +4 pixels */
void match_point_local_area(uint16_t *previous_image, uint16_t *current_image,
                            int8_t *match_x, int8_t *match_y)
{
	int8_t sd_min_x = -4, sd_min_y = -4;
	uint32_t sd_min_value = UINT32_MAX;
	uint32_t current_sd;

	uint16_t *current_image_run = current_image;

	int8_t x, y;
	for(x = -4; x <= +4; x++) {
		current_image_run = &current_image[FLOW_IMG_SIZE * x];

		for(y = -4; y <= +4; y++) {
			current_sd = calculate_ssd16(&previous_image[0], &current_image_run[y]);

			/* distance weighting */
			current_sd *= distance_weighting_table[x + 4][y + 4];

			if(current_sd < sd_min_value) {
				sd_min_x = x;
				sd_min_y = y;
				sd_min_value = current_sd;
			}
		}
	}

	/* bad result */
//	if(sad_min_value > BLOCK_MATCHING_THRESHOLD) {
//		*match_x = 0;
//		*match_y = 0;
//

	*match_x = sd_min_x;
	*match_y = sd_min_y;
}

void flow_estimate(uint16_t *previous_image, uint16_t *current_image,
                   float *flow_vx, float *flow_vy, float delta_t)
{

	/* convert the 72x72 start address into 64x64 address */
	int offset = TEMPLATE_MIDPOINT_OFFSET + TEMPLATE_SEARCH_SUBAREA_OFFSET;
	int start_x, start_y;
	uint16_t *frame1;
	uint16_t *frame2;

	int8_t match_x = 0, match_y = 0; //match point relative to the local flow position

	/* histogram filter */
	//x, y displacement in range of -4 ~ +4 (9 possibilities)
	uint16_t histogram_x[FLOW_DISP_SIZE] = {0};
	uint16_t histogram_y[FLOW_DISP_SIZE] = {0};
	int8_t highest_vote_x = 0, highest_vote_y = 0;
	int vote_count = 0;

	float predict_disp_x = 0, predict_disp_y = 0;

	/* calculate the flow for every 64x64 points */
	int x, y;
	for(x = 0; x < FLOW_COUNT; x++) {
		for(y = 0; y < FLOW_COUNT; y++) {
			/* calculate the matching point using SAD */
			start_x = x + offset;
			start_y = y + offset;
			frame1 = &previous_image[start_x * FLOW_IMG_SIZE + start_y];
			frame2 = &current_image[start_x * FLOW_IMG_SIZE + start_y];
	
			match_point_local_area(frame1, frame2, &match_x, &match_y);
			/* convert the position relative the full image */
			flow.match_x[x][y] = match_x + start_x;
			flow.match_y[x][y] = match_y + start_y;

			if(match_x == 0 && match_y == 0) {
				continue;
			}

			/* histogram voting */
			int vote_x =  match_x + 4;
			int vote_y =  match_y + 4;
			histogram_x[vote_x]++;
			histogram_y[vote_y]++;

			predict_disp_x += match_x;
			predict_disp_y += match_y;

			vote_count++;
		}
	}

	printf("%d\n", vote_count);

	if(vote_count < HISTOGRAM_THRESHOLD) {
		predict_disp_x = 0.0f;
		predict_disp_y = 0.0f;

		*flow_vx = 0;
		*flow_vy = 0;

		return;
	}

	predict_disp_x /= (float)vote_count;
	predict_disp_y /= (float)vote_count;

	/* flow unit: [mm/s] */
	float flow_px_vx = +((float)lidar_distance * 10.0f / FOCAL_LENGTH_PX * predict_disp_x) / delta_t;
	float flow_px_vy = -((float)lidar_distance * 10.0f / FOCAL_LENGTH_PX * predict_disp_y) / delta_t;

	/* rotation compensation */
	*flow_vx = flow_px_vx;
	*flow_vy = flow_px_vy;

	/* connvert to [cm/s] */
	*flow_vx /= 10.0f;
	*flow_vy /= 10.0f;

#if 0
	printf("x: %f, y: %f\n"
	       "x vote count: %d, y vote count: %d\n",
	       predict_disp_x, predict_disp_y,
	       histogram_y[highest_vote_x], histogram_y[highest_vote_y]);
#endif
}

void simulate_opical_flow_on_pc(float *flow_vx, float *flow_vy, float delta_t)
{
	int last = (now + 1) % 2;

	current_image = (uint16_t *)flow.image[now].frame;
	previous_image = (uint16_t *)flow.image[last].frame;

	if(gyro_z > 1.0f) {
		*flow_vx = 0;
		*flow_vy = 0;
	}

	/* flow estimation */
	flow_estimate(previous_image, current_image, flow_vx, flow_vy, delta_t);
}

void flow_visualize()
{
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
