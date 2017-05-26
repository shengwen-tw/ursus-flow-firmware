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

#if 0 //not using in this program
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
#endif

uint32_t calculate_ssd16_row(uint16_t *template_image, uint16_t *search_image, int row_offset)
{
	uint16_t *_template = template_image;
	uint16_t *_search = search_image;

	uint32_t ssd = 0;
	int16_t diff;

	_template += row_offset * FLOW_IMG_SIZE;
	_search += row_offset * FLOW_IMG_SIZE;

	int r;
	for(r = 0; r < TEMPLATE_SIZE; r++) {
		diff = *_template - *_search;
		ssd += diff * diff;

		_template++;
		_search++;
	}

	return ssd;
}

uint32_t calculate_ssd16_column(uint16_t *template_image, uint16_t *search_image, int column_offset)
{
	uint16_t *_template = template_image;
	uint16_t *_search = search_image;

	uint32_t ssd = 0;
	int16_t diff;

	_template += column_offset;
	_search += column_offset;

	int c;
	for(c = 0; c < TEMPLATE_SIZE; c++) {
		diff = *_template - *_search;
		ssd += diff * diff;

		_template += FLOW_IMG_SIZE;
		_search += FLOW_IMG_SIZE;
	}

	return ssd;
}

/* calculate sum of squared difference for 10-bits image */
uint32_t calculate_ssd16_full(uint16_t *template_image, uint16_t *search_image)
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
void match_point_local_area_full(uint16_t *previous_image, uint16_t *current_image,
                                  int8_t *match_x, int8_t *match_y)
{
	int8_t ssd_min_x = -4, ssd_min_y = -4;
	uint32_t ssd_min_value = UINT32_MAX;
	uint32_t ssd;
	uint32_t dw_ssd;

	uint16_t *running_search_image = &current_image[0];
	uint16_t *fixed_template_image = &previous_image[0];

	int8_t r, c;
	for(r = -4; r <= +4; r++) {
		running_search_image = &current_image[FLOW_IMG_SIZE * r];

		for(c = -4; c <= +4; c++) {
			ssd = calculate_ssd16_full(fixed_template_image, &running_search_image[c]);

			flow.subarea_ssd_row_start[r + 4][c + 4] = ssd;
			flow.subarea_ssd_last[r + 4][c + 4] = ssd;

			/* distance weighting */
			dw_ssd = ssd * distance_weighting_table[r + 4][c + 4];

			if(dw_ssd < ssd_min_value) {
				ssd_min_x = r;
				ssd_min_y = c;
				ssd_min_value = dw_ssd;
			}
		}
	}

	/* bad result */
//	if(sad_min_value > BLOCK_MATCHING_THRESHOLD) {
//		*match_x = 0;
//		*match_y = 0;
//

	*match_x = ssd_min_x;
	*match_y = ssd_min_y;
}

void match_point_local_area_row_dp(uint16_t *previous_image, uint16_t *current_image,
                                      int8_t *match_x, int8_t *match_y)
{
	int8_t ssd_min_x = -4, ssd_min_y = -4;
	uint32_t ssd_min_value = UINT32_MAX;
	uint32_t ssd;
	uint32_t dw_ssd;

	uint16_t *running_search_image = &current_image[0];
	uint16_t *fixed_template_image = &previous_image[0];

	uint32_t row_cuttoff_ssd;
	uint32_t row_addin_ssd;

	int8_t r, c;
	for(r = -4; r <= +4; r++) {
		running_search_image = &current_image[FLOW_IMG_SIZE * r];

		for(c = -4; c <= +4; c++) {
			row_cuttoff_ssd = calculate_ssd16_row(fixed_template_image, &running_search_image[c], -1);
			row_addin_ssd = calculate_ssd16_row(fixed_template_image, &running_search_image[c], 7);

			ssd = flow.subarea_ssd_row_start[r + 4][c + 4];
			//cut left old edge and add right new edge
			ssd -= row_cuttoff_ssd;
			ssd += row_addin_ssd;
			//update ssd table
			flow.subarea_ssd_row_start[r + 4][c + 4] = ssd;
			flow.subarea_ssd_last[r + 4][c + 4] = ssd;

			//distance weighting
			dw_ssd = ssd * distance_weighting_table[r + 4][c + 4];

			if(dw_ssd < ssd_min_value) {
				ssd_min_x = r;
				ssd_min_y = c;
				ssd_min_value = dw_ssd;
			}
		}
	}

	/* bad result */
//	if(sad_min_value > BLOCK_MATCHING_THRESHOLD) {
//		*match_x = 0;
//		*match_y = 0;
//

	*match_x = ssd_min_x;
	*match_y = ssd_min_y;
}

void match_point_local_area_column_dp(uint16_t *previous_image, uint16_t *current_image,
                                   int8_t *match_x, int8_t *match_y)
{
	int8_t ssd_min_x = -4, ssd_min_y = -4;
	uint32_t ssd_min_value = UINT32_MAX;
	uint32_t ssd;
	uint32_t dw_ssd;

	uint16_t *running_search_image = &current_image[0];
	uint16_t *fixed_template_image = &previous_image[0];

	uint32_t column_cuttoff_ssd;
	uint32_t column_addin_ssd;

	int8_t r, c;
	for(r = -4; r <= +4; r++) {
		running_search_image = &current_image[FLOW_IMG_SIZE * r];

		for(c = -4; c <= +4; c++) {
			column_cuttoff_ssd = calculate_ssd16_column(fixed_template_image, &running_search_image[c], -1);
			column_addin_ssd = calculate_ssd16_column(fixed_template_image, &running_search_image[c], 7);

			//read last ssd
			ssd = flow.subarea_ssd_last[r + 4][c + 4];
			//cut left old edge and add right new edge
			ssd -= column_cuttoff_ssd;
			ssd += column_addin_ssd;
			//update ssd table
			flow.subarea_ssd_last[r + 4][c + 4] = ssd;

			//distance weighting
			dw_ssd = ssd * distance_weighting_table[r + 4][c + 4];

			if(dw_ssd < ssd_min_value) {
				ssd_min_x = r;
				ssd_min_y = c;
				ssd_min_value = dw_ssd;
			}
		}
	}

	/* bad result */
//	if(sad_min_value > BLOCK_MATCHING_THRESHOLD) {
//		*match_x = 0;
//		*match_y = 0;
//

	*match_x = ssd_min_x;
	*match_y = ssd_min_y;
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
	int vote_x, vote_y;
	int vote_count = 0;

	float predict_disp_x = 0, predict_disp_y = 0;
	
	/* estimate the first flow by calculating all ssd */
	start_x = 0 + offset;
	start_y = 0 + offset;
	frame1 = &previous_image[start_x * FLOW_IMG_SIZE + start_y];
	frame2 = &current_image[start_x * FLOW_IMG_SIZE + start_y];
	match_point_local_area_full(frame1, frame2, &match_x, &match_y);

	flow.match_x[0][0] = match_x + start_x;
	flow.match_y[0][0] = match_y + start_y;

	//if not both equal zero
	if(match_x || match_y) {
		/* histogram voting */
		vote_x =  match_x + 4;
		vote_y =  match_y + 4;
		histogram_x[vote_x]++;
		histogram_y[vote_y]++;

		predict_disp_x += match_x;
		predict_disp_y += match_y;

		vote_count++;
	}

	/* row iteration for first row */
	int x, y;
	for(y = 1; y < FLOW_COUNT; y++) {
		start_x = 0 + offset;
		start_y = y + offset;
		frame1 = &previous_image[start_x * FLOW_IMG_SIZE + start_y];
		frame2 = &current_image[start_x * FLOW_IMG_SIZE + start_y];
	
		//XXX:match_point_local_area_column_dp(frame1, frame2, &match_x, &match_y);
		match_point_local_area_full(frame1, frame2, &match_x, &match_y);


		/* convert the position relative the full image */
		flow.match_x[0][y] = match_x + start_x;
		flow.match_y[0][y] = match_y + start_y;

		//if not both equal zero
		if(match_x || match_y) {
			/* histogram voting */
			vote_x =  match_x + 4;
			vote_y =  match_y + 4;
			histogram_x[vote_x]++;
			histogram_y[vote_y]++;

			predict_disp_x += match_x;
			predict_disp_y += match_y;

			vote_count++;
		} else {
			continue;
		}
	}

	/* estimate the flow by only calculate the non-overlap region's ssd (start from second row) */
	for(x = 1; x < FLOW_COUNT; x++) {
		/* row iteration (go down) */
		start_x = x + offset;
		start_y = 0 + offset;
		frame1 = &previous_image[start_x * FLOW_IMG_SIZE + start_y];
		frame2 = &current_image[start_x * FLOW_IMG_SIZE + start_y];
	
		//XXX:match_point_local_area_row_dp(frame1, frame2, &match_x, &match_y);
		match_point_local_area_full(frame1, frame2, &match_x, &match_y);

		/* convert the position relative the full image */
		flow.match_x[x][0] = match_x + start_x;
		flow.match_y[x][0] = match_y + start_y;

		//if not both equal zero
		if(match_x || match_y) {
			/* histogram voting */
			vote_x =  match_x + 4;
			vote_y =  match_y + 4;
			histogram_x[vote_x]++;
			histogram_y[vote_y]++;

			predict_disp_x += match_x;
			predict_disp_y += match_y;

			vote_count++;
		}

		/* column iteration (go right) */
		for(y = 1; y < FLOW_COUNT; y++) {
			start_x = x + offset;
			start_y = y + offset;
			frame1 = &previous_image[start_x * FLOW_IMG_SIZE + start_y];
			frame2 = &current_image[start_x * FLOW_IMG_SIZE + start_y];
	
			//XXX:match_point_local_area_column_dp(frame1, frame2, &match_x, &match_y);
			match_point_local_area_full(frame1, frame2, &match_x, &match_y);


			/* convert the position relative the full image */
			flow.match_x[x][y] = match_x + start_x;
			flow.match_y[x][y] = match_y + start_y;

			//if not both equal zero
			if(match_x || match_y) {
				/* histogram voting */
				vote_x =  match_x + 4;
				vote_y =  match_y + 4;
				histogram_x[vote_x]++;
				histogram_y[vote_y]++;

				predict_disp_x += match_x;
				predict_disp_y += match_y;

				vote_count++;
			} else {
				continue;
			}
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