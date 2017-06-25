#include <stdlib.h>
#include <stdbool.h>

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "gpio.h"
#include "timer.h"

#include "mpu9250.h"
#include "mt9v034.h"
#include "lidar.h"

#include "imu.h"

#include "flow.h"
#include "usb_link.h"
#include "fcb_link.h"
#include "system_time.h"
#include "distance_weighting.h"
#include "ssd16.h"
#include "kalman_filter.h"

extern flow_t flow; //declare symbol in linker script (.dtcm section)

vector3d_f_t gyro_data;
vector3d_f_t accel_data;
uint16_t lidar_distance;

__attribute__((section(".itcmtext")))
uint32_t calculate_ssd16_row(uint16_t *template_image, uint16_t *search_image, int row_offset)
{
	uint16_t *_template = template_image;
	uint16_t *_search = search_image;

	uint32_t ssd[2] = {0, 1};
	int16_t diff[2];

	_template += row_offset * FLOW_IMG_SIZE;
	_search += row_offset * FLOW_IMG_SIZE;

	int r;
	for(r = 0; r < (TEMPLATE_SIZE / 2); r++) {
		*((uint32_t *)diff) = __SSUB16(*(((uint32_t *)_template) + r), *(((uint32_t *)_search) + r));
		*((uint64_t *)ssd) = __SMLALD(*((uint32_t *)diff), *((uint32_t *)diff), *((uint64_t *)ssd));

#if 0
		diff = *_template - *_search;
		ssd += diff * diff;
#endif
	}

	return ssd[0] + ssd[1];
}

__attribute__((section(".itcmtext")))
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

__attribute__((section(".itcmtext")))
uint32_t calculate_ssd16_full(uint16_t *template_image, uint16_t *search_image)
{
	/* ssd minimum value is 1 since later will do the distance weighting
	   and required not to be 0 */
	uint32_t ssd[2] = {0, 1};
	int16_t diff[2];

	uint16_t *_template = template_image;
	uint16_t *_search = search_image;

	int r, c;
	for(r = 0; r < TEMPLATE_SIZE; r++) {
		for(c = 0; c < (TEMPLATE_SIZE / 2); c++) {
			*((uint32_t *)diff) =
			        __SSUB16(*(((uint32_t *)_template) + c), *(((uint32_t *)_search) + c));
			*((uint64_t *)ssd) =
			        __SMLALD(*((uint32_t *)diff), *((uint32_t *)diff), *((uint64_t *)ssd));

#if 0
			int16_t diff = _template[c] - _search[c];
			ssd += diff * diff;
#endif
		}

		/* point to next row */
		_template += FLOW_IMG_SIZE;
		_search += FLOW_IMG_SIZE;
	}

	return ssd[0] + ssd[1];
}

/* Find the matching point on two images in local -4 ~ +4 pixels */
__attribute__((section(".itcmtext")))
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

			flow.subarea_ssd_column_start[r + 4][c + 4] = ssd;
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

#if 0
	/* bad result */
	if(ssd_min_value > BLOCK_MATCHING_THRESHOLD) {
		*match_x = 0;
		*match_y = 0;
	}
#endif

	*match_x = ssd_min_x;
	*match_y = ssd_min_y;
}

__attribute__((section(".itcmtext")))
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

			ssd = flow.subarea_ssd_column_start[r + 4][c + 4];
			//cut left old edge and add right new edge
			ssd -= column_cuttoff_ssd;
			ssd += column_addin_ssd;
			//update ssd table
			flow.subarea_ssd_column_start[r + 4][c + 4] = ssd;
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

#if 0
	/* bad result */
	if(ssd_min_value > BLOCK_MATCHING_THRESHOLD) {
		*match_x = 0;
		*match_y = 0;
	}
#endif

	*match_x = ssd_min_x;
	*match_y = ssd_min_y;
}

__attribute__((section(".itcmtext")))
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

			//read last ssd
			ssd = flow.subarea_ssd_last[r + 4][c + 4];
			//cut left old edge and add right new edge
			ssd -= row_cuttoff_ssd;
			ssd += row_addin_ssd;
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

#if 0
	/* bad result */
	if(ssd_min_value > BLOCK_MATCHING_THRESHOLD) {
		*match_x = 0;
		*match_y = 0;

	}
#endif

	*match_x = ssd_min_x;
	*match_y = ssd_min_y;
}

__attribute__((section(".itcmtext")))
void flow_estimate(uint16_t *previous_image, uint16_t *current_image,
                   float *flow_vx, float *flow_vy, float delta_t)
{
	/* convert the 40x40 start address into 32*32 address */
	int offset = EDGE_PRESERVE_SIZE;
	int start_x, start_y;
	uint16_t *frame1;
	uint16_t *frame2;

	int8_t match_x = 0, match_y = 0; //match point relative to the local flow position

	/* histogram filter */
	//x, y displacement in range of -4 ~ +4 (9 possibilities)
	uint16_t histogram_x[FLOW_DISP_SIZE] = {0};
	uint16_t histogram_y[FLOW_DISP_SIZE] = {0};
	int vote_x, vote_y;
	int vote_count = 0;

	float predict_disp_x = 0, predict_disp_y = 0;

	/* estimate the first flow by calculating all ssd */
	start_x = 0 + offset;
	start_y = 0 + offset;
	frame1 = &previous_image[start_x * FLOW_IMG_SIZE + start_y];
	frame2 = &current_image[start_x * FLOW_IMG_SIZE + start_y];
	match_point_local_area_full(frame1, frame2, &match_x, &match_y);

#if (DISABLE_USB == 0)
	flow.match_x[0][0] = match_x + (FLOW_MIDPOINT_OFFSET + 0);
	flow.match_y[0][0] = match_y + (FLOW_MIDPOINT_OFFSET + 0);
#endif

	//if not both equal zero
	if(match_x - match_y) {
		/* histogram voting */
		vote_x =  match_x + 4;
		vote_y =  match_y + 4;
		histogram_x[vote_x]++;
		histogram_y[vote_y]++;

		predict_disp_x += match_x;
		predict_disp_y += match_y;

		vote_count++;
	}

	/* column iteration for first column */
	int r, c;
	for(r = 1; r < FLOW_COUNT; r++) {
		start_x = r + offset;
		start_y = 0 + offset;
		frame1 = &previous_image[start_x * FLOW_IMG_SIZE + start_y];
		frame2 = &current_image[start_x * FLOW_IMG_SIZE + start_y];

		match_point_local_area_row_dp(frame1, frame2, &match_x, &match_y);

#if (DISABLE_USB == 0)
		/* convert the position relative the full image */
		flow.match_x[r][0] = match_x + (FLOW_MIDPOINT_OFFSET + r);
		flow.match_y[r][0] = match_y + (FLOW_MIDPOINT_OFFSET + 0);
#endif

		//if not both equal zero
		if(match_x - match_y) {
			/* histogram voting */
			vote_x =  match_x + 4;
			vote_y =  match_y + 4;
			histogram_x[vote_x]++;
			histogram_y[vote_y]++;

			predict_disp_x += match_x;
			predict_disp_y += match_y;

			vote_count++;
		}
	}

	/* estimate the flow by only calculate the non-overlap region's ssd (start from second row) */
	for(c = 1; c < FLOW_COUNT; c++) {
		/* column iteration (go down) */
		start_x = 0 + offset;
		start_y = c + offset;
		frame1 = &previous_image[start_x * FLOW_IMG_SIZE + start_y];
		frame2 = &current_image[start_x * FLOW_IMG_SIZE + start_y];

		match_point_local_area_column_dp(frame1, frame2, &match_x, &match_y);

#if (DISABLE_USB == 0)
		/* convert the position relative the full image */
		flow.match_x[0][c] = match_x + (FLOW_MIDPOINT_OFFSET + 0);
		flow.match_y[0][c] = match_y + (FLOW_MIDPOINT_OFFSET + c);
#endif

		//if not both equal zero
		if(match_x - match_y) {
			/* histogram voting */
			vote_x =  match_x + 4;
			vote_y =  match_y + 4;
			histogram_x[vote_x]++;
			histogram_y[vote_y]++;

			predict_disp_x += match_x;
			predict_disp_y += match_y;

			vote_count++;
		}

		/* row iteration (go down) */
		for(r = 1; r < FLOW_COUNT; r++) {
			start_x = r + offset;
			start_y = c + offset;
			frame1 = &previous_image[start_x * FLOW_IMG_SIZE + start_y];
			frame2 = &current_image[start_x * FLOW_IMG_SIZE + start_y];

			match_point_local_area_row_dp(frame1, frame2, &match_x, &match_y);

#if (DISABLE_USB == 0)
			/* convert the position relative the full image */
			flow.match_x[r][c] = match_x + (FLOW_MIDPOINT_OFFSET + r);
			flow.match_y[r][c] = match_y + (FLOW_MIDPOINT_OFFSET + c);
#endif

			//if not both equal zero
			if(match_x - match_y) {
				/* histogram voting */
				vote_x =  match_x + 4;
				vote_y =  match_y + 4;
				histogram_x[vote_x]++;
				histogram_y[vote_y]++;

				predict_disp_x += match_x;
				predict_disp_y += match_y;

				vote_count++;
			}
		}
	}

	if(vote_count < HISTOGRAM_THRESHOLD) {
		*flow_vx = 0;
		*flow_vy = 0;

		gpio_off(LED_2); //no flow

		return;
	}

	predict_disp_x /= (float)vote_count;
	predict_disp_y /= (float)vote_count;

	/* flow unit: [cm/s] */
	*flow_vx = +((float)lidar_distance / FOCAL_LENGTH_PX * predict_disp_x) / delta_t;
	*flow_vy = -((float)lidar_distance / FOCAL_LENGTH_PX * predict_disp_y) / delta_t;

	gpio_on(LED_2); //flow detected
}

__attribute__((section(".itcmtext")))
void flow_estimate_task(void)
{
	float current_time;
	float previous_time;
	float delta_t;
	float fps;

	int now = 0, next = 1;

	float flow_vx = 0.0f, flow_vy = 0.0f;
	float kalman_vx = 0.0f, kalman_vy = 0.0f;

	mt9v034_start_capture_image((uint32_t)flow.image[now].frame);
	mt9v034_wait_finish();
	previous_time = get_time_sec();

	mt9v034_start_capture_image((uint32_t)flow.image[next].frame);

	while(1) {
		gpio_on(LED_1);

		lidar_read();

		mpu9250_read(&gyro_data, &accel_data);

		/* wait until image finished capturing */
		mt9v034_wait_finish();

		current_time = get_time_sec();
		delta_t = current_time - previous_time;
		previous_time = current_time;
		fps = 1.0f / delta_t;

#if (IMAGE_FOWARD_NO_FLOW == 1)
		mt9v034_start_capture_image((uint32_t)flow.image[0].frame);

		usb_image_foward();
#else
		flow_estimate(
		        (uint16_t *)flow.image[now].frame,
		        (uint16_t *)flow.image[next].frame,
		        &flow_vx, &flow_vy, delta_t
		);

		mt9v034_start_capture_image((uint32_t)flow.image[next].frame);

		usb_send_flow_info();
#endif
		now = next;
		next = (next + 1) % 2;

		accel_data.x *= 9.8f;
		accel_data.y *= 9.8f;
		accel_data.z *= 9.8f;

		kalman_filter(&kalman_vx, &kalman_vy, flow_vx, flow_vy,
		              accel_data.x, accel_data.y, delta_t);

		send_flow_to_fcb(&lidar_distance, &kalman_vx, &kalman_vy, &current_time, &delta_t, &fps);

		//send_debug_message("lidar:%3d, vx:%+2.3f, vy:%+2.3f, time:%.1f, delta_t:%1f, fps:%.1f\n\r",
		//                   lidar_distance, flow_vx, flow_vy, current_time, delta_t, fps);

		/* print sensor info in the following csv format:
		 * flow_vx, flow_vy, accel_ax, accel_ay, delta_t, system_time */
		//send_debug_message("%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f\n\r",
		//                   flow_vx, flow_vy, accel_data.x, accel_data.y, delta_t, current_time);

		gpio_off(LED_1);
	}
}
