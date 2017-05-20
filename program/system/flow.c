#include <stdlib.h>
#include <stdbool.h>

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "gpio.h"
#include "timer.h"

#include "mpu9250.h"
#include "mt9v034.h"
#include "lidar.h"

#include "delay.h"
#include "imu.h"

#include "flow.h"
#include "usb_link.h"
#include "fcb_link.h"
#include "system_time.h"
#include "distance_weighting.h"

extern TaskHandle_t fcb_link_task_handle;
extern TaskHandle_t usb_link_task_handle;

SemaphoreHandle_t flow_task_semaphore;

flow_t flow;

vector3d_f_t gyro_data;
uint16_t lidar_distance;

bool do_gyro_calibrate = false; //set true to eanble the calibration function

float drift_x = 0;
float drift_y = 0;
float drift_z = 0;

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
	//int8_t highest_vote_x = 0, highest_vote_y = 0;
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

			/* histogram voting */
			int vote_x =  match_x + 4;
			int vote_y =  match_y + 4;
			histogram_x[vote_x]++;
			histogram_y[vote_y]++;

			if(match_x == 0 && match_y == 0) {
				continue;
			}

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
	} else {
		/* calculate weighted average */
		int disp_px;
		for(disp_px = -4; disp_px <= 4; disp_px++) {
			predict_disp_x += disp_px * histogram_x[disp_px + 4];
			predict_disp_y += disp_px * histogram_y[disp_px + 4];
		}

		predict_disp_x /= (float)vote_count;
		predict_disp_y /= (float)vote_count;
	}

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

void flow_estimate_task(void)
{
	/* wait until the mcu peripherial initialization is finished */
	vTaskDelay(MILLI_SECOND_TICK(5));

	if(mpu9250_init()) {
		while(1); //This is bad
	}

	if(mt9v034_init()) {
		while(1); //This is bad
	}

	lidar_init(&lidar_distance);

	/* successfully initialized the hardware == */
	gpio_on(LED_3); //red led
	vTaskResume(fcb_link_task_handle);
	//vTaskResume(usb_link_task_handle); //XXX: remember to unblock me later
	/* ======================================== */

	if(do_gyro_calibrate == true) {
		mpu9250_drift_error_estimate(&drift_x, &drift_y, &drift_z);
	}

	timer_init();

	/* XXX: usb direct camera output */
	usb_fs_init();

	float current_time;
	float previous_time;
	float delta_t;
	float fps;

	int next = 0;

	float flow_vx, flow_vy;

	mt9v034_start_capture_image((uint32_t)flow.image[0].frame);
	mt9v034_wait_finish();
	previous_time = get_time_sec();

	mt9v034_start_capture_image((uint32_t)flow.image[1].frame);

	while(1) {
		//gpio_off(LED_1);

		mpu9250_read(&gyro_data);

		/* wait until image finished capturing */
		mt9v034_wait_finish();

		current_time = get_time_sec();
		delta_t = current_time - previous_time;
		previous_time = current_time;
		fps = 1.0f / delta_t;

		flow_estimate(
		        (uint16_t *)flow.image[0].frame,
		        (uint16_t *)flow.image[1].frame,
		        &flow_vx, &flow_vy, delta_t
		);
#if 1
		mt9v034_start_capture_image((uint32_t)flow.image[next].frame);
#else
		mt9v034_start_capture_image((uint32_t)flow.image[0].frame);
		//usb_send_onboard_info();
#endif
		next = (next + 1) % 2;

		send_flow_to_fcb(lidar_distance, flow_vx, flow_vy, current_time, delta_t, fps);

		//gpio_on(LED_1);
		gpio_toggle(LED_1);
	}
}
