#ifndef LOCATION_ESTIMATION_H_
#define LOCATION_ESTIMATION_H_

#include <stdint.h>

#include "stm32f4xx.h"                  // Device header
#define ARM_MATH_CM4 1    
#include "arm_math.h"


uint32_t intersection_points_of_circles( float32_t x1, float32_t y1, float32_t r1, float32_t x2, float32_t y2, float32_t r2, float32_t* ip1_x, float32_t* ip1_y, float32_t* ip2_x, float32_t* ip2_y);

float32_t trilateration_error(float32_t botx, float32_t boty, float32_t x1, float32_t y1, float32_t x2, float32_t y2, float32_t x3, float32_t y3, float32_t d1, float32_t d2, float32_t d3, float32_t q1, float32_t q2, float32_t q3);

float32_t multilateration_error(float32_t botx, float32_t boty, float32_t *beaconX, float32_t *beaconY, float32_t *distances, float32_t *qualities, uint32_t nr_of_beacons);

uint32_t get_robot_position_newton_bisection( float32_t* botx_ptr, float32_t* boty_ptr, //robot position
																																		float32_t* alt_botx_ptr, float32_t* alt_boty_ptr, //alternative robot position when only two beacons are available 	
																																		float32_t d1, float32_t d2, float32_t d3, // distances to Beacon 
																																		float32_t q1, float32_t q2, float32_t q3, //Signal Qualities
																																		float32_t *beaconX, float32_t * beaconY); //fixed beacon positions

uint32_t get_robot_position_newton_bisection_coop( float32_t* botx_ptr, float32_t* boty_ptr, //robot position
																																		float32_t* alt_botx_ptr, float32_t* alt_boty_ptr, //alternative robot position when only two beacons are available 	
																																		float32_t *distances, // distances to Beacon 
																																		float32_t *qualities, //Signal Qualities
																																		float32_t *beaconX, float32_t * beaconY, //fixed beacon positions
																																		uint32_t nr_of_beacons,
																																		float32_t last_botx, float32_t last_boty);

#endif

