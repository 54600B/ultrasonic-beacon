#include "location_estimation.h"

const float TRILATERATION_NETWON_ITERATION_IMPROVEMENT_TRESHOLD = 0.05; // stop the newton iteration if the improvement in the current iteration is less than = 50um 
const uint32_t MAXIMAL_NUMBER_OF_NEWTON_ITERATIONS = 20; 

/*----------------------------------------------------------------------------
intersection_points_of_circles: calculates the intersection points of two circles 
 *----------------------------------------------------------------------------*/
uint32_t intersection_points_of_circles( float32_t x1, float32_t y1, float32_t r1, float32_t x2, float32_t y2, float32_t r2, float32_t* ip1_x, float32_t* ip1_y, float32_t* ip2_x, float32_t* ip2_y)
{
	float32_t distance_between_centers_squared = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
	float32_t d;
	arm_sqrt_f32( distance_between_centers_squared, &d);
	
	float32_t d1 = ((r1*r1) + (d*d) - (r2*r2))/(2.0f*d); //distance from (x1, y1) to the intersection from line (x1,y1)->(x2,y2) and the line between the intersection points of the circles 
	float32_t h_squared = (r1*r1) - (d1*d1);
	if (h_squared >= 0.0f)
	{
		float32_t h;
		arm_sqrt_f32(h_squared, &h);
		
		float32_t xm = x1 + (x2-x1)*(d1/d);
		float32_t ym = y1 + (y2-y1)*(d1/d);
		
		float32_t xp = (y1-y2)*(h/d);
		float32_t yp = (x1-x2)*(h/d);
		
		*ip1_x = xm + xp;
		*ip1_y = ym - yp;
		*ip2_x = xm - xp;
		*ip2_y = ym + yp;
		
		return 2;
	}else{
		return 0;
	}
}


/*----------------------------------------------------------------------------
trilateration_error: returns the weighted trilateration error from robot position, beacon position, beacon distance and distance measurement quality
 *----------------------------------------------------------------------------*/
float32_t trilateration_error(float32_t botx, float32_t boty, float32_t x1, float32_t y1, float32_t x2, float32_t y2, float32_t x3, float32_t y3, float32_t d1, float32_t d2, float32_t d3, float32_t q1, float32_t q2, float32_t q3)
{
	float32_t act_d1_squared = (x1-botx)*(x1-botx) + (y1-boty)*(y1-boty);
	float32_t act_d1;
	arm_sqrt_f32(act_d1_squared, &act_d1);
	
	float32_t act_d2_squared = (x2-botx)*(x2-botx) + (y2-boty)*(y2-boty);
	float32_t act_d2;
	arm_sqrt_f32(act_d2_squared, &act_d2);
	
	float32_t act_d3_squared = (x3-botx)*(x3-botx) + (y3-boty)*(y3-boty);
	float32_t act_d3;
	arm_sqrt_f32(act_d3_squared, &act_d3);
	
	return fabs( fabs((act_d1-d1)*(act_d1-d1))*q1 + fabs((act_d2-d2)*(act_d2-d2))*q2 + fabs((act_d3-d3)*(act_d3-d3))*q3 );
}

/*----------------------------------------------------------------------------
trilateration_error: returns the weighted trilateration error from robot position, beacon position, beacon distance and distance measurement quality
 *----------------------------------------------------------------------------*/
float32_t multilateration_error(float32_t botx, float32_t boty, float32_t *beaconX, float32_t *beaconY, float32_t *distances, float32_t *qualities, uint32_t nr_of_beacons)
{
	float32_t error_sum = 0.0f;
	uint32_t beacon_id;
	for (beacon_id=0 ; beacon_id<nr_of_beacons ; beacon_id++)
	{
		float32_t bx = beaconX[beacon_id];
		float32_t by = beaconY[beacon_id];
		float32_t distance_squared = (bx-botx)*(bx-botx) + (by-boty)*(by-boty);
		float32_t distance;
		arm_sqrt_f32(distance_squared, &distance);
		
		float32_t distance_error = fabs( distance - distances[beacon_id] );
		error_sum += distance_error*distance_error*qualities[beacon_id];
	}
	return error_sum;
}

float32_t sign(float32_t value)
{
	if(value >= 0.0f){
		return 1.0f;
	}else{
		return -1.0f;
	}
}


uint32_t get_robot_position_newton_bisection_coop( float32_t* botx_ptr, float32_t* boty_ptr, //robot position
																																		float32_t* alt_botx_ptr, float32_t* alt_boty_ptr, //alternative robot position when only two beacons are available 	
																																		float32_t *distances, // distances to Beacon 
																																		float32_t *qualities, //Signal Qualities
																																		float32_t *beaconX, float32_t * beaconY, //fixed beacon positions
																																		uint32_t nr_of_beacons,
																																		float32_t last_botx, float32_t last_boty)
{
//ndd = 10; %Numerical Derivation Distance
//max_iterations = 10;
//max_bisection_steps = 8;
//max_step_distance = 500;
//	
//botx = 0;
//boty = 0;
	float32_t max_step_distance = 500.0;
	float32_t k=1.6;                           // Newton iterator step factor
	float32_t ndd = 10.0;	                 // Numerical Derivation Distance used to calculate the derivations
	uint32_t max_iterations = 10;         // maximum number of newton iterations
	uint32_t executed_newton_iterations = max_iterations;
	uint32_t max_bisection_steps = 8; // number of bisection steps in each newton iteration
	float32_t botx = 0.0;
	float32_t boty = 0.0;
	
	float32_t x1 = beaconX[0];
	float32_t y1 = beaconY[0]; 
	float32_t x2 = beaconX[1];
	float32_t y2 = beaconY[1];
	float32_t x3 = beaconX[2];
	float32_t y3 = beaconY[2];
	float32_t d1 = distances[0];
	float32_t d2 = distances[1];
	float32_t d3 = distances[2];
	float32_t q1 = qualities[0];
	float32_t q2 = qualities[1];
	float32_t q3 = qualities[2];

//% good starting point needed for eg d1=3400, d2=3200, d3=1200 

//p12 = intersection_points_of_circles(beaconX(1), beaconY(1), d1, ...
//																		 beaconX(2), beaconY(2), d2);
//p23 = intersection_points_of_circles(beaconX(2), beaconY(2), d2, ...
//																		 beaconX(3), beaconY(3), d3);
//p13 = intersection_points_of_circles(beaconX(1), beaconY(1), d1, ...
//																		 beaconX(3), beaconY(3), d3);

	float32_t intersection_points[12] = {0,0, 0,0, 0,0,    0,0, 0,0, 0,0,};
	uint32_t x_points12 = intersection_points_of_circles(x1, y1, d1, x2, y2, d2, &intersection_points[0], &intersection_points[1], &intersection_points[2], &intersection_points[3] );
	uint32_t x_points23 = intersection_points_of_circles(x2, y2, d2, x3, y3, d3, &intersection_points[4], &intersection_points[5], &intersection_points[6], &intersection_points[7] );
	uint32_t x_points13 = intersection_points_of_circles(x1, y1, d1, x3, y3, d3, &intersection_points[8], &intersection_points[9], &intersection_points[10], &intersection_points[11] );

	if ((x_points12==0)||(x_points23==0)||(x_points13==0))
	{
		float32_t px1 = 0.0f;
		float32_t py1 = 0.0f;
		float32_t px2 = 0.0f;
		float32_t py2 = 0.0f;
		if(x_points12>=1)
		{
			px1 = intersection_points[0];
			py1 = intersection_points[1];
			px2 = intersection_points[2];
			py2 = intersection_points[3]; 
		}
		if(x_points23>=1)
		{
			px1 = intersection_points[4];
			py1 = intersection_points[5];
			px2 = intersection_points[6];
			py2 = intersection_points[7]; 
		}
		if(x_points13>=1)
		{
			px1 = intersection_points[8];
			py1 = intersection_points[9];
			px2 = intersection_points[10];
			py2 = intersection_points[11]; 
		}
		if ((x_points12==0)&&(x_points23==0)&&(x_points13==0))
		{
			//<find two remaining fixed beacons>
			uint32_t fixed_beacon_id_1 = 0;
			uint32_t fixed_beacon_id_2 = 1;
			float dist1 = 0.0f;
			float dist2 = 0.0f;
			if ((q1<q2)&&(q1<q3))
			{
				fixed_beacon_id_1 = 1;
				dist1 = d2;
				fixed_beacon_id_2 = 2;
				dist2 = d3;
			}
			if ((q2<q1)&&(q2<q3))
			{
				fixed_beacon_id_1 = 0;
				dist1 = d1;
				fixed_beacon_id_2 = 2;
				dist2 = d3;
			}
			if ((q3<q1)&&(q3<q2))
			{
				fixed_beacon_id_1 = 0;
				dist1 = d1;
				fixed_beacon_id_2 = 1;
				dist1 = d3;
			}
			//<find two remaining fixed beacons>
			
			float dx = beaconX[fixed_beacon_id_2]-beaconX[fixed_beacon_id_1];
			float dy = beaconY[fixed_beacon_id_2]-beaconY[fixed_beacon_id_1];
			float distance_between_beacons;
			arm_sqrt_f32( dx*dx+dy*dy, &distance_between_beacons);
			float distance_to_beacon_1 = dist1+ (distance_between_beacons -(dist1+dist2))*0.5f;
			
			px1 = beaconX[fixed_beacon_id_1] + dx *(distance_to_beacon_1/distance_between_beacons);
			py1 = beaconY[fixed_beacon_id_1] + dy *(distance_to_beacon_1/distance_between_beacons);			
			px2 = px1;
			py2 = py1;
		}
		
		*botx_ptr = px1;
		*boty_ptr = py1;
		*alt_botx_ptr = px2;
		*alt_boty_ptr = py2;

		float32_t trilateration_error_1 = trilateration_error(px1, py1 ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		float32_t trilateration_error_2 = trilateration_error(px2, py2 ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);

		if (trilateration_error_1 < trilateration_error_2){
			botx = px1;
			boty = py1;
		}else{
			botx = px2;
			boty = py2;
		}

	}else{
	
//cross_points = [[p12(1);p12(2)],[p12(3);p12(4)],[p23(1);p23(2)],[p23(3);p23(4)],[p13(1);p13(2)],[p13(3);p13(4)]];                                 

//min_trilateration_error = trilateration_error(botx ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//for point = cross_points
//		%if (abs(point(1)) < 1000) && (abs(point(2)) < 1500)
//				intersect_trilat_error = trilateration_error(point(1) ,point(2),x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//				%point
//				%intersect_trilat_error
//				if min_trilateration_error > intersect_trilat_error
//						min_trilateration_error = intersect_trilat_error;
//						botx = point(1);
//						boty = point(2);
//				end
//		%end
//end
	
		float32_t min_trilateration_error = trilateration_error(botx ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		uint32_t intersect_point_nr;
		for (intersect_point_nr=0 ; intersect_point_nr<6 ; intersect_point_nr++)
		{
			float32_t x = intersection_points[intersect_point_nr*2];
			float32_t y = intersection_points[intersect_point_nr*2+1];
			float32_t intersect_trilat_error = trilateration_error(x ,y ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);

			if ((fabs(x) <= 1100)&&(fabs(x) < 1600))
			{
				if (min_trilateration_error > intersect_trilat_error)
				{
					min_trilateration_error = intersect_trilat_error;
					botx = x;
					boty = y;
				}
			}
		}
	}

	float32_t trilateration_error_temp = trilateration_error(botx ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
	float32_t trilateration_error_old_position = trilateration_error(last_botx ,last_boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);

	if (trilateration_error_old_position < trilateration_error_temp){
		botx = last_botx;
		boty = last_boty;
	}
	//printf("it botx=%i boty=%i\n", (int)botx, (int)boty);
//%botx
//%boty

//iteration_points = zeros(2,max_iterations+1);
//iteration_points(1,1) = botx;
//iteration_points(2,1) = boty;                                            

//iterations = 0;
//while iterations < max_iterations
//		iterations = iterations +1;
	


	uint32_t iterations = 0;
	while (iterations < max_iterations)
	{
		iterations = iterations +1;
	
//		err_x0  = trilateration_error(botx+ndd  ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//		err_x1  = trilateration_error(botx      ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//		err_x2  = trilateration_error(botx-ndd  ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);

//		err_dx_01 = err_x1 - err_x0; %1st derivation
//		err_dx_12 = err_x2 - err_x1;  
//		
//		err_dx = err_x2-err_x0;
//		err_dx2 = err_x2-2*err_x1+err_x0; % err_dx_12-err_dx_01; %2nd derivation   

//		x_step = (err_dx)/((k)*err_dx2);
//		
//		if (abs(err_dx2) >= 0.00001)
//				if abs(x_step) > max_step_distance
//						x_step = max_step_distance * sign(x_step);
//				end
//		end
		
		float32_t err_x0  = multilateration_error(botx+ndd, boty, beaconX, beaconY, distances, qualities, nr_of_beacons); //trilateration_error(botx+ndd  ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		float32_t err_x1  = multilateration_error(botx       , boty, beaconX, beaconY, distances, qualities, nr_of_beacons); //trilateration_error(botx         ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		float32_t err_x2  = multilateration_error(botx-ndd, boty, beaconX, beaconY, distances, qualities, nr_of_beacons); //trilateration_error(botx-ndd   ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);

		float32_t err_dx_01 = err_x1 - err_x0; //1st derivation
    float32_t err_dx_12 = err_x2 - err_x1;  
		
		float32_t err_dx = err_x2-err_x0;
		float32_t err_dx2 = err_x2-2*err_x1+err_x0;  //2nd derivation   

		float32_t x_step = (err_dx)/((k)*err_dx2);
    if (fabs(err_dx2) >= 0.00001){
			if (fabs(x_step) > max_step_distance){
				x_step = max_step_distance * sign(x_step);
			}
		}

//		err_y0  = trilateration_error(botx ,boty+ndd  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//		err_y1  = trilateration_error(botx ,boty      ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//		err_y2  = trilateration_error(botx ,boty-ndd  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);

//		err_dy_01 = err_y1 - err_y0; %1st derivation
//		err_dy_12 = err_y2 - err_y1;
//		
//		err_dy = err_y2-err_y0;
//		err_dy2 = err_y2-2*err_y1+err_y0; % err_dx_12-err_dx_01; %2nd derivation   

//		y_step = (err_dy)/((k)*err_dy2);
//		
//		if (abs(err_dy2) >= 0.00001)
//				if abs(y_step) > max_step_distance
//						y_step = max_step_distance * sign(y_step);
//				end
//		end

		float32_t err_y0  = multilateration_error(botx, boty+ndd, beaconX, beaconY, distances, qualities, nr_of_beacons); //trilateration_error(botx ,boty+ndd  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		float32_t err_y1  = multilateration_error(botx, boty       , beaconX, beaconY, distances, qualities, nr_of_beacons); //trilateration_error(botx ,boty      ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		float32_t err_y2  = multilateration_error(botx, boty-ndd, beaconX, beaconY, distances, qualities, nr_of_beacons); //trilateration_error(botx ,boty-ndd  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);

		float32_t err_dy_01 = err_y1 - err_y0; //1st derivation
		float32_t err_dy_12 = err_y2 - err_y1;
		float32_t err_dy = err_y2-err_y0;
		float32_t err_dy2 = err_y2-2*err_y1+err_y0; //2nd derivation   

		float32_t y_step = (err_dy)/((k)*err_dy2);
		if (fabs(err_dy2) >= 0.00001){
			if (fabs(y_step) > max_step_distance){
				y_step = max_step_distance * sign(y_step);
			}
		}


//		%<find minimum in trilateration error (zero in first deriv) by bisection> 
//		% one dimensional bisection in direction of the gradient
//		% start step size is determined by newton iteration step 
//	 
//		
//		%dx =   trilateration_error(botx-ndd ,boty  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) ...
//		%     - trilateration_error(botx+ndd ,boty  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//		%dy =   trilateration_error(botx ,boty-ndd  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) ...
//		%     - trilateration_error(botx ,boty+ndd  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//		
//		 
//		deriation_length = sqrt(err_dx^2 + err_dy^2);
//		ndd_x = (err_dx*ndd) / deriation_length;
//		ndd_y = (err_dy*ndd) / deriation_length;
//		
//		initial_step_length =  sqrt( x_step^2 + y_step^2 );
//		x_step = (err_dx*initial_step_length) / deriation_length;
//		y_step = (err_dy*initial_step_length) / deriation_length;

		float32_t derivation_length_squared = err_dx*err_dx + err_dy*err_dy;
		float32_t derivation_length;
		arm_sqrt_f32(derivation_length_squared, &derivation_length);
		float32_t ndd_x = (err_dx*ndd) / derivation_length;
		float32_t ndd_y = (err_dy*ndd) / derivation_length;

		float32_t initial_step_length_squared =  x_step*x_step + y_step*y_step;
		float32_t initial_step_length;
		arm_sqrt_f32(initial_step_length_squared, &initial_step_length);
		x_step = (err_dx*initial_step_length) / derivation_length;
		y_step = (err_dy*initial_step_length) / derivation_length;


//		slope_1st = trilateration_error(botx-ndd_x ,boty-ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) ...
//							- trilateration_error(botx+ndd_x ,boty+ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//		
//		slope = slope_1st;
//		
//		bisection_step = 1;
//		
//		nbotx = botx;
//		nboty = boty;

		float32_t slope_1st =  multilateration_error(botx-ndd_x, boty-ndd_y, beaconX, beaconY, distances, qualities, nr_of_beacons) //trilateration_error(botx-ndd_x ,boty-ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) 
																	- multilateration_error(botx+ndd_x, boty+ndd_y, beaconX, beaconY, distances, qualities, nr_of_beacons);// trilateration_error(botx+ndd_x ,boty+ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		float32_t slope = slope_1st;
		uint32_t bisection_step = 1;
		float32_t nbotx = botx;
		float32_t nboty = boty;

//		%add [x_step, y_step] to current location until the
//		%sign of derivation changes    
//		while (sign(slope) == sign(slope_1st)) && (bisection_step <= max_bisection_steps)  
//				nbotx = nbotx+x_step;
//				nboty = nboty+y_step;
//				
//				slope = trilateration_error(nbotx-ndd_x ,nboty-ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) ...
//							- trilateration_error(nbotx+ndd_x ,nboty+ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);  
//				
//				bisection_step = bisection_step+1;
//		end

		while ((sign(slope) == sign(slope_1st)) && (bisection_step <= max_bisection_steps))
		{			
			nbotx = nbotx+x_step;
			nboty = nboty+y_step;
		
			slope = //trilateration_error(nbotx-ndd_x ,nboty-ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) 
								 //- trilateration_error(nbotx+ndd_x ,nboty+ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);  
																		multilateration_error(nbotx-ndd_x, nboty-ndd_y, beaconX, beaconY, distances, qualities, nr_of_beacons) //trilateration_error(botx-ndd_x ,boty-ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) 
																	- multilateration_error(nbotx+ndd_x, nboty+ndd_y, beaconX, beaconY, distances, qualities, nr_of_beacons);// trilateration_error(botx+ndd_x ,boty+ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
			bisection_step = bisection_step+1;
		}

//		
//		%sign(slope @ [nbotx, nboty]) != sign(slope @ [nbotx-x_step, nboty-x_step])
//		nbotx_1 = nbotx;
//		nboty_1 = nboty;
//		sign_1 = sign(slope);
//		
//		nbotx_2 = nbotx-x_step;
//		nboty_2 = nboty-y_step;
//		sign_2 = sign(slope_1st);
//		
//		mid_x = nbotx_2;
//		mid_y = nboty_2;
		
		float32_t nbotx_1 = nbotx;
		float32_t nboty_1 = nboty;
		float32_t sign_1 = sign(slope);

		float32_t nbotx_2 = nbotx-x_step;
		float32_t nboty_2 = nboty-y_step;
		float32_t sign_2 = sign(slope_1st);
		
		float32_t mid_x = nbotx_2;
		float32_t mid_y = nboty_2;
		

//		while (bisection_step <= max_bisection_steps)
//				
//				mid_x = (nbotx_1 + nbotx_2) /2.0;
//				mid_y = (nboty_1 + nboty_2) /2.0;
//				
//				slope = trilateration_error(mid_x-ndd_x ,mid_y-ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) ...
//							- trilateration_error(mid_x+ndd_x ,mid_y+ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);  
//			 
//				sign_mid = sign(slope);
//				
//				if sign_mid == sign_1
//						nbotx_1 = mid_x;
//						nboty_1 = mid_y;
//						sign_1 = sign_mid;
//				else
//						nbotx_2 = mid_x;
//						nboty_2 = mid_y;
//						sign_2 = sign_mid;
//				end
//				
//				bisection_step = bisection_step+1;
//		end

		while (bisection_step <= max_bisection_steps){
			mid_x = (nbotx_1 + nbotx_2) /2.0f;
			mid_y = (nboty_1 + nboty_2) /2.0f;

			slope = //trilateration_error(mid_x-ndd_x ,mid_y-ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) 
								//- trilateration_error(mid_x+ndd_x ,mid_y+ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);  
			
																		multilateration_error(mid_x-ndd_x ,mid_y-ndd_y, beaconX, beaconY, distances, qualities, nr_of_beacons) 
																	- multilateration_error(mid_x+ndd_x ,mid_y+ndd_y, beaconX, beaconY, distances, qualities, nr_of_beacons);
			
			float32_t sign_mid = sign(slope);
			if (sign_mid == sign_1){
				nbotx_1 = mid_x;
				nboty_1 = mid_y;
				sign_1 = sign_mid;
			}else{
				nbotx_2 = mid_x;
				nboty_2 = mid_y;
				sign_2 = sign_mid;
			}
			
		bisection_step = bisection_step+1;
		}


		float32_t new_botx = mid_x;
		float32_t new_boty = mid_y;
//		%</find minimum in trilateration error (zero in first deriv) by bisection>

		float32_t jump_squared = (botx-new_botx)*(botx-new_botx) + (boty-new_boty)*(boty-new_boty);
		if (jump_squared < (0.1*0.1)){
			iterations = max_iterations;
			executed_newton_iterations = iterations;
		} 
		botx = new_botx;
		boty = new_boty;		
	}  //Newton iteration loop
	*botx_ptr = botx;
	*boty_ptr = boty;	
	*alt_botx_ptr = botx;
	*alt_boty_ptr = boty;
	return  executed_newton_iterations;
}


/*----------------------------------------------------------------------------
get_robot_position_newton_bisection: estimates the robot position using 2d newton ierations with a bisection setp adjusting the step size 
minimizing the error defined by the function trilateration_error
returns the nr of needed iterations
 *----------------------------------------------------------------------------*/
uint32_t get_robot_position_newton_bisection( float32_t* botx_ptr, float32_t* boty_ptr, //robot position
																																		float32_t* alt_botx_ptr, float32_t* alt_boty_ptr, //alternative robot position when only two beacons are available 	
																																		float32_t d1, float32_t d2, float32_t d3, // distances to Beacon 
																																		float32_t q1, float32_t q2, float32_t q3, //Signal Qualities
																																		float32_t *beaconX, float32_t * beaconY) //fixed beacon positions
{
//ndd = 10; %Numerical Derivation Distance
//max_iterations = 10;
//max_bisection_steps = 8;
//max_step_distance = 500;
//	
//botx = 0;
//boty = 0;
	float32_t max_step_distance = 500.0;
	float32_t k=1.6;                           // Newton iterator step factor
	float32_t ndd = 10.0;	                 // Numerical Derivation Distance used to calculate the derivations
	uint32_t max_iterations = 10;         // maximum number of newton iterations
	uint32_t executed_newton_iterations = max_iterations;
	uint32_t max_bisection_steps = 8; // number of bisection steps in each newton iteration
	float32_t botx = 0.0;
	float32_t boty = 0.0;
	
	float32_t x1 = beaconX[0];
	float32_t y1 = beaconY[0]; 
	float32_t x2 = beaconX[1];
	float32_t y2 = beaconY[1];
	float32_t x3 = beaconX[2];
	float32_t y3 = beaconY[2];

//% good starting point needed for eg d1=3400, d2=3200, d3=1200 

//p12 = intersection_points_of_circles(beaconX(1), beaconY(1), d1, ...
//																		 beaconX(2), beaconY(2), d2);
//p23 = intersection_points_of_circles(beaconX(2), beaconY(2), d2, ...
//																		 beaconX(3), beaconY(3), d3);
//p13 = intersection_points_of_circles(beaconX(1), beaconY(1), d1, ...
//																		 beaconX(3), beaconY(3), d3);

	float32_t intersection_points[12] = {0,0, 0,0, 0,0,    0,0, 0,0, 0,0,};
	uint32_t x_points12 = intersection_points_of_circles(x1, y1, d1, x2, y2, d2, &intersection_points[0], &intersection_points[1], &intersection_points[2], &intersection_points[3] );
	uint32_t x_points23 = intersection_points_of_circles(x2, y2, d2, x3, y3, d3, &intersection_points[4], &intersection_points[5], &intersection_points[6], &intersection_points[7] );
	uint32_t x_points13 = intersection_points_of_circles(x1, y1, d1, x3, y3, d3, &intersection_points[8], &intersection_points[9], &intersection_points[10], &intersection_points[11] );

	if ((x_points12==0)||(x_points23==0)||(x_points13==0))
	{
		float32_t px1 = 0.0f;
		float32_t py1 = 0.0f;
		float32_t px2 = 0.0f;
		float32_t py2 = 0.0f;
		if(x_points12>=1)
		{
			px1 = intersection_points[0];
			py1 = intersection_points[1];
			px2 = intersection_points[2];
			py2 = intersection_points[3]; 
		}
		if(x_points23>=1)
		{
			px1 = intersection_points[4];
			py1 = intersection_points[5];
			px2 = intersection_points[6];
			py2 = intersection_points[7]; 
		}
		if(x_points13>=1)
		{
			px1 = intersection_points[8];
			py1 = intersection_points[9];
			px2 = intersection_points[10];
			py2 = intersection_points[11]; 
		}
		if ((x_points12==0)&&(x_points23==0)&&(x_points13==0))
		{
			//<find two remaining fixed beacons>
			uint32_t fixed_beacon_id_1 = 0;
			uint32_t fixed_beacon_id_2 = 1;
			float dist1 = 0.0f;
			float dist2 = 0.0f;
			if ((q1<q2)&&(q1<q3))
			{
				fixed_beacon_id_1 = 1;
				dist1 = d2;
				fixed_beacon_id_2 = 2;
				dist2 = d3;
			}
			if ((q2<q1)&&(q2<q3))
			{
				fixed_beacon_id_1 = 0;
				dist1 = d1;
				fixed_beacon_id_2 = 2;
				dist2 = d3;
			}
			if ((q3<q1)&&(q3<q2))
			{
				fixed_beacon_id_1 = 0;
				dist1 = d1;
				fixed_beacon_id_2 = 1;
				dist1 = d3;
			}
			//<find two remaining fixed beacons>
			
			float dx = beaconX[fixed_beacon_id_2]-beaconX[fixed_beacon_id_1];
			float dy = beaconY[fixed_beacon_id_2]-beaconY[fixed_beacon_id_1];
			float distance_between_beacons;
			arm_sqrt_f32( dx*dx+dy*dy, &distance_between_beacons);
			float distance_to_beacon_1 = dist1+ (distance_between_beacons -(dist1+dist2))*0.5f;
			
			px1 = beaconX[fixed_beacon_id_1] + dx *(distance_to_beacon_1/distance_between_beacons);
			py1 = beaconY[fixed_beacon_id_1] + dy *(distance_to_beacon_1/distance_between_beacons);			
			px2 = px1;
			py2 = py1;
		}
		
		*botx_ptr = px1;
		*boty_ptr = py1;
		*alt_botx_ptr = px2;
		*alt_boty_ptr = py2;
		return 0;
	}
	
//cross_points = [[p12(1);p12(2)],[p12(3);p12(4)],[p23(1);p23(2)],[p23(3);p23(4)],[p13(1);p13(2)],[p13(3);p13(4)]];                                 

//min_trilateration_error = trilateration_error(botx ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//for point = cross_points
//		%if (abs(point(1)) < 1000) && (abs(point(2)) < 1500)
//				intersect_trilat_error = trilateration_error(point(1) ,point(2),x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//				%point
//				%intersect_trilat_error
//				if min_trilateration_error > intersect_trilat_error
//						min_trilateration_error = intersect_trilat_error;
//						botx = point(1);
//						boty = point(2);
//				end
//		%end
//end
	
	float32_t min_trilateration_error = trilateration_error(botx ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
  uint32_t intersect_point_nr;
	for (intersect_point_nr=0 ; intersect_point_nr<6 ; intersect_point_nr++)
	{
		float32_t x = intersection_points[intersect_point_nr*2];
		float32_t y = intersection_points[intersect_point_nr*2+1];
		float32_t intersect_trilat_error = trilateration_error(x ,y ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		
		if (min_trilateration_error > intersect_trilat_error)
		{
			min_trilateration_error = intersect_trilat_error;
			botx = x;
			boty = y;
		}
	}

//%botx
//%boty

//iteration_points = zeros(2,max_iterations+1);
//iteration_points(1,1) = botx;
//iteration_points(2,1) = boty;                                            

//iterations = 0;
//while iterations < max_iterations
//		iterations = iterations +1;
	
	uint32_t iterations = 0;
	while (iterations < max_iterations)
	{
		iterations = iterations +1;
	
//		err_x0  = trilateration_error(botx+ndd  ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//		err_x1  = trilateration_error(botx      ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//		err_x2  = trilateration_error(botx-ndd  ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);

//		err_dx_01 = err_x1 - err_x0; %1st derivation
//		err_dx_12 = err_x2 - err_x1;  
//		
//		err_dx = err_x2-err_x0;
//		err_dx2 = err_x2-2*err_x1+err_x0; % err_dx_12-err_dx_01; %2nd derivation   

//		x_step = (err_dx)/((k)*err_dx2);
//		
//		if (abs(err_dx2) >= 0.00001)
//				if abs(x_step) > max_step_distance
//						x_step = max_step_distance * sign(x_step);
//				end
//		end
		
		float32_t err_x0  = trilateration_error(botx+ndd  ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		float32_t err_x1  = trilateration_error(botx         ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		float32_t err_x2  = trilateration_error(botx-ndd   ,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);

		float32_t err_dx_01 = err_x1 - err_x0; //1st derivation
    float32_t err_dx_12 = err_x2 - err_x1;  
		
		float32_t err_dx = err_x2-err_x0;
		float32_t err_dx2 = err_x2-2*err_x1+err_x0;  //2nd derivation   

		float32_t x_step = (err_dx)/((k)*err_dx2);
    if (fabs(err_dx2) >= 0.00001){
			if (fabs(x_step) > max_step_distance){
				x_step = max_step_distance * sign(x_step);
			}
		}

//		err_y0  = trilateration_error(botx ,boty+ndd  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//		err_y1  = trilateration_error(botx ,boty      ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//		err_y2  = trilateration_error(botx ,boty-ndd  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);

//		err_dy_01 = err_y1 - err_y0; %1st derivation
//		err_dy_12 = err_y2 - err_y1;
//		
//		err_dy = err_y2-err_y0;
//		err_dy2 = err_y2-2*err_y1+err_y0; % err_dx_12-err_dx_01; %2nd derivation   

//		y_step = (err_dy)/((k)*err_dy2);
//		
//		if (abs(err_dy2) >= 0.00001)
//				if abs(y_step) > max_step_distance
//						y_step = max_step_distance * sign(y_step);
//				end
//		end

		float32_t err_y0  = trilateration_error(botx ,boty+ndd  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		float32_t err_y1  = trilateration_error(botx ,boty      ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		float32_t err_y2  = trilateration_error(botx ,boty-ndd  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);

		float32_t err_dy_01 = err_y1 - err_y0; //1st derivation
		float32_t err_dy_12 = err_y2 - err_y1;
		float32_t err_dy = err_y2-err_y0;
		float32_t err_dy2 = err_y2-2*err_y1+err_y0; //2nd derivation   

		float32_t y_step = (err_dy)/((k)*err_dy2);
		if (fabs(err_dy2) >= 0.00001){
			if (fabs(y_step) > max_step_distance){
				y_step = max_step_distance * sign(y_step);
			}
		}


//		%<find minimum in trilateration error (zero in first deriv) by bisection> 
//		% one dimensional bisection in direction of the gradient
//		% start step size is determined by newton iteration step 
//	 
//		
//		%dx =   trilateration_error(botx-ndd ,boty  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) ...
//		%     - trilateration_error(botx+ndd ,boty  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//		%dy =   trilateration_error(botx ,boty-ndd  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) ...
//		%     - trilateration_error(botx ,boty+ndd  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//		
//		 
//		deriation_length = sqrt(err_dx^2 + err_dy^2);
//		ndd_x = (err_dx*ndd) / deriation_length;
//		ndd_y = (err_dy*ndd) / deriation_length;
//		
//		initial_step_length =  sqrt( x_step^2 + y_step^2 );
//		x_step = (err_dx*initial_step_length) / deriation_length;
//		y_step = (err_dy*initial_step_length) / deriation_length;

		float32_t derivation_length_squared = err_dx*err_dx + err_dy*err_dy;
		float32_t derivation_length;
		arm_sqrt_f32(derivation_length_squared, &derivation_length);
		float32_t ndd_x = (err_dx*ndd) / derivation_length;
		float32_t ndd_y = (err_dy*ndd) / derivation_length;

		float32_t initial_step_length_squared =  x_step*x_step + y_step*y_step;
		float32_t initial_step_length;
		arm_sqrt_f32(initial_step_length_squared, &initial_step_length);
		x_step = (err_dx*initial_step_length) / derivation_length;
		y_step = (err_dy*initial_step_length) / derivation_length;


//		slope_1st = trilateration_error(botx-ndd_x ,boty-ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) ...
//							- trilateration_error(botx+ndd_x ,boty+ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
//		
//		slope = slope_1st;
//		
//		bisection_step = 1;
//		
//		nbotx = botx;
//		nboty = boty;

		float32_t slope_1st =  trilateration_error(botx-ndd_x ,boty-ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) 
																	- trilateration_error(botx+ndd_x ,boty+ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		float32_t slope = slope_1st;
		uint32_t bisection_step = 1;
		float32_t nbotx = botx;
		float32_t nboty = boty;

//		%add [x_step, y_step] to current location until the
//		%sign of derivation changes    
//		while (sign(slope) == sign(slope_1st)) && (bisection_step <= max_bisection_steps)  
//				nbotx = nbotx+x_step;
//				nboty = nboty+y_step;
//				
//				slope = trilateration_error(nbotx-ndd_x ,nboty-ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) ...
//							- trilateration_error(nbotx+ndd_x ,nboty+ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);  
//				
//				bisection_step = bisection_step+1;
//		end

		while ((sign(slope) == sign(slope_1st)) && (bisection_step <= max_bisection_steps))
		{			
			nbotx = nbotx+x_step;
			nboty = nboty+y_step;
		
			slope = trilateration_error(nbotx-ndd_x ,nboty-ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) 
								- trilateration_error(nbotx+ndd_x ,nboty+ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);  
			bisection_step = bisection_step+1;
		}

//		
//		%sign(slope @ [nbotx, nboty]) != sign(slope @ [nbotx-x_step, nboty-x_step])
//		nbotx_1 = nbotx;
//		nboty_1 = nboty;
//		sign_1 = sign(slope);
//		
//		nbotx_2 = nbotx-x_step;
//		nboty_2 = nboty-y_step;
//		sign_2 = sign(slope_1st);
//		
//		mid_x = nbotx_2;
//		mid_y = nboty_2;
		
		float32_t nbotx_1 = nbotx;
		float32_t nboty_1 = nboty;
		float32_t sign_1 = sign(slope);

		float32_t nbotx_2 = nbotx-x_step;
		float32_t nboty_2 = nboty-y_step;
		float32_t sign_2 = sign(slope_1st);
		
		float32_t mid_x = nbotx_2;
		float32_t mid_y = nboty_2;
		

//		while (bisection_step <= max_bisection_steps)
//				
//				mid_x = (nbotx_1 + nbotx_2) /2.0;
//				mid_y = (nboty_1 + nboty_2) /2.0;
//				
//				slope = trilateration_error(mid_x-ndd_x ,mid_y-ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) ...
//							- trilateration_error(mid_x+ndd_x ,mid_y+ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);  
//			 
//				sign_mid = sign(slope);
//				
//				if sign_mid == sign_1
//						nbotx_1 = mid_x;
//						nboty_1 = mid_y;
//						sign_1 = sign_mid;
//				else
//						nbotx_2 = mid_x;
//						nboty_2 = mid_y;
//						sign_2 = sign_mid;
//				end
//				
//				bisection_step = bisection_step+1;
//		end

		while (bisection_step <= max_bisection_steps){
			mid_x = (nbotx_1 + nbotx_2) /2.0f;
			mid_y = (nboty_1 + nboty_2) /2.0f;

			slope = trilateration_error(mid_x-ndd_x ,mid_y-ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3) 
								- trilateration_error(mid_x+ndd_x ,mid_y+ndd_y  ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);  
			
			float32_t sign_mid = sign(slope);
			if (sign_mid == sign_1){
				nbotx_1 = mid_x;
				nboty_1 = mid_y;
				sign_1 = sign_mid;
			}else{
				nbotx_2 = mid_x;
				nboty_2 = mid_y;
				sign_2 = sign_mid;
			}
			
		bisection_step = bisection_step+1;
		}


		float32_t new_botx = mid_x;
		float32_t new_boty = mid_y;
//		%</find minimum in trilateration error (zero in first deriv) by bisection>

		float32_t jump_squared = (botx-new_botx)*(botx-new_botx) + (boty-new_boty)*(boty-new_boty);
		if (jump_squared < (0.1*0.1)){
			iterations = max_iterations;
			executed_newton_iterations = iterations;
		} 
		botx = new_botx;
		boty = new_boty;		
	}  //Newton iteration loop
	*botx_ptr = botx;
	*boty_ptr = boty;	
	*alt_botx_ptr = botx;
	*alt_boty_ptr = boty;
	return  executed_newton_iterations;
}
	
/*----------------------------------------------------------------------------
get_robot_position: estimates the robot position using 2d newton ierations minimizing the error defined by the function trilateration_error
returns the nr of needed iterations
 *----------------------------------------------------------------------------*/
//                                               robot position    , Master Beacon , Slave1 Beacon , Slave2 Beacon ,  distances to Beacon     ,   Signal Qualities
uint32_t get_robot_position( float* botx, float* boty, float x1, float y1, float x2, float y2, float x3, float y3, float d1, float d2, float d3, float q1, float q2, float q3)
{
	uint32_t iterations = 0;
	uint32_t max_iterations = MAXIMAL_NUMBER_OF_NEWTON_ITERATIONS;
	
	while (iterations < max_iterations)
	{
		iterations = iterations +1;
		
		float err_x0  = trilateration_error((*botx)-1    ,*boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		float err_x1  = trilateration_error((*botx)       ,*boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		float err_x2  = trilateration_error((*botx)+1   ,*boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		
		float err_dx_01 = err_x1 - err_x0; //1st derivation
		float err_dx_12 = err_x2 - err_x1;           
		float err_dx2   = err_dx_12-err_dx_01;//2nd derivation   
		
		float new_botx  = (*botx);
		if (err_dx2 >= 0.1f)
		{
			new_botx = new_botx - ((err_dx_01+err_dx_12)/(4.0f*err_dx2));
		}
		
		float err_y0  = trilateration_error(*botx ,(*boty)-1     ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		float err_y1  = trilateration_error(*botx ,(*boty)         ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		float err_y2  = trilateration_error(*botx ,(*boty)+1     ,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3);
		
		float err_dy_01 = err_y1 - err_y0; //1st derivation
		float err_dy_12 = err_y2 - err_y1;       
		float err_dy2   = err_dy_12-err_dy_01; //2nd derivation 

		float new_boty  = *boty;
		if (err_dy2 >= 0.1f)
		{
			new_boty = new_boty - (err_dy_01+err_dy_12)/(4.0f*err_dy2);
		}
		
		/*
		printf("iteration %i\n", iterations);
		
		printf("err_x0=%f\n", err_x0);
		printf("err_x1=%f\n", err_x1);
		printf("err_x2=%f\n", err_x2);
		
		printf("err_dx_01=%f\n", err_dx_01);
		printf("err_dx_12=%f\n", err_dx_12);
		
		printf("new_botx=%f\n", new_botx);
		printf("new_boty=%f\n", new_boty);
		*/
		
		if (sqrt( (*botx-new_botx)*(*botx-new_botx) + (*boty-new_boty)*(*boty-new_boty) ) < TRILATERATION_NETWON_ITERATION_IMPROVEMENT_TRESHOLD)
		{
			iterations = max_iterations;
		}
		*botx = new_botx;
		*boty = new_boty;
	}
	return iterations;
}

