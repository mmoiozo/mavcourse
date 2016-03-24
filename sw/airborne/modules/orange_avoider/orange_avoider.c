/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#include "modules/orange_avoider/orange_avoider.h"

/**
* Charles 
*/
//#include "modules/computer_vision/colorfilter.h"

#include "modules/ext_colorfilter/ext_colorfilter.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include <time.h>
#include <stdlib.h>

uint8_t safeToGoForwards=FALSE;

//int tresholdColorCount = 1000; //was 200
/**
* Charles 
*/
// Set all tresholds!!!
float tresholdOrange_lcnt = 0.5;
float tresholdOrange_clcnt = 0.5;
float tresholdOrange_crcnt = 0.5;
float tresholdOrange_rcnt = 0.5;
float tresholdavg_lcnt = 0.4;
float tresholdavg_clcnt = 0.4;
float tresholdavg_crcnt = 0.4;
float tresholdavg_rcnt = 0.4;
int32_t incrementForAvoidance;
uint8_t stopGoingHigher=FALSE;
float maxHeight = 6;

void orange_avoider_init() {
	// Initialise the variables of the colorfilter to accept orange
	color_lum_min=0;
	color_lum_max=131;
	color_cb_min=93;
	color_cb_max=255;
	color_cr_min=134;
	color_cr_max=255;
	// Initialise random values
	srand(time(NULL));
	chooseRandomIncrementAvoidance();
}
void orange_avoider_periodic() {
	// Check the amount of orange. If this is above a threshold
	// you want to turn a certain amount of degrees
	safeToGoForwards = (
		(  (lcnt < tresholdOrange_lcnt)    // left sector orange count
		&&(clcnt < tresholdOrange_clcnt)  // left centre sector orange count
		&&(crcnt < tresholdOrange_crcnt)  // right centre sector orange count
		&&(rcnt < tresholdOrange_rcnt) // right sector orange count
		) &&
		(
		(avg_lcnt < tresholdavg_lcnt)
		&&(avg_clcnt < tresholdavg_clcnt)
		&&(avg_crcnt < tresholdavg_crcnt)
		&&(avg_rcnt < tresholdavg_rcnt)
		)		
	 );  
	printf("Save to go:%d \n avgleft= %3f, avgleftc= %3f, avgrightc= %3f, avgright= %3f \n oraleft %3f, oraleftc %3f, orarightc, %3f, oraright %3f \n", safeToGoForwards, avg_lcnt,avg_clcnt,avg_crcnt,avg_rcnt, lcnt, clcnt, crcnt, rcnt);

	checkHeight(maxHeight);
}


/**
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, int32_t increment)
{
  *heading = *heading + increment;
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(*heading); // HEADING HAS INT32_ANGLE_FRAC....
  return FALSE;
}




uint8_t change_waypoint_random_inside_obstacle(uint8_t waypoint)
{
	struct EnuCoor_i new_coor;
	struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position
	float maxy = 51.990657;
	float miny = 51.990601;
	float minx = 4.376748;
	float maxx = 4.376845;
	
	srand(time(NULL));
	
	
	// Get random number
 	float ry = (float)rand()*(maxy-miny)+miny;
	float rx = (float)rand()*(maxx-minx)+minx;

	// determine the random place of the waypoint inside the obstacle zone
	
	new_coor.x = rx;
	new_coor.y = ry;
	new_coor.z = pos->z; // Keep the height the same	

	// Set the waypoint to the calculated position
	waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);
	
  return FALSE;
}

uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters){
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	  // Calculate the sine and cosine of the heading the drone is keeping
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	  return FALSE;
}



uint8_t chooseRandomIncrementAvoidance(){

	int r = rand() % 4;
	if(r==0){
		incrementForAvoidance=1000;
	}
	else if(r==1){
		incrementForAvoidance=2000;
	}
	else if(r==2){
		incrementForAvoidance=-2000; //was 350
	}
	else{
		incrementForAvoidance=-1000; // was -350
	}
	return FALSE;
}

float checkHeight(float maxHeight){

	  //struct UtmCoor_f *pos = stateGetPositionUtm_f(); // Get your current position
	  struct EnuCoor_f *pos3 = stateGetPositionEnu_f(); //
	  //struct EcefCoor_f *pos2 = stateGetPositionEcef_f();

	  //float altitude = pos->alt;
	  //float altitude2 = pos2->y;
	  float altitude3 = pos3->z;

	  	  printf("Checking what the z coordinate is %f maxHeight: %f \n", altitude3, maxHeight);
	  	  if(altitude3 > maxHeight){
	  	  		stopGoingHigher=TRUE;
	  	  	  }
	  	  	  else{
	  	  	  	stopGoingHigher=FALSE;
	  	            }

	  	printf("Is maxheight?  %d \n", stopGoingHigher);
	return FALSE;
}

