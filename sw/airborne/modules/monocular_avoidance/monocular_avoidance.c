/*
 * Copyright (C) Michaël Ozo
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/monocular_avoidance/monocular_avoidance.c"
 * @author Michaël Ozo
 * Obstacle avoidance unsing bebob frontfacing camera imu and optitrack data.
 */

// Own header
#include "modules/computer_vision/cv.h"
#include "modules/monocular_avoidance/monocular_avoidance.h"
#include "subsystems/datalink/telemetry.h"
#include <stdio.h>
#include <std.h>

// Computer Vision
#include "lib/vision/image.h"
#include "lib/vision/lucas_kanade.h"
#include "lib/vision/fast_rosten.h"

//state 
#include "state.h"
#include "subsystems/imu.h"

// Calculations
#include <math.h>
#include "subsystems/ins/ins_int.h" // used for ins.sonar_z
#include "boards/ardrone/navdata.h" // for ultrasound Height

//coordinates
#include "math/pprz_geodetic_double.h"
#include "math/pprz_algebra_double.h"
#include "subsystems/gps/gps_datalink.h"


#ifndef OPTICFLOW_FAST9_THRESHOLD
#define OPTICFLOW_FAST9_THRESHOLD 30//10//20//5
#endif

#ifndef OPTICFLOW_FAST9_MIN_DISTANCE
#define OPTICFLOW_FAST9_MIN_DISTANCE 10
#endif

#ifndef OPTICFLOW_FAST9_ADAPTIVE
#define OPTICFLOW_FAST9_ADAPTIVE FALSE
#endif

#ifndef OPTICFLOW_MAX_TRACK_CORNERS
#define OPTICFLOW_MAX_TRACK_CORNERS 25//50
#endif

#ifndef OPTICFLOW_WINDOW_SIZE
#define OPTICFLOW_WINDOW_SIZE 10//20
#endif

#ifndef OPTICFLOW_MAX_ITERATIONS
#define OPTICFLOW_MAX_ITERATIONS 10//20
#endif

#ifndef OPTICFLOW_THRESHOLD_VEC
#define OPTICFLOW_THRESHOLD_VEC 2
#endif

#ifndef OPTICFLOW_SUBPIXEL_FACTOR
#define OPTICFLOW_SUBPIXEL_FACTOR 10
#endif

#ifndef OPTICFLOW_WINDOW_SIZE
#define OPTICFLOW_WINDOW_SIZE 10
#endif

struct opticflow_result_t result;
struct opticflow_t opticflow;


// Filter Settings
uint8_t color_lum_min = 5;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 5;
uint8_t color_cb_max  = 200;
uint8_t color_cr_min  = 5;
uint8_t color_cr_max  = 230;

int color_count = 0;

//Attitude
int32_t phi_temp = 0;
int32_t theta_temp = 0;
int32_t psi_temp = 0;

//Position
int32_t x_temp = 0;
int32_t y_temp = 0;
int32_t z_temp = 0;


bool_t process_frame(struct image_t* img);
bool_t process_frame(struct image_t* img)
{
  
  
  // Convert image to grayscale
  image_to_grayscale(img, &opticflow.img_gray);

  // Copy to previous image if not set
  if (!opticflow.got_first_img) {
    image_copy(&opticflow.img_gray, &opticflow.prev_img_gray);
    opticflow.got_first_img = TRUE;
  }
  
  // *************************************************************************************
  // Corner detection
  // *************************************************************************************

  // FAST corner detection (TODO: non fixed threshold)
  struct point_t *corners = fast9_detect(img, OPTICFLOW_FAST9_THRESHOLD, OPTICFLOW_FAST9_MIN_DISTANCE,
                                         20, 20, &result.corner_cnt);
  
  // Adaptive threshold
  if (opticflow.fast9_adaptive) {

    // Decrease and increase the threshold based on previous values
    if (result.corner_cnt < 40 && opticflow.fast9_threshold > 5) {
      opticflow.fast9_threshold--;
    } else if (result.corner_cnt > 50 && opticflow.fast9_threshold < 60) {
      opticflow.fast9_threshold++;
    }
  }
  
  image_show_points(img, corners, result.corner_cnt);
  
  int32_t debug = result.corner_cnt;
  int32_t debug_tr = opticflow.fast9_threshold;
  phi_temp = ANGLE_BFP_OF_REAL(stateGetNedToBodyEulers_f()->phi);
  theta_temp = ANGLE_BFP_OF_REAL(stateGetNedToBodyEulers_f()->theta);
  psi_temp = ANGLE_BFP_OF_REAL(stateGetNedToBodyEulers_f()->psi);
  x_temp = POS_BFP_OF_REAL(stateGetPositionEnu_f()->x);
  y_temp = POS_BFP_OF_REAL(stateGetPositionEnu_f()->y);
  z_temp = POS_BFP_OF_REAL(stateGetPositionEnu_f()->z);
 
 //int32_t debug = img->h;
  
  DOWNLINK_SEND_MONOCULAR_AVOIDANCE(DefaultChannel, DefaultDevice, &debug, &debug_tr, &phi_temp, &theta_temp, &psi_temp, &x_temp, &y_temp, &z_temp);
  
  
  // Check if we found some corners to track
  if (result.corner_cnt < 1) {
    free(corners);
    image_copy(&opticflow.img_gray, &opticflow.prev_img_gray);
    return FALSE;
  }
  
  // *************************************************************************************
  // Corner Tracking
  // *************************************************************************************

  // Execute a Lucas Kanade optical flow
  result.tracked_cnt = result.corner_cnt;
  struct flow_t *vectors = opticFlowLK(&opticflow.img_gray, &opticflow.prev_img_gray, corners, &result.tracked_cnt,
                                       opticflow.window_size / 2, opticflow.subpixel_factor, opticflow.max_iterations,
                                       opticflow.threshold_vec, opticflow.max_track_corners);

  //image_show_flow(img, vectors, result.tracked_cnt, opticflow.subpixel_factor);
  
  //int32_t vector_debug = vectors[0].flow_x;
  
  //stateGetPositionEnu_f()->x;

 // DOWNLINK_SEND_MONOCULAR_AVOIDANCE(DefaultChannel, DefaultDevice, &vector_debug);
 
  return FALSE;
}

 void monocular_avoidance(void) 
{
   cv_add(process_frame);
   
   /* Create the image buffers */
  image_create(&opticflow.img_gray, 272, 272, IMAGE_GRAYSCALE);
  image_create(&opticflow.prev_img_gray, 272, 272, IMAGE_GRAYSCALE);
  
  /* Create the image buffers */
  //image_create(&opticflow.img_gray, 400, 400, IMAGE_GRAYSCALE);
  //image_create(&opticflow.prev_img_gray, 400, 400, IMAGE_GRAYSCALE);
  
  /* Create the image buffers */
  //image_create(opticflow.img_gray, 272, 272, IMAGE_GRAYSCALE);
  //image_create(opticflow.prev_img_gray, 272, 272, IMAGE_GRAYSCALE);
   
   /* Set the default values */
   
   opticflow.got_first_img = FALSE;
  opticflow.max_track_corners = OPTICFLOW_MAX_TRACK_CORNERS;
  opticflow.window_size = OPTICFLOW_WINDOW_SIZE;
  opticflow.window_size = OPTICFLOW_WINDOW_SIZE;
  opticflow.subpixel_factor = OPTICFLOW_SUBPIXEL_FACTOR;
  opticflow.max_iterations = OPTICFLOW_MAX_ITERATIONS;
  opticflow.threshold_vec = OPTICFLOW_THRESHOLD_VEC;

  opticflow.fast9_adaptive = OPTICFLOW_FAST9_ADAPTIVE;
  opticflow.fast9_threshold = OPTICFLOW_FAST9_THRESHOLD;
  opticflow.fast9_min_distance = OPTICFLOW_FAST9_MIN_DISTANCE;
  
}


