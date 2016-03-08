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

#ifndef OPTICFLOW_FAST9_THRESHOLD
#define OPTICFLOW_FAST9_THRESHOLD 5//20
#endif

#ifndef OPTICFLOW_FAST9_MIN_DISTANCE
#define OPTICFLOW_FAST9_MIN_DISTANCE 10
#endif

#ifndef OPTICFLOW_FAST9_ADAPTIVE
#define OPTICFLOW_FAST9_ADAPTIVE TRUE
#endif

struct opticflow_result_t *result;
//struct opticflow_t *opticflow;

bool_t process_frame(struct image_t* img);
bool_t process_frame(struct image_t* img)
{
  
  // *************************************************************************************
  // Corner detection
  // *************************************************************************************

  // FAST corner detection (TODO: non fixed threshold)
  struct point_t *corners = fast9_detect(img, OPTICFLOW_FAST9_THRESHOLD, OPTICFLOW_FAST9_MIN_DISTANCE,
                                         20, 20, &result->corner_cnt);

  /*
  // Adaptive threshold
  if (opticflow->fast9_adaptive) {

    // Decrease and increase the threshold based on previous values
    if (result->corner_cnt < 40 && opticflow->fast9_threshold > 5) {
      opticflow->fast9_threshold--;
    } else if (result->corner_cnt > 50 && opticflow->fast9_threshold < 60) {
      opticflow->fast9_threshold++;
    }
  }
*/
  image_show_points(img, corners, result->corner_cnt);
//int32_t debug = result->corner_cnt;
int32_t debug = img->h;
  
  DOWNLINK_SEND_MONOCULAR_AVOIDANCE(DefaultChannel, DefaultDevice, &debug);
  return FALSE;
}

 void monocular_avoidance(void) 
{
   cv_add(process_frame);
   
   /* Set the default values */
   /*
  opticflow->max_track_corners = OPTICFLOW_MAX_TRACK_CORNERS;
  opticflow->window_size = OPTICFLOW_WINDOW_SIZE;
  opticflow->subpixel_factor = OPTICFLOW_SUBPIXEL_FACTOR;
  opticflow->max_iterations = OPTICFLOW_MAX_ITERATIONS;
  opticflow->threshold_vec = OPTICFLOW_THRESHOLD_VEC;

  opticflow->fast9_adaptive = OPTICFLOW_FAST9_ADAPTIVE;
  opticflow->fast9_threshold = OPTICFLOW_FAST9_THRESHOLD;
  opticflow->fast9_min_distance = OPTICFLOW_FAST9_MIN_DISTANCE;
*/   
}


