/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * opencv
 */


#include "opencv_example.h"



using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

#include <cmath>


int opencv_example(char* img, int width, int height)
{
	// Create a new image, using the original bebop image.
	Mat M(width,height, CV_8UC2, img);
	Mat image;
	// If you want a color image, uncomment this line
	// cvtColor(M, image, CV_YUV2RGB_Y422);
	// For a grayscale image, use this one
	cvtColor(M, image, CV_YUV2GRAY_Y422);

	// Blur it, because we can
	blur(image, image, Size(5,5));

	// Canny edges, only works with grayscale image
	int edgeThresh=35;
	Canny(image, image, edgeThresh, edgeThresh*3);

	// Convert back to YUV422, and put it in place of the original image
	for (int row=0; row <height; row++){
		for (int col=0; col <width; col++){
			img[(row*width+col)*2+1] = image.at<uint8_t>(row,col);
			img[(row*width+col)*2] = 127;
		}
	}
	return 0;
}

int derotation_test(char* img, int width, int height, float theta, float phi, float psi, float theta_prev, float phi_prev, float psi_prev)
{
	// Create a new image, using the original bebop image.
	Mat M(width,height, CV_8UC2, img);
	Mat image;
	vector<Point2f> corner;
	vector<Point2f> corner_prev;
	// If you want a color image, uncomment this line
	// cvtColor(M, image, CV_YUV2RGB_Y422);
	// For a grayscale image, use this one
	cvtColor(M, image, CV_YUV2GRAY_Y422);

	int max_points = 10;
	int x = 0;
        int y = 0;
	int x_prev = 0;
        int y_prev = 0;
	float divergence = 0;
	float x_focal = 100;//?
	float y_focal = 100;//?
	int x_total = 0;
	int y_total = 0;
	
	for (int i=0; i <max_points; i++)
	{
	 x = corner[i].x-(width/2);
         y = corner[i].y-(height/2);
	 x_prev = corner_prev[i].x-(width/2);
         y_prev = corner_prev[i].y-(height/2);
	 
	 //optional distortion correction here 
	 
	float x_angle = atan((float)x/x_focal);
	float y_angle = atan((float)y/y_focal);
	 
	 int u = x - x_prev;
	 int v = y - y_prev;
	 x_total += u;
	 y_total += v;
	}
	

	// Convert back to YUV422, and put it in place of the original image
	for (int row=0; row <height; row++){
		for (int col=0; col <width; col++){
			img[(row*width+col)*2+1] = image.at<uint8_t>(row,col);
			img[(row*width+col)*2] = 127;
		}
	}
	return 0;
}
