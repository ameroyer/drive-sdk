//=============================================================================
// Copyright © 2008 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: FlyClient_C.c,v 1.29 2010/04/13 21:35:02 hirokim Exp $
//=============================================================================

#if defined(WIN32) || defined(WIN64)
#define _CRT_SECURE_NO_WARNINGS		
#endif

#include "C/FlyCapture2_C.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <string.h>
#include <signal.h> // to catch CTRL-C
#include "/usr/local/include/opencv2/opencv.hpp"
#include "quickselect.h"
#include "get_camera.hpp"
#include <pthread.h>
#include <math.h>
#include "../ML/state.hpp"

using namespace cv;


// Constants
static int ppm_width = 1696;
static int ppm_height = 720;
static volatile int keepRunning = 1;

void intHandler(int dummy) {
    keepRunning = 0;
}

// List of discrete states/ centroids
static std::vector<State> states_list; // List of all possible states

/**
 * Compute the median image from a sequence of images
 */
static int nmedian;
static int nthreads;

// Multithread version
void* compute_median_subthread(void* aux) { 
    int indx = *((int*) aux);
    int start = IMAGE_SIZE / nthreads * indx;
    int end = start + IMAGE_SIZE / nthreads;
     if (indx == nthreads - 1) {
	end = IMAGE_SIZE;
    }
    int i, j;
    unsigned char pix[nmedian];
    for (j = start; j < end; j++) {
	for (i = 0; i < nmedian; i++) {
	    pix[i] = input_median[i][j];
	}
        background->data[j] = quick_select(pix, nmedian);
    }	    
}

void compute_median_multithread(int nfiles, int nthread) {
    fprintf(stderr, "Background update - start\n");
    nmedian = nfiles; 
    nthreads = nthread;
    int i, ret;  
    pthread_t threads[nthread];  

    // Launch threads
    for (i = 0; i < nthread; i++) {
        ret = pthread_create (&(threads[i]), 0, compute_median_subthread,  new int(i));
    }

    // Join threads
    for (i = 0; i < nthread; i++) {
        pthread_join(threads[i], NULL);
    }
    fprintf(stderr, "Background update - end\n");
}

// One thread version
void compute_median(int nfiles, unsigned char** array, shared_struct* result) {
	    fprintf(stderr, "Background update - start\n");
    int i, j;    
    unsigned char pix[nfiles];
    for (j = 0; j < IMAGE_SIZE; j++) {
	for (i = 0; i < nfiles; i++) {
	    pix[i] = array[i][j];
	}
	result->data[j] = quick_select(pix, nfiles);
    }
    fprintf(stderr, "Background update - end\n");
}


/**
 * Get location from image using OpenCV Blob detector
 */
cv::Ptr<cv::SimpleBlobDetector> detector;
std::vector<KeyPoint> keypoints;

// Init blob detection method
void init_blob_detector() {
    SimpleBlobDetector::Params params;
    params.minThreshold = 80;
    params.filterByColor = 0;
    params.filterByConvexity = 0;
    params.filterByCircularity = 0;
    params.filterByArea = 1;
    params.minArea = 300;
    params.maxArea = 3500;
    detector = cv::SimpleBlobDetector::create(params);
}    


// Return the most likely color name associated to a hue value
const char* get_car_from_hue(int h) {
    if (h < 30 || h > 270) {
	return "red";
    } else if (h < 50) {
	return "orange";
    } else if (h < 80) {
	return "yellow";
    } else if (h < 180) {
	return "green";
    } else {
	return "blue";
    }
}

// Return hue, saturation and value of a rgb color
void get_hsv_from_rgb(float r, float g, float b, float* result) {
    // Get min and max in the color triplet
    float max = r;
    float min = g;
    if (r < b) {
	max = b;
	if (b < g) {
	    max = g;
	    min = r;
	} else {
	    if (r < g) {
		min = r;
	    }
	}
    } else {
	if (b < g) {
	    min = b;
	    if (g > r) {
		max = r;
	    }
	}
    }
    // Compute saturation
    result[1] = 0;
    if (max != 0) {
	result[1] = 1. - min / max;
    }
    // Compute value
    result[2] = max / 255.;
    // Compute hue
    if (max == min) {
	result[0] = 1;
    } else if (max == r) {
	result[0] = (g - b) / (max - min);
    } else if (max == g) {
	result[0] = 2. + (b - r) / (max - min);
    } else {
	result[0] = 4. + (r - g) / (max - min);
    }
    if (result[0] < 0) {
	result[0] = 60 * result[0] + 360;
    } else {
	result[0] = 60 * result[0];
    }
}


// Get mean hue value in a square of size ray centered around (x, y)
int get_mean_hue(unsigned char* data, int x, int y, int ray) {
    int i, j, col;
    float result[3];
    float hue, total;
    int startx =  (x >= ray) ? x - ray : 0;
    int endx =  (x + ray < ppm_width) ? x + ray : ppm_width;
    int starty =  (y >= ray) ? y - ray : 0;
    int endy =  (y + ray < ppm_height) ? y + ray : ppm_height;
    int line = starty * ppm_width * 3;

    for (i = starty; i < endy; i++) {
        for (j = startx; j < endx; j++) {
            col = line + j * 3;
	    get_hsv_from_rgb(data[col] , data[col+1], data[col+2], result);

	    // If acceptable value (ie no black or white), count it
	    if (result[2] < 0.9 && result[1] > 0.25) {
	    	hue += result[0];
		total += 1;
	    }

        }
	line += ppm_width * 3;
    }
    return hue/total;
}

// Get discretized state corresponding to a position
int get_state(float x, float y) {
    float min_dist = 5000000;
    float d;
    int result;
    for(std::vector<State>::iterator it = states_list.begin(); it != states_list.end(); ++it) {
	d = it->get_distance_squared(x, y);
	if (d < min_dist) {
	    min_dist = d;
	    result = it->get_id();
	}
    }
    return result;
}

// Update the camera location for our car and the other (obstacles)
void get_camera_loc(shared_struct* shm, int index, int verbose, const char* car_color) {
    // Detection
    Mat im;    
    cvtColor(Mat(ppm_height, ppm_width, CV_8UC3, shm->data), im, COLOR_BGR2HSV);
    detector->detect(im, keypoints);

    // Finding car
    int h;
    int obst = 0;
    camera_loc->success = 0;
    for(std::vector<KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); ++it) {
	h = get_mean_hue(shm->data, (int) (*it).pt.x, (int) (*it).pt.y, (int) (0.5 * (*it).size));
	if (camera_obst->total == 0 || !strcmp(get_car_from_hue(h), car_color)) {
	    // Update speed and direction
	    camera_loc->direction[0] = ((*it).pt.x - camera_loc-> x);
	    camera_loc->direction[1] = ((*it).pt.y - camera_loc-> y);
	    camera_loc->speed = 1. / (index - camera_loc->update_time);

	    // Update position
	    camera_loc->x = (*it).pt.x;
	    camera_loc->y = (*it).pt.y;
	    camera_loc->size = (*it).size;
	    camera_loc->state = get_state((*it).pt.x, (*it).pt.y);

	    camera_loc->success = 1;
	} 
	else if (obst < camera_obst->total) {
	    camera_obst->obst[obst*3] = (*it).pt.x;
	    camera_obst->obst[obst*3 + 1] = (*it).pt.y;
	    camera_obst->obst[obst*3 + 2] = (*it).size;
	    camera_obst->states[obst] = get_state((*it).pt.x, (*it).pt.y);
	}
    }	
    camera_obst->found = obst;
    camera_obst->update_time = index;
    camera_loc->update_time = index;

    // Additional verbose output
    if (verbose) {
	if (keypoints.size() > 0) {
	    fprintf(stderr, "Camera loc: (%.2f, %.2f) \n",  keypoints[0].pt.x, keypoints[0].pt.y);
	} else {
	    fprintf(stderr, "Error, no Camera loc found\n");
	}
    	Mat im_with_keypoints = Mat(ppm_height, ppm_width, CV_8UC3, shm->data);
	drawKeypoints( im_with_keypoints, keypoints, im_with_keypoints, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    	cvtColor(im_with_keypoints, im_with_keypoints, COLOR_BGR2RGB);
	char filename[256];
	snprintf(filename, 255, "/home/cvml1/Code/Images/KPfc2TestImage%08ld.ppm", shm->count);
	imwrite(filename,  im_with_keypoints);	
    }
}


// In case the camera localization failed, try to estimate the position by dead reckoning
void get_camera_lock_dead_reckon(int time, float result[2]) {
	result[0] = camera_loc->x + camera_loc->speed * (time - camera_loc->update_time) * camera_loc->direction[0];
	result[1] = camera_loc->y + camera_loc->speed * (time - camera_loc->update_time) * camera_loc->direction[1];
}


/**
 * Substract two camera images
 */
void sub(shared_struct* im1, shared_struct* im2, shared_struct* result){
    int i;
    for (i = 0; i < IMAGE_SIZE; i++) {
	if (im1->data[i] > im2->data[i]) {
	    result->data[i] = im1->data[i] - im2->data[i];
	} else {
	    result->data[i] = im2->data[i] - im1->data[i];
	}
    }
}

/**
 * Substract two camera images with min thresolding function
 */
void sub_thres_min(shared_struct* im1, shared_struct* im2, shared_struct* result, int threshold){
    int i;
    int aux = 0;
    for (i = 0; i < IMAGE_SIZE; i++) {
	aux = im1->data[i] - im2->data[i];
	if (aux > 0) {
	    if (aux < threshold) {
		result->data[i] = 0;
	    } else {
		result->data[i] = aux;
	    }
	} else {
	    if ( - aux < threshold) {
		result->data[i] = 0;
	    } else {
		result->data[i] = -aux;
	    }
	}
    }
}

/**
 * Substract two camera images with thresolding function
 */
void sub_thres(shared_struct* im1, shared_struct* im2, shared_struct* result, int threshold){
    int i;
    for (i = 0; i < IMAGE_SIZE; i++) {
	if (im1->data[i] > im2->data[i]) {
	    if (im1->data[i] - im2->data[i] < threshold) {
		result->data[i] = 0;
	    } else {
		result->data[i] = 255;
	    }
	} else {
	    if (im2->data[i] - im1->data[i] < threshold) {
		result->data[i] = 0;
	    } else {
		result->data[i] = 255;
	    }
	}
    }
}

/**
 * Export image in a readable PPM file and store output file path in "filename"
 */
void export_ppm(char* filename, const int width, const int height, shared_struct* shm) {
    char PPMheader[32];
    snprintf(PPMheader, 31, "P6\n%d %d 255\n", width, height);
    snprintf(filename, 255, "/home/cvml1/Code/Images/fc2TestImage%08ld.ppm", shm->count);
    FILE *fid = fopen(filename, "wb"); 
    int res = fwrite(PPMheader, strlen(PPMheader), 1, fid);
    res = fwrite( shm, 1920*1200*3, 1, fid); 
    fclose(fid);   
}

/**
 * Export image without PPM header as a txt file and store output file path in "filename"
 */
void export_txt(char* filename, const int width, const int height, shared_struct* shm) {
    snprintf(filename, 255, "/home/cvml1/Code/Images/fc2TestImage%08ld.txt", shm->count);
    FILE *fid = fopen(filename, "wb"); 
    int res = fwrite( shm, 1920*1200*3, 1, fid); 
    fclose(fid);   
}
