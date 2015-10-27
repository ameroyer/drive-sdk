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
using namespace cv;


static volatile int keepRunning = 1;

void intHandler(int dummy) {
    keepRunning = 0;
}

static int ppm_width = 1696;
static int ppm_height = 720;

/**
 * Compute the median image from a sequence of images
 */
void compute_median(int nfiles, unsigned char** array, shared_struct* result) {
    int i, j;    
    unsigned char pix[nfiles];
    for (j = 0; j < IMAGE_SIZE; j++) {
	for (i = 0; i < nfiles; i++) {
	    pix[i] = array[i][j];
	}
	result->data[j] = quick_select(pix, nfiles);
    }
}


/**
 * Get location from image using OpenCV Blob detector
 */
cv::Ptr<cv::SimpleBlobDetector> detector;
std::vector<KeyPoint> keypoints;

void init_blob_detector() {
    SimpleBlobDetector::Params params;
    params.thresholdStep = 1;
    params.minThreshold = 120;
    params.maxThreshold = 122;
    params.filterByColor = 0;
    params.filterByConvexity = 0;
    params.filterByCircularity = 0;
    params.filterByArea = 1;
    params.minArea = 50;
    params.maxArea = 3000;
    detector = cv::SimpleBlobDetector::create(params);
}
    

void get_camera_loc(shared_struct* shm) {
    //TO TEST
    //TODO check block average color
    //TODO if keypoints empty use anki location
    Mat im = Mat(ppm_height, ppm_width, CV_8UC3, shm->data);
    detector->detect(im, keypoints);
    if (keypoints.size() > 0) {
	fprintf(stderr, "Camera loc: (%.2f, %.2f) \n",  keypoints[0].pt.x, keypoints[0].pt.y);
    } else {
	fprintf(stderr, "Error, no Camera loc found\n");
    }
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
 * Substract two camera images with thresolding function
 */
//TODO instead of 255 opn three channel, put 1 channel and chhose color accordingly
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
 * Export image in a readable PPM file
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
 * Export image without PPm header as a txt file
 */
void export_txt(char* filename, const int width, const int height, shared_struct* shm) {
    snprintf(filename, 255, "/home/cvml1/Code/Images/fc2TestImage%08ld.txt", shm->count);
    FILE *fid = fopen(filename, "wb"); 
    int res = fwrite( shm, 1920*1200*3, 1, fid); 
    fclose(fid);   
}
