//=============================================================================
// Copyright  2008 Point Grey Research, Inc. All Rights Reserved.
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
#include <fstream>
#include <sstream>
#include <string>
#include <sys/time.h>
#include <time.h>
#include "../ML/state.hpp"

using namespace cv;


// Constants
static int ppm_width = 1696;
static int ppm_height = 720;
static float laptime=-1.;
static struct timeval lapfinishtime;


/**
 * car_finished: call when the camera detects the car crosses the finish line
 */
void car_finished() {
    gettimeofday(&lapfinishtime, NULL);
    laptime = (lapfinishtime.tv_sec - lapstarttime.tv_sec) + (lapfinishtime.tv_usec - lapstarttime.tv_usec) / 1000000.;
    lapstarttime = lapfinishtime;
}

/**
 * is_car_finished: return the most recent laptime, or 0 if the car is still in a lap
 */
float is_car_finished() {
    float res;
    if (laptime > 0) {
	res = laptime;
	laptime = 0.;
    } else {
	res = 0.;
    }
    return res;
}


/**
 * Initialize the global list of centroids (see state.hpp)
 */
void init_centroids_list(char* filename, int verbose) {
    std::ifstream infile(filename);
    std::string line;
    int id=0;
    while (std::getline(infile, line))
	{
	    std::istringstream iss(line);
	    float x, y, c;
	    int i, j, s;
	    if (!(iss >> x >> y >> i >> j >> c >> s)) { break; }
	    Centroid cent(id, x, y, 1. - c, i, j, s);
	    centroids_list.push_back(cent);
	    id++;
	    if (verbose) {
		printf ("centroid %d: (%f, %f) curv: %f, start: %d, lane: %d \n",cent.get_id(),cent.get_x(),cent.get_y(), cent.get_stra(), cent.get_start(), cent.get_lane());
	    }
	}
    return;
}


/**
 * Compute the median image from a sequence of images
 */
static int nmedian;
static int nthreads;

// 1. Multithread version
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

// 2. One thread version
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

/*
 * Init parameters for blob detection
 */
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


/*
 * Return the most likely color name associated to some value
 */
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

/*
 * Return hue, saturation and value from a rgb color. The result is stored in the array given as input
 */
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


/*
 * Get mean hue value in a square of size ray centered around (x, y)
 */
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

/*
 * Get closest centroids from position (x, y)
 */
int get_centroid(float x, float y) {
    float min_dist = 5000000;
    float d;
    int result;
    for(std::vector<Centroid>::iterator it = centroids_list.begin(); it != centroids_list.end(); ++it) {
	d = it->get_distance_squared(x, y);
	if (d < min_dist) {
	    min_dist = d;
	    result = it->get_id();
	}
    }
    return result;
}

/*
 * Update the camera location
 */
void get_camera_loc(shared_struct* shm, int index, int verbose, const char* car_color) {

    Mat im;
    int h, c;
    int obst = 0;

    // Detection
    cvtColor(Mat(ppm_height, ppm_width, CV_8UC3, shm->data), im, COLOR_BGR2HSV);
    detector->detect(im, keypoints);

    // Identify our car
    camera_loc->success = 0;
    for(std::vector<KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); ++it) {
	h = get_mean_hue(shm->data, (int) (*it).pt.x, (int) (*it).pt.y, (int) (0.5 * (*it).size));

	// If no opponents, or if hue matches our car, take this centroid
	if (!camera_obst->total || !strcmp(get_car_from_hue(h), car_color)) {
	    // Update speed and direction
	    camera_loc->direction[0] = ((*it).pt.x - camera_loc-> x);
	    camera_loc->direction[1] = ((*it).pt.y - camera_loc-> y);
	    camera_loc->speed = 1. / (index - camera_loc->update_time);
	    // Update position
	    camera_loc->x = (*it).pt.x;
	    camera_loc->y = (*it).pt.y;
	    camera_loc->size = (*it).size;
	    camera_loc->success = 1;
	}
	// Else detect as obsatcle
	else if (obst < camera_obst->total) {
	    camera_obst->obst[obst*3] = (*it).pt.x;
	    camera_obst->obst[obst*3 + 1] = (*it).pt.y;
	    camera_obst->obst[obst*3 + 2] = (*it).size;
	    camera_obst->centroids[obst] = get_centroid((*it).pt.x, (*it).pt.y);
	}
    }

    // If our car was not found, set coordinates with dead reckoning
    if (!camera_loc->success) {
	camera_loc->x = camera_loc->x + camera_loc->speed * (index - camera_loc->update_time) * camera_loc->direction[0];
	camera_loc->y = camera_loc->y + camera_loc->speed * (index - camera_loc->update_time) * camera_loc->direction[1];
    }

    // Set new centroid
    c = get_centroid(camera_loc->x, camera_loc->y);
    if  (c > floor(centroids_list.size() * 0.75) && camera_loc->centroid < floor(centroids_list.size() * 0.25)) {
	car_finished();
    }

    // Update
    camera_loc->centroid = c;
    camera_obst->found = obst;
    camera_obst->update_time = index;
    camera_loc->update_time = index;

    // Additional verbose output
    if (verbose) {
	// Draw
	Mat track = imread("/home/cvml1/Code/CV/discretized_track_h75_v4.png", CV_LOAD_IMAGE_COLOR);
    	Mat output = Mat(ppm_height, ppm_width, CV_8UC3, shm->data);
	add(track, output, output);
	drawKeypoints(output, keypoints, output, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	Centroid c = centroids_list[camera_loc->centroid];
	circle(output, Point(c.get_x(), c.get_y()), 5, Scalar(0, 255, 0), -1);
    	cvtColor(output, output, COLOR_BGR2RGB);
	// Write
	char filename[256];
	snprintf(filename, 255, "/home/cvml1/Code/Images/KPfc2TestImage%08ld.ppm", shm->count);
	imwrite(filename, output);
    }
}


/**
 * Substract two camera images with min thresolding function
 */
void sub(shared_struct* im, shared_struct* bg, shared_struct* result, int min){
    int i;
    int aux = 0;
    for (i = 0; i < IMAGE_SIZE; i++) {
	aux = im->data[i] - bg->data[i];
	if (aux > 0) {
	    if (aux < min) {
		result->data[i] = 0;
	    } else {
		result->data[i] = aux;
	    }
	} else {
	    result->data[i] = 0;
	}
    }
}


/**
 * Substract two camera images with min thresolding function (absolute value version)
 */
void sub_pos(shared_struct* im, shared_struct* bg, shared_struct* result, int min) {
    int i;
    int aux = 0;
    for (i = 0; i < IMAGE_SIZE; i++) {
	aux = im->data[i] - bg->data[i];
	if (aux > 0) {
	    if (aux < min) {
		result->data[i] = 0;
	    } else {
		result->data[i] = aux;
	    }
	} else {
	    if (- aux < min) {
		result->data[i] = 0;
	    } else {
		result->data[i] = - aux;
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
