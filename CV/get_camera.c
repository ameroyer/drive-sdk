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

static volatile int keepRunning = 1;

void intHandler(int dummy) {
    keepRunning = 0;
}

#define IMAGE_SIZE 1920*1200*3
typedef struct {
    unsigned char data[IMAGE_SIZE];
    long count;
} shared_struct;



/**
 * Compute the median image from a sequence of images
 */
void compute_median(int nfiles, char** array, shared_struct* result) {
    int i, j;    
    char pix[nfiles];
    for (j = 0; j < IMAGE_SIZE; j++) {
	for (i = 0; i < nfiles; i++) {
	    pix[i] = array[i][j];
	}
	result->data[j] = quick_select(pix, nfiles);
    }
}



void get_camera_loc(shared_struct* shm, const int width, const int height){
    int x, y, i;
    int min_x = 1920;
    int max_x = 0;
    int min_y = 1200;
    int max_y = 0;
    for (i = 0; i < IMAGE_SIZE / 3; i++) {
	if (shm->data[i] != 0) {
	    x = i / width;
	    y = i % width;
	    if (x < min_x) {
		min_x = x;}
	    else if (x > max_x) {
		max_x = x;}
	    if (y < min_y) {
		min_y = y;}
	    else if (y > max_y) {
		max_y = y;}
	}
    }
    fprintf(stderr, "Camera loc: (%d, %d) x (%d, %d)\n", min_x, min_y, max_x, max_y);
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


/*
  int main(int argc, char** argv)
  {        
  const int k_numImages = 10;

  GrabImagesFromSharedMemory( k_numImages );   

  printf( "Done! \n" );
  //   getchar();

  return 0;
  }
*/
