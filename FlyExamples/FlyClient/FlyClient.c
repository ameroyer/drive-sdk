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


void GrabImagesFromSharedMemory( int numImagesToGrab )
{
    int i;

    signal(SIGINT, intHandler);

    int shmid;
    key_t key;
    shared_struct *shm;
    key = 192012003; // 1920x1200x3
    const int width = 1696;
    const int height = 720;
    const int channels = 3;

    /* Find shared memory segment.  */
    if ((shmid = shmget(key, sizeof(shared_struct), 0666)) < 0) { perror("shmget"); exit(1); }

    /* Attach shared memory segment to our data space.  */
    if ((shm = (shared_struct*)shmat(shmid, NULL, 0)) == (shared_struct *) -1) { perror("shmat"); exit(1); }

    char PPMheader[32];
    snprintf(PPMheader, 31, "P6\n%d %d 255\n", width, height);
    
    for ( i=0; (i < numImagesToGrab) && keepRunning ; i++ )
    {
	char filename[256];
        snprintf(filename, 255, "fc2TestImage%08ld.ppm", shm->count);
        FILE *fid = fopen(filename, "wb"); 
        if (fid == 0) {
            printf( "Error in fopen.\n");
            printf( "Please check write permissions.\n");
            continue;
        }
        int res = fwrite( PPMheader, strlen(PPMheader), 1, fid);
        if (res == 0) {
            printf( "Error in fwrite header.\n");
            printf( "Please check write permissions.\n");
        } else {
            res = fwrite( shm, 1920*1200*3, 1, fid);   
            if (res == 0) {
                printf( "Error in fwrite body.\n");
                printf( "Please check write permissions.\n");
            }
        }
        fclose(fid);
    }

    /* detach local  from shared memory */
    if ( shmdt(shm) == -1) { perror("shmdt"); exit(1); } 
    
}

int main(int argc, char** argv)
{        
    const int k_numImages = 10;

    GrabImagesFromSharedMemory( k_numImages );   

    printf( "Done! \n" );
 //   getchar();

    return 0;
}

