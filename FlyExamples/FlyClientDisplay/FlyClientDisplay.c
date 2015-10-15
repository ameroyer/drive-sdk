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
#include <unistd.h>
#include <SDL/SDL.h>

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


void GrabImagesFromSharedMemory(SDL_Surface* screen, int start_slow  )
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
  SDL_Surface *surface;
  Uint32 rmask, gmask, bmask, amask;
  SDL_Event event;
  int pause=0;
  int slow=start_slow;

  /* Find shared memory segment.  */
  if ((shmid = shmget(key, sizeof(shared_struct), 0666)) < 0) { perror("shmget"); exit(1); }

  /* Attach shared memory segment to our data space.  */
  if ((shm = (shared_struct*)shmat(shmid, NULL, 0)) == (shared_struct *) -1) { perror("shmat"); exit(1); }
    
  rmask = 0x0000ff;
  gmask = 0x00ff00;
  bmask = 0xff0000;
  amask = 0x000000;

  surface = SDL_CreateRGBSurface(0, width, height, 24, rmask, gmask, bmask, amask);


  while (  keepRunning )
    {

      if(!pause){
	memcpy(surface->pixels,shm,width*height*3);
	/* Blit onto the screen surface  (Double Buffering) */
	if(SDL_BlitSurface(surface, NULL, screen, NULL) < 0)
	  fprintf(stderr, "BlitSurface error: %s\n", SDL_GetError());
	
	SDL_UpdateRect(screen, 0, 0, width, height);
	
      }
      if(slow) usleep(1000000/2);  // 2 frames per sec (for remote)
      else   usleep(5000);

      while( SDL_PollEvent( &event ) ){
	switch(event.type){
	case SDL_QUIT: keepRunning=0; break;
	case SDL_KEYDOWN:
	  switch(event.key.keysym.sym){
	  case SDLK_s: slow = !slow; break;
	  case SDLK_SPACE: pause = !pause; break;
	  }; 
	  break;

	default: break;
	}
      }

    }

  /* detach local  from shared memory */
  if ( shmdt(shm) == -1) { perror("shmdt"); exit(1); } 
    
}

int main(int argc, char** argv)
{        
  /* initialize SDL */
  SDL_Init(SDL_INIT_VIDEO);

  /* set the title bar */
  SDL_WM_SetCaption("FlyCap Client 4 Car Race", "FlyCap Client");

  /* create window */
  SDL_Surface* screen = SDL_SetVideoMode(1696, 720, 0, SDL_SWSURFACE);

  GrabImagesFromSharedMemory( screen, argc>=2 );   

  SDL_Quit();

  //   getchar();

  return 0;
}
