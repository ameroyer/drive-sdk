/*
 * Main File for controlling the car vehicle
 * Modified version of vehicle tool with threaded execution and read from file instead of stdin
 */


#include "examples/simple-c-interface/anki-simplified.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include "C/FlyCapture2_C.h"
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <string.h>

static int exit_signal = 0;

/**
 * Self-localization information
 */
typedef struct camera_localization {
    int x    ;    /// x coordinate
    int y ;    /// y coordinate
    int update_time ; /// Last update time (is increased every time we get an update from the car)
} camera_localization_t
;


/**
 * Structure result of image grabbed from camera
 */
#define IMAGE_SIZE 1920*1200*3
typedef struct {
    unsigned char data[IMAGE_SIZE];
    long count;
} shared_struct;




/**
 * Get background image as media image
 *
 */
void get_background_as_median(char** image_sequences){
    fprintf(stderr, "Compute background\n");
    int i;
}


/**
 * Get vehicle's location through camera (runs on independent thread)
 */
struct arg_struct {
    char* vehicle_color;
    char* default_background;
    int update;
    int background_update;
    int history;
};

void update_camera_loc(void* aux) {
    // Read arguments
    struct arg_struct *args = (struct arg_struct *)aux;
    int history = args->history;
    int update = args->update;
    int background_update = args->background_update;

    // Init parameters
    int shmid;
    key_t key;
    shared_struct *shm;
    key = 192012003; // 1920x1200x3
    const int width = 1696;
    const int height = 720;
    
    /* Find shared memory segment.  */
    if ((shmid = shmget(key, sizeof(shared_struct), 0666)) < 0) { perror("shmget"); exit(1); }
    /* Attach shared memory segment to our data space.  */
    if ((shm = (shared_struct*)shmat(shmid, NULL, 0)) == (shared_struct *) -1) { perror("shmat"); exit(1); }

    // Grab picture every "update"  seconds and update location
    char arr[20][256];
    int index = 0;
    char imagefile [256];

    while (!exit_signal) {
	if (index && (index % background_update == 0)) {
	    get_background_as_median(arr);
	}
	GrabImageFromSharedMemory(arr[index % history], shmid, key, shm, width, height);
	//fprintf(stderr, "%s",  a[index]);
	index += 1;
	sleep(update);
    }
    
    
    /* Detach local  from shared memory */
    if ( shmdt(shm) == -1) { perror("shmdt"); exit(1); } 


} 



/**
 * Return car's MAC address given color or name
 */
char* get_car_mac(char* color) {
    if (strcmp(color, "grey") || strcmp(color, "boson") || strcmp(color, "gray")) {
        return "D9:81:41:5C:D4:31"; 
    }
    else if (strcmp(color, "blue") || strcmp(color, "katal")) {
        return "D8:64:85:29:01:C0";
    }
    else if (strcmp(color, "koural") || strcmp(color, "yellow")) {
        return "EB:0D:D8:05:CA:1A";
    }
    else {
        return "E6:D8:52:F1:D9:43";
    }
} 

/**
 *  Print car's current location
 */
void print_loc(AnkiHandle h){
    localization_t loc;
    loc = anki_s_get_localization(h);
    printf("Location: segm: %03x subsegm: %03x clock-wise: %i last-update: %i\n",
	   loc.segm, loc.subsegm, loc.is_clockwise, loc.update_time);
}



/**
 * Main routine
 **/
int main(int argc, char *argv[]) {
    // Update picture every second and process it  
    struct arg_struct args;
    args.vehicle_color = "grey";
    args.default_background = "";
    args.update = 1;
    args.background_update = 2;
    args.history = 20;

    pthread_t camera;
    int ret = pthread_create (&camera, 0, (void*)update_camera_loc,  &args);
    sleep(5);
    //GrabImagesFromSharedMemory(1);  
    /*
    // Read parameters
    if(argc<2){
    fprintf(stderr, "usage: %s car-name [adaptater] [verbose]\n",argv[0]);
    exit(0);
    }
    const char* adapter = "hci0";
    const char* car_id  = get_car_mac(argv[1]);
    if (argc > 2) {
    adapter = argv[2];
    }
    // Init bluethooth and wait for connection successful
    fprintf(stderr, "Attempting connection\n");
    AnkiHandle h = anki_s_init(adapter, car_id, argc>3);
    while(!anki_s_is_connected(h)) {};
    fprintf(stderr, "Connection successful\n");

    // Send commands
    int i;
    if(anki_s_set_speed(h,1000,20000)!=0) return 1;
    print_loc(h);
    //for(i=0; i<10; i++){ usleep(200000); anki_s_change_lane(h,-40,100,1000); }  
    for(i=0; i<10; i++){ usleep(200000);  print_loc(h);  }
    anki_s_set_speed(h,500,20000);
    for(i=0; i<10; i++){ usleep(200000);  print_loc(h);  }
    anki_s_change_lane(h,-50,100,1000);
    for(i=0; i<10; i++){ usleep(200000);  print_loc(h);  }
    anki_s_set_speed(h,0,20000);
    sleep(1);

    // Disconnect
    anki_s_close(h);*/
    exit_signal = 1;
    return 0;
}
