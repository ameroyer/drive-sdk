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

/**
 * Structures types
 */
// Camera images
#define IMAGE_SIZE 1920*1200*3
typedef struct {
    unsigned char data[IMAGE_SIZE];
    long count;
} shared_struct;

// Position from camera
typedef struct camera_localization {
    int x    ;    /// x coordinate
    int y ;    /// y coordinate
    int update_time ; /// Last update time (is increased every time we get an update from the car)
} camera_localization_t
;


/** 
 * Global variables
 */
static int exit_signal = 0;
static shared_struct* background;



/**
 * Get background image as median image
 *
 */
void get_background_as_median(int len, char image_sequences[][256], char* output){
    char cmd[1000];
    sprintf(cmd, "python Python/compute_median.py %s", output);
    int i;
    for (i = 0; i < len; i++) {
	strcat(cmd, " ");
	strcat(cmd, image_sequences[i]);
    }
    system(cmd);
}


/**
 * Get vehicle's location through camera (runs on independent thread)
 */
struct arg_struct {
    char* vehicle_color;
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
    char filename[256];
    shared_struct* temp = (shared_struct*) malloc(sizeof(shared_struct));
    
    /* Find shared memory segment.  */
    if ((shmid = shmget(key, sizeof(shared_struct), 0666)) < 0) { perror("shmget"); exit(1); }
    /* Attach shared memory segment to our data space.  */
    if ((shm = (shared_struct*)shmat(shmid, NULL, 0)) == (shared_struct *) -1) { perror("shmat"); exit(1); }

    // Grab picture every "update"  seconds and update location
    char arr[history][256];
    int index = 0;

    while (!exit_signal) {
	//Store images for background update
	if (index && (index % background_update == 0)) {
	    fprintf(stderr, "Update Background\n");
	    //get_background_as_median(history, arr, background);
	}
	//Compute differential image
	sub(shm, background, temp);
	export_ppm(filename, width, height, temp);
	export_ppm(filename, width, height, shm);
	   
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
    /*
     * Load thread to Update picture every second and process it  
     */
    //Set arguments
    struct arg_struct args;
    args.vehicle_color = "grey";
    args.update = 1;
    args.background_update = 60;
    args.history = 10;

    // Load default background
    background = (shared_struct*) malloc(sizeof(shared_struct));
    background->count = 0;
    FILE *f = fopen("/home/cvml1/Code/Images/default_background.txt", "rb");
    long fsize = IMAGE_SIZE;
    fread(background->data, fsize, 1, f);
    fclose(f);

    // Launch thread
    pthread_t camera;
    char filename[256];
    export_ppm(filename, 1696, 720, background);
    int ret = pthread_create (&camera, 0, (void*)update_camera_loc,  &args);
    sleep(1);


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
    pthread_join(camera, NULL);
    //free(background);
    return 0;
}
