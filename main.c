/*
 * Main File for controlling the car vehicle
 * Modified version of vehicle tool with threaded execution and read from file instead of stdin
 */


#include "examples/simple-c-interface/anki-simplified.h"
#include "CV/get_camera.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include "C/FlyCapture2_C.h"
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <string.h>
#include <signal.h>
#include <time.h>


/**
 * Global variables
 */
static int exit_signal = 0;
static pthread_t camera;
static AnkiHandle h;
camera_localization_t* camera_loc;
camera_localization_t* camera_obst;
shared_struct* background;
unsigned char** input_median;


/**
 * Handle keyboard interrupts
 */
static int kbint = 1;
void intHandler(int sig) {    
    fprintf(stderr, "Keyboard interrupt - execute final block\n");
    kbint = 0;  
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
 *  Print car's current location (from camera)
 */
void print_camera_loc(){
    if (camera_loc->success) {
	printf("Camera location: x: %.2f y: %.2f size: %.2f last-update: %i\n",
	       camera_loc->x, camera_loc->y, camera_loc->size, camera_loc->update_time);
    } else {
	printf("Error in camera detection at time %i\n", camera_loc->update_time); 
    }
}

/**
 * Get vehicle's location through camera (runs on independent thread)
 */
// Struct type to pass arguments to thread
struct arg_struct {
    char* vehicle_color;   // Our vehicle's color
    int update;            // Image update (time in millisecond)
    int background_update; // new background every X image updates
    int background_start;  // index of first background computation
    int history;           // nbr of images for bacground computation
    int verbose;           // 0-1: set verbosity level
};

// Main function run on the thread
void update_camera_loc(void* aux) {
    // Read arguments
    struct arg_struct *args = (struct arg_struct *)aux;
    int img_update = args->update;
    int bg_history = args->history;
    int bg_start = args->background_start;
    int bg_update = args->background_update;
    int verbose = args->verbose;

    // Init shared memory
    int shmid;
    key_t key;
    shared_struct *shm;
    key = 192012003; // 1920x1200x3
    const int width = 1696;
    const int height = 720;
    if ((shmid = shmget(key, sizeof(shared_struct), 0666)) < 0) { perror("shmget"); exit(1); }
    if ((shm = (shared_struct*)shmat(shmid, NULL, 0)) == (shared_struct *) -1) { perror("shmat"); exit(1); }

    // Additional Paramaters
    int index = 0;
    char filename[256];
    int next_bg_update = bg_start - bg_history;
    shared_struct* temp = (shared_struct*) malloc(sizeof(shared_struct));

    //Init arrays for saving past images for background computation
    int i;
    input_median = (unsigned char**) malloc(sizeof(unsigned char*) * bg_history);
    for (i = 0; i < bg_history; i++) {
	input_median[i] = malloc(sizeof(unsigned char) * IMAGE_SIZE);
    }

    /*
     * Update location until receiving exit signal
     */
    while (!exit_signal && kbint) {
	// DEBUG
	if (verbose) {
	    fprintf(stderr, "Index: %d - Next bg update: %d - Current saving index: %d\n", index, next_bg_update  - next_bg_update%bg_update + bg_start, (next_bg_update + bg_history - bg_start) % bg_update);
	}

	// Update background if needed
	if ((next_bg_update - bg_start) % bg_update  == 0) {
	    next_bg_update += bg_update - bg_history;	
	    //compute_median(bg_history, input_median, background);
	    compute_median_multithread(bg_history, 8);
	    if (verbose) {
		export_ppm(filename, width, height, background);
	    }
	}

	// Compute differential image in temp and update location
	temp->count = index + 1;
	//sub(shm, background, temp);
	sub_thres(shm, background, temp, 80);
	get_camera_loc(temp, index, verbose);

	// DEBUG
	if (verbose) {
	    export_ppm(filename, width, height, temp);
	}

	// If needed, save current image for next background update
	if (index == next_bg_update) {
	    memcpy(input_median[(next_bg_update + bg_history - bg_start) % bg_update], shm->data, IMAGE_SIZE);
	    next_bg_update += 1;
	}	   
	index += 1;
	usleep(img_update * 1000);
    }
    
    
    // Free and close
    if ( shmdt(shm) == -1) { perror("shmdt"); exit(1); } 
    for (i = 0; i < bg_history; i++) {
	free(input_median[i]);
    }
    free(input_median);
} 



/**
 * Return car's MAC address given color or name
 */
char* get_car_mac(char* color) {
    if (!strcmp(color, "grey") || !strcmp(color, "boson") || !strcmp(color, "gray")) {
        return "D9:81:41:5C:D4:31"; 
    }
    else if (!strcmp(color, "blue") || !strcmp(color, "katal")) {
        return "D8:64:85:29:01:C0";
    }
    else if (!strcmp(color, "koural") || !strcmp(color, "yellow")) {
        return "EB:0D:D8:05:CA:1A";
    }
    else if (!strcmp(color, "nuke") || !strcmp(color, "green")) {
        return "C5:34:5D:26:BE:53";
    }
    else if (!strcmp(color, "hadion") || !strcmp(color, "orange")) {
        return "D4:48:49:03:98:95";
    }
    else {
        return "E6:D8:52:F1:D9:43";
    }
} 


/**
 * Main routine
 **/
int main(int argc, char *argv[]) {    
    signal(SIGINT, intHandler);
    /*
     * Read parameters and Initialization
     */
    if(argc<2){
	fprintf(stderr, "usage: %s car-name [nbr of cars] [adaptater] [verbose]\n",argv[0]);
	exit(0);
    }
    const char* adapter = "hci0";
    char* car_id  = get_car_mac(argv[1]);
    int opponents = 3;
    if (argc > 2) {
	adapter = argv[2];
	if (argc > 3) {
		opponents = atoi(argv[3]);
    		camera_obst = (camera_localization_t*) malloc(sizeof(camera_localization_t) * opponents);
	}
    }
    init_blob_detector();
    camera_loc = (camera_localization_t*) malloc(sizeof(camera_localization_t));

    /*
     * Load thread to Update picture every second and process it  
     */
    // Set arguments
    struct arg_struct args;
    args.vehicle_color = car_id;
    args.update = 50; //time in milliseconds
    args.background_update = 1000;
    args.background_start = 20;
    args.history = 15;
    args.verbose = argc > 4;

    // Load default background
    background = (shared_struct*) malloc(sizeof(shared_struct));
    background->count = 0;
    FILE *f = fopen("/home/cvml1/Code/Images/default_background.txt", "rb");
    fread(background->data, IMAGE_SIZE, 1, f);
    fclose(f);

    // Launch camera thread
    int ret = pthread_create (&camera, 0, (void*)update_camera_loc,  &args);
    
    /*
     * Set vehicle's behaviour
     */
    // Init bluethooth and wait for connection successful
    fprintf(stderr, "Attempting connection to %s\n", car_id);
    h = anki_s_init(adapter, car_id, argc>4);
    while(!anki_s_is_connected(h) || !anki_s_is_sdk_ctrl_mode(h));
    fprintf(stderr, "Connection successful\n");

    // Set initial speed
    int res;
    res = anki_s_set_speed(h,600,20000);
    // Define correct direction
    int race_clockwise=0;

    int update_time_previous=-1;

    // Run until ctrl-C
    // print locations every 0.5 seconds
    while (kbint && !res) {
	usleep(500*1000);
	print_loc(h);
	print_camera_loc();
	printf("\n");
	//check if car stands till (no update of loc information) and set speed randomly  if so
	if(update_time_previous==anki_s_get_localization(h).update_time){
		printf("standing still\n");		 
		anki_s_set_speed(h,500+1000*((double) rand() / (RAND_MAX)),20000);
		
	}
	//check direction and perform uturn if false
	if(anki_s_get_localization(h).is_clockwise!=race_clockwise){
		anki_s_uturn(h); //TODO: does not work!
		printf("uturn\n");
	}
	update_time_previous=anki_s_get_localization(h).update_time;
    }

    /*
     * Close and disconnect
     */
    anki_s_close(h);
    exit_signal = 1;
    pthread_join(camera, NULL);
    free(background);
    free(camera_loc);
    return 0;
}
