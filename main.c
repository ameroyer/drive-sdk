/*
 * Main File for controlling the car vehicle
 * Modified version of vehicle tool with threaded execution and read from file instead of stdin
 */


#include "examples/simple-c-interface/anki-simplified.h"
#include "CV/get_camera.hpp"
#include "ML/policies.hpp"
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

// Bash colors
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define KBOLD  "\x1B[1m"
#define RESET "\033[0m"


/**
 * Global variables
 */
static int camera_index = 0; //index for camera update
static int run_index = 0; //index for global simulation update
static int exit_signal = 0;
static pthread_t camera; // Thread running the camera detection
static AnkiHandle h;
camera_localization_t* camera_loc;
camera_obst_localization_t* camera_obst;
shared_struct* background;
unsigned char** input_median;
struct timeval lapstarttime;



/**
 * Handle keyboard interrupts
 */
static int kbint = 1;
static void intHandler(int sig) {
    fprintf(stderr, "Keyboard interrupt - execute final block\n");
    kbint = 0;
}


/**
 *  Print car's current location
 */
void print_loc(AnkiHandle h){
    localization_t loc;
    loc = anki_s_get_localization(h);
    printf(KBLU "[Loc]" RESET " segm: %03x subsegm: %03x clock-wise: %i last-update: %i\n", loc.segm, loc.subsegm, loc.is_clockwise, loc.update_time);
}


/**
 *  Print car's current location (from camera)
 */
void print_camera_loc(){
    if (camera_loc->success) {
	printf(KBOLD KGRN "[Camera loc]" RESET " centroid: %d x: %.2f y: %.2f size: %.2f last-update: %i\n",
	       camera_loc->centroid,camera_loc->x, camera_loc->y, camera_loc->size, camera_loc->update_time);
    } else {
	printf(KBOLD KRED "[Camera loc (DR)]" RESET " centroid: %d x: %.2f y: %.2f size: %.2f last-update: %i\n", camera_loc->centroid,camera_loc->x, camera_loc->y, camera_loc->size, camera_loc->update_time);
    }
}

/**
 * Get vehicle's location through camera (runs on independent thread)
 */
// Struct type to pass arguments to thread
struct arg_struct {
    const char* vehicle_color;   // Our vehicle's color
    int update;            // Image update (time in millisecond)
    int background_update; // new background every X image updates
    int background_start;  // index of first background computation
    int history;           // nbr of images for bacground computation
    int n_obst;            //nbr  of opponents on the track
    int verbose;           // 0-1: set verbosity level
};



// Main function run on the camera detection thread
void* update_camera_loc(void* aux) {

    // Read arguments
    struct arg_struct *args = (struct arg_struct *)aux;
    const char* car_color = args->vehicle_color;
    int img_update = args->update;
    int bg_history = args->history;
    int bg_start = args->background_start;
    int bg_update = args->background_update;
    int verbose = args->verbose;

    //Init structures
    init_blob_detector();
    init_centroids_list("CV/centroids_h75_v4.txt", verbose);

    camera_obst = (camera_obst_localization_t*) malloc(sizeof(camera_obst_localization_t));
    camera_obst->total = args->n_obst;

    camera_loc = (camera_localization_t*) malloc(sizeof(camera_localization_t));
    camera_loc->x = 850;
    camera_loc->y = 50;
    camera_loc->size = 0;
    camera_loc->speed = 1.;
    camera_loc->direction[0] = 1;
    camera_loc->direction[1] = 0;
    camera_loc->centroid=0;
    camera_loc->update_time = 0;
    camera_loc->is_clockwise = 1;

    input_median = (unsigned char**) malloc(sizeof(unsigned char*) * bg_history);
    int i;
    for (i = 0; i < bg_history; i++) {
	input_median[i] = (unsigned char*) malloc(sizeof(unsigned char) * IMAGE_SIZE);
    }

    // Init shared memory
    int shmid;
    key_t key;
    shared_struct *shm;
    key = 192012003; // 1920x1200x3
    const int width = 1696;
    const int height = 720;
    if ((shmid = shmget(key, sizeof(shared_struct), 0666)) < 0) { perror("shmget"); exit(1); }
    if ((shm = (shared_struct*)shmat(shmid, NULL, 0)) == (shared_struct *) -1) { perror("shmat"); exit(1); }

    // Load background
    background = (shared_struct*) malloc(sizeof(shared_struct));
    background->count = 0;
    //FILE *f = fopen("/home/cvml1/Code/Images/default_background.txt", "rb");
    //fread(background->data, IMAGE_SIZE, 1, f);
    //fclose(f);
    memcpy(background->data, shm->data, IMAGE_SIZE);


    // Additional Paramaters
    camera_index = 0;
    char filename[256];
    int next_bg_update = bg_start - bg_history;
    shared_struct* temp = (shared_struct*) malloc(sizeof(shared_struct));


    /*
     * Update location until receiving exit signal
     */
    while (!exit_signal && kbint) {
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
	temp->count = camera_index + 1;
	sub(shm, background, temp, 80);
	get_camera_loc(temp, camera_index, verbose, car_color);

	// If needed, save current image for next background update
	if (camera_index == next_bg_update) {
	    memcpy(input_median[(next_bg_update + bg_history - bg_start) % bg_update], shm->data, IMAGE_SIZE);
	    next_bg_update += 1;
	}

	// Next
	camera_index += 1;
	usleep(img_update * 1000);
    }

    // Close
    if ( shmdt(shm) == -1) { perror("shmdt"); exit(1); }
    for (i = 0; i < bg_history; i++) {
	free(input_median[i]);
    }
    free(input_median);
}



/**
 * Return car's MAC address given color or name
 */
const char* get_car_mac(char* color) {
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
 * Return car's color given color or name
 */
const char* get_car_color(char* color) {
    if (!strcmp(color, "grey") || !strcmp(color, "boson") || !strcmp(color, "gray")) {
        return "grey";
    }
    else if (!strcmp(color, "blue") || !strcmp(color, "katal")) {
        return "blue";
    }
    else if (!strcmp(color, "koural") || !strcmp(color, "yellow")) {
        return "yellow";
    }
    else if (!strcmp(color, "nuke") || !strcmp(color, "green")) {
        return "green";
    }
    else if (!strcmp(color, "hadion") || !strcmp(color, "orange")) {
        return "orange";
    }
    else {
        return "red";
    }
}



/**
 * Main routine
 **/
int main(int argc, char *argv[]) {
    signal(SIGINT, intHandler);

    /*
     * Hyper parameters
     */
    float camera_update = 0.04; // Update of the camera picture, in percent of seconds
    float control_update = 0.5; // Update of the vehicle action, `` `` ``
    float background_update = 5; // Update of the background, `` `` ``
    int background_start = 20;  // Index at which the background computation starts
    int background_history = 10; // Number of images to use for median computation
    int nlap = 5; // Number of laps before the car stops
    // training paramters
    int training = 1;//1;
    //int training = 0;
    int nepisodes = 20;
    int nsteps = 30;
    //control_update = 0.3;

    /*
     * Read parameters
     */
    if(argc<2){
	fprintf(stderr, "usage: %s car-name [nbr of opponents] [adaptater] [verbose]\n",argv[0]);
	exit(0);
    }
    const char* adapter = "hci0";
    const char* car_id  = get_car_mac(argv[1]);
    const char* car_color  = get_car_color(argv[1]);
    int opponents = 3;
    if (argc > 2) {
	opponents = atoi(argv[2]);
	if (argc > 3) {
	    adapter = argv[3];
	}
    }

    /*
     * Load thread for camera update and processing
     */
    gettimeofday(&lapstarttime, NULL);
    struct arg_struct camera_args = {car_color, 1000 * camera_update, 1000 * background_update, background_start, background_history, opponents, argc > 4};
    int ret = pthread_create (&camera, 0, update_camera_loc,  &camera_args);


    /*
     * Control vehicle's behaviour (run until ctrl-c)
     */

    // Init bluethooth and wait for connection successful
    fprintf(stderr, "Attempting connection to %s\n", car_id);
    h = anki_s_init(adapter, car_id, argc>4);
    fprintf(stderr, "Connection successful\n");

    // Additional parameters
    run_index = 0;
    int res, lap;
    //localization_t loc;
    float laptime=-1.;
    float totaltime = 0.;
    float minlaptime = 50.;
    //float epsilon = 0.0001;
    //float previous_camera_loc[2] = {camera_loc->x, camera_loc->y};

    /*
     * 1. First: normal (non trainng) mode
     */
    if (!training) {
	// Initialize a policy
	init_det_one_car_policy();

	// Start
	res = anki_s_set_speed(h, 1500, 5000);
	camera_loc->real_speed = 1500;

	while (kbint && !res && nlap > 0) {
	    // Display
	    print_loc(h);
	    print_camera_loc();
	    printf("\n");

	    // Check if lap finished
	    laptime = is_car_finished();
	    if (laptime > 1.){
		nlap -= 1;
		fprintf(stderr, "    > Lap time: %.3f\n\n", laptime);
		totaltime += laptime;
		if (laptime < minlaptime) {
		    minlaptime = laptime;
		}
	    }

	    // Check direction and perform uturn if false
	    if(!camera_loc->is_clockwise){
		anki_s_uturn(h);
		fprintf(stderr, "U-turn\n");
	    }

	    // Check if car stands still (no update of loc information) and set speed randomly  if so
	    //if(camera_loc->success && abs(previous_camera_loc[0] - camera_loc->x) < epsilon && abs(previous_camera_loc[1] - camera_loc->y) < epsilon){
	    //anki_s_set_speed(h,500 + 1000 * ((double) rand() / (RAND_MAX)), 20000);
	    //	fprintf(stderr, "Still\n");
	    //  }

	    // Apply policy decsion
	    res = apply_policy(h, *camera_loc);
	    if (res < 0) { // changed speed
		camera_loc->real_speed = - res;
		res = 0;
	    }

	    // Next loop
	    run_index += 1;
	    //previous_camera_loc[0] = camera_loc->x;
	    //previous_camera_loc[1] = camera_loc->y;
	    usleep(control_update * 1000000);
	}
    }
    /*
     * 2. Or: training mode
     */
    else {
	init_totrain_onecar_policy(0.9);
	export_policy(0,  "/home/cvml1/Code/TrainRuns/");
	// Start
	res = anki_s_set_speed(h, 1000, 20000);
	camera_loc->real_speed = 1000;
	int episode, step;
	for (episode = 0; episode < nepisodes; episode++) {
		fprintf(stderr, "EPISODE %d \n",episode);
	    for (step = 0; step < nsteps; step++) {
		fprintf(stderr, "STEP %d \n ",step);
		// Display
		print_loc(h);
		print_camera_loc();
		printf("\n");

		// Check direction and perform uturn if false
		if(!camera_loc->is_clockwise){
		    anki_s_uturn(h);
		    fprintf(stderr, "U-turn\n");
		}

		// Apply policy decsion
		res = apply_policy_trainingmode(h, *camera_loc, 0.8, 0.5, 0.8);
		if (res < 0) { // changed speed
		    camera_loc->real_speed = - res;
		    res = 0;
		}
		
		// Check if lap finished
		laptime = is_car_finished();
		if (laptime > 2.){
		    fprintf(stderr, "    > Lap time: %.3f\n\n", laptime);
		    break;
		}

		// Next loop
		run_index += 1;
		usleep(control_update * 1000000);

		if (!kbint || res) {
			break;
		}

	    }
	    //save run
	    export_run(episode, "/home/cvml1/Code/TrainRuns/");
	    reset_run();
	    if (!kbint || res) {
		break;
	    }
	}
	// Save policy
	export_policy(nepisodes,  "/home/cvml1/Code/TrainRuns/");
    }


    /*
     * Close and disconnect
     */
    printf("\n   > Total time elapsed: %f\n", totaltime);
    printf("   > Minimum lap time: %f\n", minlaptime);
    exit_signal = 1;
    anki_s_close(h);
    pthread_join(camera, NULL);
    free(background);
    free(camera_loc);
    return 0;
}
