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
static int camera_index = 0;    //index for camera update
static int run_index = 0;       //index for global simulation update
static pthread_t camera;        // Thread running the camera detection
static AnkiHandle h;
camera_localization_t* camera_loc;          // Vehicle's position
camera_obst_localization_t* camera_obst;    // Obstacles position
shared_struct* background;                  // Background image
unsigned char** input_median;               // Store images for background computation
struct timeval lapstarttime;                // Start time for current time
static int exit_signal = 0;                 // 1 iff ctrl-C detected
static int starting_signal = 0;



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
    printf(KBLU "[Loc]" RESET " Segm (%03x, %03x) clock-wise: %i, [update: %i]\n", loc.segm, loc.subsegm, loc.is_clockwise, loc.update_time);
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
    init_centroids_list("CV/centroids_h40_v4.txt", verbose);

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
    FILE *f = fopen("/home/cvml1/Code/Images/default_background.txt", "rb");
    fread(background->data, IMAGE_SIZE, 1, f);
    fclose(f);


    // Additional Paramaters
    camera_index = 0;
    char filename[256];
    int next_bg_update = bg_start - bg_history;
    shared_struct* temp = (shared_struct*) malloc(sizeof(shared_struct));

    // Wait for starting signal
    while (!starting_signal) {};

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
	get_camera_loc(temp, camera_index + 1, verbose, car_color);

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
    float camera_update = 0.05;   // Update of the camera picture, in percent of seconds
    float control_update = 0.15;  // Update of the vehicle action, `` `` ``
    float background_update = 5;  // Update of the background, `` `` ``
    int background_start = 80;    // Index at which the background computation starts
    int background_history = 15;  // Number of images to use for median computation
    int nlap = 10;                // Number of laps before the car stops


    /*
     * Training parameters
     */
    int nepisodes = 101;
    int nsteps = 1000;
    float learning_rate = 0.4;
    float discount_factor = 0.95;
    float epsilon_decay = 0.8;

    /*
     * Read input parameters
     */
    if(argc<2){
	fprintf(stderr, "usage: %s car-name [train/test/background/debug] [hour] [minutes] [nbr of opponents]  [verbose]\n",argv[0]);
	exit(0);
    }
    const char* adapter = "hci0";
    const char* car_id  = get_car_mac(argv[1]);
    const char* car_color  = get_car_color(argv[1]);
    int hour_start = -1;
    int minute_start = -1;
    int mode = 0; // [0 train, 1 deterministic policy, 2 trained policy, 3 compute background, 4 debug]
    int opponents = 3;
    int verbose = 0;
    if (argc > 2) {
	mode = atoi(argv[2]);
	if (argc > 3) {
	    hour_start = atoi(argv[3]);
	    if (argc > 4) {
		minute_start = atoi(argv[4]);
	        if (argc > 5) {
	            opponents = atoi(argv[5]);
		    if (argc > 6) {
			verbose = 1;
		    }
	        }
	    }
	}
    }

    /*
     * Load thread for camera update and processing
     */
    gettimeofday(&lapstarttime, NULL);
    struct arg_struct camera_args = {car_color, 1000 * camera_update, 1000 * background_update, background_start, background_history, opponents, verbose};
    int ret = pthread_create (&camera, 0, update_camera_loc,  &camera_args);


    /*
     * Control vehicle's behaviour (run until ctrl-c)
     */

    // Init bluethooth and wait for connection successful
    fprintf(stderr, "Attempting connection to %s\n", car_id);
    h = anki_s_init(adapter, car_id, verbose);
    fprintf(stderr, "Connection successful\n");

    // Additional parameters
    run_index = 0;
    int res, lap;
    float laptime=-1.;
    float totaltime = 0.;
    float minlaptime = 50.;


    /*
     * ============================= 0. DEBUG Mode for detection - run several cars on track
     */
    if (mode == 4) {
     starting_signal = 1;
     res = anki_s_set_speed(h, 700, 5000);
     AnkiHandle h2 = anki_s_init(adapter, "EB:0D:D8:05:CA:1A", verbose);
     res = anki_s_set_speed(h2, 500, 5000);
     usleep(10*1000000);
     anki_s_close(h);
     anki_s_close(h2);
    }

    /*
     * ============================= 1. Compute default background
     */
    else if (mode == 3) {
        starting_signal = 1;
	printf("[Running...]\n");
        res = anki_s_set_speed(h, 700, 5000);
	sleep(10);
	printf("Storing background\n");
	// Export text	
        FILE *fid = fopen("/home/cvml1/Code/Images/default_background.txt", "wb");
        int res = fwrite(background, 1920*1200*3, 1, fid);
        fclose(fid);
	// Export ppm
        char PPMheader[32];
        snprintf(PPMheader, 31, "P6\n%d %d 255\n", 1696, 720);
        fid = fopen("/home/cvml1/Code/Images/default_background.ppm", "wb");
        res = fwrite(PPMheader, strlen(PPMheader), 1, fid);
        res = fwrite(background, 1920*1200*3, 1, fid);
        fclose(fid);
    }
    /*
     * ============================= 2. Policy mode
     */
    else if (mode == 1 || mode == 2) {
	// Initialize a deterministic policy
	if (mode == 1) {
	   init_det_one_car_policy();
        } 
	// Initialize trained policy
	else {
	    init_trained_policy("/home/cvml1/Code/TrainRuns/Training_NoRnd_ExploreSpeed/policy_table_70.txt");
	}

        // Starts at timestamp if given	
	time_t secs = time(0);
	struct tm *local = localtime(&secs);
	printf("[Waiting for start time....]\n");
        while ((hour_start  - local->tm_hour) > 0 || (minute_start - local->tm_min) > 0) {
          secs = time(0);
          local = localtime(&secs);
	}
     
	// Start
        starting_signal = 1;
	res = anki_s_set_speed(h, 1700, 5000);
	camera_loc->real_speed = 1700;

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
			while(!camera_loc->is_clockwise) {};
		fprintf(stderr, "U-turn\n");
	    }


	    // Apply deterministic policy decsion
	    res = apply_policy(h, *camera_loc);
	    if (res < 0) { // update real speed
		camera_loc->real_speed = - res;
		res = 0;
	    }

	    // Next step
	    run_index += 1;
	    usleep(control_update * 1000000);
	}
    }
    /*
     * ======================================== 3. Training mode
     */
    else if (mode == 0) {
	//Initialize
	init_totrain_onecar_policy(0.);
	//init_trained_policy("/home/cvml1/Code/TrainRuns/Training_1801/policy_table_301.txt");
	export_policy(0,  "/home/cvml1/Code/TrainRuns/");
	export_policy_table(0,  "/home/cvml1/Code/TrainRuns/");

	//Start
        starting_signal = 1;
	res = anki_s_set_speed(h, 1200, 2000);
	camera_loc->real_speed = 1200;
	int episode, step;

	// Start episode
	for (episode = 0; episode < nepisodes; episode++) {
	    fprintf(stderr, KMAG "----------------- EPISODE %d" RESET,episode);
    	    gettimeofday(&lapstarttime, NULL);

	    // Start run
	    for (step = 0; step < nsteps; step++) {
		fprintf(stderr, "-------- STEP %d ",step);
		// Display
		printf("\n");
		print_loc(h);
		print_camera_loc();

		// Check direction and perform uturn if false [wait until completion and restart]
		if(!camera_loc->is_clockwise){
		    anki_s_uturn(h);
			while(!camera_loc->is_clockwise) {};
		    fprintf(stderr, KMAG "U-turn\n" RESET);
		    run_index += 1;
		    usleep(1 * 1000000);
		    break;
		}

		// Apply policy decsion (.., .. , learning_rate, discount_factor, epsilondecay, with reward based on distance [1] or 0 reward [0])
		//res = apply_policy_trainingmode(h, *camera_loc, learning_rate, discount_factor, epsilon_decay, 1);
		res = apply_policy_trainingmode(h, *camera_loc, learning_rate, discount_factor, epsilon_decay, 0);
		if (res < 0) { // update real speed
		    camera_loc->real_speed = - res;
		    res = 0;
		}
		
		// Detect finished lap
		laptime = is_car_finished();
		if (laptime > 2.8){
		    fprintf(stderr, "    > Lap time: %.3f\n\n", laptime);
		    totaltime += laptime;
		    if (laptime < minlaptime) {
		        minlaptime = laptime;
		    }
		    // Update policy based on lap time
		    update_policy_afterlap(laptime, learning_rate, discount_factor, epsilon_decay);

		    // Next lap
		    run_index += 1;
		    usleep(control_update * 1000000);
		    break;
		}
	
		// Break if error
		if (!kbint || res) {
			break;
		}

		// Next step
		run_index += 1;
		usleep(control_update * 1000000);
	    }

	    // Save run
	    export_run(episode, "/home/cvml1/Code/TrainRuns/",laptime);
	    reset_run();
	    if (!kbint || res) {
		break;
	    }

	    if (episode>0&&episode%10==0){
		export_policy(episode,  "/home/cvml1/Code/TrainRuns/");
		export_policy_table(episode,  "/home/cvml1/Code/TrainRuns/");
	    }
	}

	    if (kbint && !res) {
	// Save policy
	export_policy(nepisodes, "/home/cvml1/Code/TrainRuns/");
	export_policy_table(nepisodes, "/home/cvml1/Code/TrainRuns/"); }
    }


    /*
     * Close and disconnect
     */
    printf("\n   > Total time elapsed: %f\n", totaltime);
    printf("   > Minimum lap time: %f\n", minlaptime);
    printf("Close\n");
    exit_signal = 1;
    anki_s_close(h);
    pthread_join(camera, NULL);
    free(background);
    free(camera_loc);
    return 0;
}
