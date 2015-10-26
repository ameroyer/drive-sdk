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
    int background_start;
    int history;
};


void update_camera_loc(void* aux) {
    // Read arguments
    struct arg_struct *args = (struct arg_struct *)aux;
    int img_update = args->update;
    int bg_history = args->history;
    int bg_start = args->background_start;
    int bg_update = args->background_update;

    // Shared memory
    int shmid;
    key_t key;
    shared_struct *shm;
    key = 192012003; // 1920x1200x3
    const int width = 1696;
    const int height = 720;

    if ((shmid = shmget(key, sizeof(shared_struct), 0666)) < 0) { perror("shmget"); exit(1); }
    if ((shm = (shared_struct*)shmat(shmid, NULL, 0)) == (shared_struct *) -1) { perror("shmat"); exit(1); }

    //Init arrays for saving past images
    char** saved_imgs;
    saved_imgs = (char**) malloc(sizeof(char*) * bg_history);
    int i;
    for (i = 0; i < bg_history; i++) {
	saved_imgs[i] = malloc(sizeof(char) * IMAGE_SIZE);
    }

    // Paramaters
    char saved_img[bg_history][256];
    int index = 0;
    char filename[256];
    int next_bg_update = bg_start - bg_history;
    shared_struct* temp = (shared_struct*) malloc(sizeof(shared_struct));

    // Update location until receiving exit signal
    while (!exit_signal) {
	fprintf(stderr, "Index: %d - Next bg update: %d - Current saving index: %d\n", index, next_bg_update  - next_bg_update%bg_update + bg_start, (next_bg_update + bg_history - bg_start) % bg_update);
	//Store images for background update
	if ((next_bg_update - bg_start) % bg_update  == 0) {
	    fprintf(stderr, "Update Background\n");
	    next_bg_update += bg_update - bg_history;
	    compute_median(bg_history, saved_imgs, background);
	    export_ppm(filename, width, height, background);
	}
	// Compute differential image in temp
	temp->count = index + 1;
	//sub(shm, background, temp);
	sub_thres(shm, background, temp, 150);
	export_ppm(filename, width, height, temp);
	get_camera_loc(temp, width, height);

	// If needed, save current image for background update
	if (index == next_bg_update) {
	    memcpy(saved_imgs[(next_bg_update + bg_history - bg_start) % bg_update], shm->data, IMAGE_SIZE);
	    //export_txt(&saved_img[(next_bg_update + bg_history - bg_start) % bg_update], width, height, shm);
	    next_bg_update += 1;
	}
	   
	index += 1;
	usleep(img_update * 1000);
    }
    
    
    /* Detach local  from shared memory */
    if ( shmdt(shm) == -1) { perror("shmdt"); exit(1); } 
    // Free
    for (i = 0; i < bg_history; i++) {
	free(saved_imgs[i]);
    }
    free(saved_imgs);
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
     * Read parameters and Initialization
     */
    if(argc<2){
	fprintf(stderr, "usage: %s car-name [adaptater] [verbose]\n",argv[0]);
	exit(0);
    }
    const char* adapter = "hci0";
    const char* car_id  = get_car_mac(argv[1]);
    if (argc > 2) {
	adapter = argv[2];
    }
    init_blob_detector();

    /*
     * Load thread to Update picture every second and process it  
     */
    // Set arguments
    struct arg_struct args;
    args.vehicle_color = "grey";
    args.update = 100; //time in milliseconds
    args.background_update = 1000;
    args.background_start = 21;
    args.history = 15;

    // Load default background
    background = (shared_struct*) malloc(sizeof(shared_struct));
    background->count = 0;
    FILE *f = fopen("/home/cvml1/Code/Images/default_background.txt", "rb");
    fread(background->data, IMAGE_SIZE, 1, f);
    fclose(f);

    // Launch thread
    pthread_t camera;
    int ret = pthread_create (&camera, 0, (void*)update_camera_loc,  &args);
    
    /*
     * Init vehicle's connection
     */
    // Init bluethooth and wait for connection successful
    fprintf(stderr, "Attempting connection to %s\n", car_id);
    AnkiHandle h = anki_s_init(adapter, car_id, argc>3);
    while(!anki_s_is_connected(h)) {};
    fprintf(stderr, "Connection successful\n");

    // Send commands
    anki_s_set_speed(h,600,20000);
    print_loc(h);
    
    /*
      int i;
      //for(i=0; i<10; i++){ usleep(200000); anki_s_change_lane(h,-40,100,1000); }  
      for(i=0; i<10; i++){ usleep(200000);  print_loc(h);  }
      anki_s_set_speed(h,500,20000);
      for(i=0; i<10; i++){ usleep(200000);  print_loc(h);  }
      anki_s_change_lane(h,-50,100,1000);
      for(i=0; i<10; i++){ usleep(200000);  print_loc(h);  }
      anki_s_set_speed(h,0,20000); */
    
    // Test
    sleep(30);

    //TODO bloc fina; + interrupt if disconnected
    // Disconnect
    anki_s_close(h);
    exit_signal = 1;
    pthread_join(camera, NULL);
    free(background);
    return 0;
}
