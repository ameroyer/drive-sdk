/*
 * Main File for controlling the car vehicle
 * Modified version of vehicle tool with threaded execution and read from file instead of stdin
 */


#include "examples/simple-c-interface/anki-simplified.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


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
   // Example: Take picture
   GrabImagesFromSharedMemory(1);  

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
    anki_s_close(h);
    return 0;
}
