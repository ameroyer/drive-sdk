#ifndef GETCAMERA_H
#define GETCAMERA_H

#ifdef __cplusplus
extern "C" {
#endif



    /**
     * Camera image type
     */
#define IMAGE_SIZE 1920*1200*3
    typedef struct {
	unsigned char data[IMAGE_SIZE];
	long count;
    } shared_struct;


    /**
     * Position from camera
     */
    typedef struct camera_localization {
	float x ;    /// x coordinate
	float y ;    /// y coordinate
	int centroid;  // Index of corresponding discretized state
	float size; //blob diameter
	float direction[2]; //Direction vector (ie new position - old position)
	float speed;
	float real_speed;
	int update_time ; /// Last update time (is increased every time we get an update from the car)
	int success;
    } camera_localization_t
    ;

    /**
     * Position from camera for other cars
     */
    typedef struct camera_obstacles_localization {
	float obst[5 * 3]; // 5 other cars at most, 3 fields by detected object (x, y, ray)
	int centroids[5]; // Corresponding discretized states
	int update_time ; /// Last update time (is increased every time we get an update from the car)
	int found;
	int total;
    } camera_obst_localization_t
    ;

    extern struct timeval lapstarttime;
    extern camera_localization_t* camera_loc;
    extern camera_obst_localization_t* camera_obst;
    extern unsigned char** input_median;
    extern shared_struct* background;

    /**
     * Compute the median image from a sequence of images
     */
    void compute_median(int nfiles, unsigned char** array, shared_struct* result);
    void compute_median_multithread(int nfiles, int nthread);

    /*
     * Init states list
     */
    void init_centroids_list(char* filename, int verbose);
    /**
     * Init OpenCV Blob detection
     */
    void init_blob_detector();

    /**
     * Return locations of object detected on the camera image
     */
    void get_camera_loc(shared_struct* shm, int index, int verbose, const char* car_color);
    void get_camera_lock_dead_reckon(int time, float result[2]);
    float is_car_finished();

    /**
     * Substract two camera images
     */
    void sub(shared_struct* im, shared_struct* bg, shared_struct* result, int min);
    void sub_pos(shared_struct* im, shared_struct* bg, shared_struct* result, int min);

    /**
     * Export image in a readable PPM file
     */
    void export_ppm(char* filename, const int width, const int height, shared_struct* shm);

    /**
     * Export image without PPm header as a txt file
     */
    void export_txt(char* filename, const int width, const int height, shared_struct* shm);


#ifdef __cplusplus
}
#endif

#endif
