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
 * Compute the median image from a sequence of images
 */
void compute_median(int nfiles, unsigned char** array, shared_struct* result);


/**
 * Init OpenCV Blob detection
 */
void init_blob_detector();

/**
 * Return locations of object detected on the camera image
 */
void get_camera_loc(shared_struct* shm, const int width, const int height);

/**
 * Substract two camera images
 */
void sub(shared_struct* im1, shared_struct* im2, shared_struct* result);

/**
 * Substract two camera images with additional binary thresholding
 */
void sub_thres(shared_struct* im1, shared_struct* im2, shared_struct* result, int threshold);

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
