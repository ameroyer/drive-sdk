#ifdef __cplusplus
extern "C" {
#endif
#include "../CV/get_camera.hpp"
#include "../examples/simple-c-interface/anki-simplified.h"

    /*
     * Init the policy for deterministic actions with only one car on track
     */
    void init_det_one_car_policy();
    
    /*
     * Apply policy to car given the state described by camera_localization c
     */
    int apply_policy(AnkiHandle h, camera_localization_t c); //apply deterministic policy
    int apply_policy_trainingmode(AnkiHandle h, camera_localization_t c, float learning_rate, float discount_factor, float epsilondecay, int distance_reward);
    void update_policy_afterlap(float laptime, float learning_rate, float discount_factor, float epsilondecay);

    /*
     * Functions to store and load training results
     */
    void init_trained_policy(char* filename);
    void init_totrain_onecar_policy(float initepsilon);
    void reset_run();
    void export_run(int current_run, char* output_dir, double laptime);
    void export_policy(int current_episode, char* output_dir);
    void export_policy_table(int current_episode, char* output_dir);
    int go_inside(AnkiHandle h, camera_localization_t c); 


#ifdef __cplusplus
}
#endif
