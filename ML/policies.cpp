#include "policies.hpp"
#include "state.hpp"
#include <iostream>
#include <fstream>

static Policy pi;
static DetOneCarPolicy pidet;
static int training = 0;
static std::vector<State> states_list;
static std::vector<Action> actions_list;
static State previous_state;
static Action previous_action;
static std::vector<State> run;
static std::vector<Action> run_actions;

/**
 * Export functions
 */	
void reset_run() {
    //reset run to an empty list
    run.clear();
}

void export_run(int current_run, char* output_dir) {
    //export current run to a file 
    std::ostringstream ss;  
    ss << output_dir << "run_" << current_run << ".txt";
    std::ofstream f;
    f.open (ss.str().c_str());
    int i;
    for(i = 0; i < run.size(); i++) {
	f << run[i].to_string() << "\n";
	f << run_actions[i].to_string() << "\n";
    }
    f << "\n";
    f.close();
}


void export_policy(int current_episode, char* output_dir) {
    //Export policy pi in a file
    std::ostringstream ss;  
    ss << output_dir << "policy_" << current_episode << ".txt";
    std::ofstream f;
    f.open (ss.str().c_str());
    f << pi.to_string();
    f.close();
}


/**
 * Functions for application / testing phase
 */

// Init a deterministic fixed policy (see states.cpp)
void init_det_one_car_policy() {
    pidet = DetOneCarPolicy();
}
    


// Load a policy stored in a file
void init_trained_policy(char* filename) {
    pi = Policy(); //TODO
}



int apply_policy(AnkiHandle h, camera_localization_t c) {
    //Choose best action from policyand apply
    if(c.centroid>=0){
	State s(centroids_list[c.centroid], c.real_speed);
	return pidet.get_next_action(s).apply(h);
    }
    return 0;
}


/**
 * Functions for training a policy
 */

// Init an empty policy for training in a situation with only one car
void init_totrain_onecar_policy() {
    pi = Policy();
    
    int speed_values[] = {1000, 1500};
    int offset_values[] = {1000, -1000};
    float lanespeed = 200;
    float accel = 2000;

    //Define states
    for(std::vector<Centroid>::iterator it = centroids_list.begin(); it != centroids_list.end(); ++it) {
	for(int i = 0; i < sizeof(speed_values) / sizeof(int); i++) {
	    states_list.push_back(State(*it, speed_values[i]));	
	}
    }
    // Define actions
    int i;
    for (i = 0; i < 2; i ++) {
	actions_list.push_back(Action(speed_values[i], accel));
    }

    for (i = 0; i < 2; i ++) {
	actions_list.push_back(Action(offset_values[i], lanespeed, accel));
    }

    // Set arbitray q-vqlues
    for(std::vector<Action>::iterator ita = actions_list.begin(); ita != actions_list.end(); ++ita) {
	for(std::vector<State>::iterator its = states_list.begin(); its != states_list.end(); ++its) {
	    pi.set_score(*its, *ita, 0.);
	}
    }
}


// Define rewards for policy
// R(s, a, t)
float reward_onecar_policy(State s, Action a, State t) {

    // Set fixed rewards for important checkpoints
    //If straight part
    if (s.get_stra() < 0.5 && t.get_stra() > 0.5) {
	// Accelerate and going to inside is good
	if ((a.get_type() == 1 && a.get_speed() > s.get_speed()) || (a.get_type() == 2 && s.get_lane() < 3 && a.get_offset() < 0)) {
	    return + 100;
	} else if ((a.get_type() == 1 && a.get_speed() <= s.get_speed()) || (a.get_type() == 2 && s.get_lane() == 3))  {
	    return - 100;
	}
    } // In curves
    else  {
	// Decelerate and going to middle
	if ((a.get_type() == 1 && a.get_speed() < s.get_speed()) || (a.get_type() == 2 && s.get_lane() > 2 && a.get_offset() > 0)) {
	    return + 100;
	} else if ((a.get_type() == 1 && a.get_speed() >= s.get_speed()) || (a.get_type() == 2 && s.get_lane() != 2 && a.get_offset() != 0))  {
	    return - 100.;
	}
    } 

    // Default is number of vertical segments travelled
    return get_distance_vseg(s.get_car(), t.get_car());
    
}


// Apply best/epsilon-best action and update policy based on previous state
int apply_policy_trainingmode(AnkiHandle h, camera_localization_t c, float learning_rate, float discount_factor, float epsilon) {
    // Get state
    State s(centroids_list[c.centroid], c.real_speed);
    run.push_back(s);

    //If not first state, update policy
    if (run.size() > 0) {
	pi.set_score(previous_state, previous_action, pi.get_score(previous_state, previous_action) * (1. - learning_rate) + learning_rate * discount_factor * pi.get_best_score(s) + learning_rate * reward_onecar_policy(previous_state, previous_action, s));
    }

    //Choose best action (greedy)
    //TODO epsilon greedy
    previous_action = pi.get_next_action(s);
    previous_state = s;
    run_actions.push_back(previous_action);
    return previous_action.apply(h);
}

