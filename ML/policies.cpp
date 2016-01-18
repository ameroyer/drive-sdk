#include "policies.hpp"
#include "state.hpp"
#include <iostream>
#include <fstream>
#include <time.h>
#include <stdlib.h>
#include <math.h>

static Policy pi;
static DetOneCarPolicy pidet;
static int training = 0;
static std::vector<State> states_list;
static std::vector<Action> actions_list;
static State previous_state;
static Action previous_action;
static std::vector<State> run;
static std::vector<Action> run_actions;
static float epsilon = 1.0;


/**
 * Export functions
 */	
void reset_run() {
    //reset run to an empty list
    run.clear();
}


void export_run(int current_run, char* output_dir, double laptime) {
    //export run to a file 
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
    f << "Laptime " << laptime << "\n";
    f.close();
}



void export_policy(int current_episode, char* output_dir) {
    //Export policy pi in a file [readbale format]
    std::ostringstream ss;  
    ss << output_dir << "policy_" << current_episode << ".txt";
    std::ofstream f;
    f.open (ss.str().c_str());
    f << pi.to_string();
    f.close();
}

void export_policy_table(int current_episode, char* output_dir) {
    //Export policy pi in a file [table format]
    std::ostringstream ss;  
    ss << output_dir << "policy_table_" << current_episode << ".txt";
    std::ofstream f;
    f.open (ss.str().c_str());
    f << pi.to_string_table();
    f.close();
}



/**
 * Deterministic policy
 */

// Init a deterministic fixed policy (see states.cpp)
void init_det_one_car_policy() {
    pidet = DetOneCarPolicy();
}


int apply_policy(AnkiHandle h, camera_localization_t c) {
    //Choose best action from policy and apply
    if(c.centroid>=0){
	State s(centroids_list[c.centroid], c.real_speed);
	return pidet.get_next_action(s).apply(h);
    }
    return 0;
}
    

/* 
 Load a policy stored in a file
*/
//TODO: fix first line for export and import (state 0)
void init_trained_policy(char* filename) {
    pi = Policy(); 
    
    int speed_values[] = {1200, 1700};
    int offset_values[] = {1000, -1000};
    float lanespeed = 200;
    float accel = 2000;
    epsilon=0.1;
    
    //Define states (same as init_totrain_onecar_policy!)
    for(std::vector<Centroid>::iterator it = centroids_list.begin(); it != centroids_list.end(); ++it) {
	for(int i = 0; i < sizeof(speed_values) / sizeof(int); i++) {
	    states_list.push_back(State(*it, speed_values[i]));	
	}
    }
    // Define actions (same as init_totrain_onecar_policy!)
    int i;
    actions_list.push_back(Action());
    for (i = 0; i < 2; i ++) {
	actions_list.push_back(Action(speed_values[i], accel));
    }
    for (i = 0; i < 2; i ++) {
	actions_list.push_back(Action(offset_values[i], lanespeed, accel));
    }
    
    std::vector<std::vector<float> > qvalue_table;
    std::ifstream infile(filename);
    std::string line;
    int linenr=0;
    while (std::getline(infile, line))
	{
		std::vector<float> qvalues_line;
	    std::istringstream iss(line);
		float q;
		int count=0;
		while (iss.good()) // while the stream is not empty
        {
            iss >> q; //get data from the stream. This will give you only up until the next whitespace.
                        //Get numbers, strings, whatever you want.
            qvalues_line.push_back(q);
            count++;
        }
        if(count>2){
			qvalue_table.push_back(qvalues_line);
		}else{
			printf("State0line ");
		}
	    linenr++;
	}
	
	// Set q-values that were loaded from file    
	linenr=0;  
	for(std::vector<State>::iterator its = states_list.begin(); its != states_list.end(); ++its) {
		std::vector<float> qvalues_state=qvalue_table.at(linenr);
		int actionnr=0;
		for(std::vector<Action>::iterator ita = actions_list.begin(); ita != actions_list.end(); ++ita) {
			if(qvalues_state.size()>0){
				//printf("\n %f", qvalues_state.at(actionnr));
				pi.set_score(*its, *ita, qvalues_state.at(actionnr));
				actionnr++;
			}
		}
		linenr++;
    }
}





/**
 * Functions for training a policy
 */

// Init an empty policy for training in a situation with only one car
void init_totrain_onecar_policy(float initepsilon) {
    pi = Policy();
    epsilon = initepsilon;
    int speed_values[] = {1200, 1700};
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
    actions_list.push_back(Action());
    
    for (i = 0; i < 2; i ++) {
	actions_list.push_back(Action(speed_values[i], accel));
    }

    for (i = 0; i < 2; i ++) {
	actions_list.push_back(Action(offset_values[i], lanespeed, accel));
    }
    
    // Set initial qvalues
    for(std::vector<Action>::iterator ita = actions_list.begin(); ita != actions_list.end(); ++ita) {
	for(std::vector<State>::iterator its = states_list.begin(); its != states_list.end(); ++its) {
            //If straight part
            if (its->get_stra() > 0.5) {
	        // Accelerate and going to inside is good
	        if ((ita->get_type() == 1 && ita->get_speed() > its->get_speed()) || (ita->get_type() == 2 && its->get_lane() < 3 && ita->get_offset() < 0)) {
	            pi.set_score(*its, *ita, 0.);
	        } else if ((ita->get_type() == 1 && ita->get_speed() <= its->get_speed()) || (ita->get_type() == 2 && its->get_lane() == 3))  {
	            pi.set_score(*its, *ita, -2.);
	        } else {
		    pi.set_score(*its, *ita, 0.);
                }
            } // In curves
            else if (its->get_stra() <= 0.5)  {
	        // Decelerate and going to middle
	        if ((ita->get_type() == 1 && ita->get_speed() < its->get_speed()) || (ita->get_type() == 2 && its->get_lane() > 2 && ita->get_offset() > 0)) {
		    pi.set_score(*its, *ita, 0.);
	        } else if ((ita->get_type() == 1 && ita->get_speed() >= its->get_speed()) || (ita->get_type() == 2 && its->get_lane() != 2 && ita->get_offset() != 0))  {
	            pi.set_score(*its, *ita, -2.);
	        } else {
	            pi.set_score(*its, *ita, 0.);
                }	
            } 
	    // Constraint  the car to stay in the track
	    if (ita->get_type() == 2 && ( (ita->get_offset() < 0 && its->get_lane() == 3) || (ita->get_offset() > 0 && its->get_lane() == 0) ) ) {
		pi.set_score(*its, *ita, -10.);
	    }  
	    // Do not set speed to current speed (= useless)
	    else if (ita->get_type() == 1 && (ita->get_speed() == its->get_speed())) {
		pi.set_score(*its, *ita, -10.);
	    } 
	}
    }
}


// Define rewards for policy
// R(s, a, t)
// Reward based on the distance travelled in a fixed time
float distance_reward_onecar_policy(State s, Action a, State t) {

    // Constraint  the car to stay in the track
    if (a.get_type() == 2 && ( (a.get_offset() < 0 && s.get_lane() == 3) || (a.get_offset() > 0 && s.get_lane() == 0) ) ) {
	return - 1000.;
    }  
    // Do not set speed to current speed (= useless)
    else if (a.get_type() == 1 && (a.get_speed() == s.get_speed())) {
	return - 1000.;
    }

    float r = get_distance_vseg(s.get_car(), t.get_car(), 1);
    if (r < 0) {
	    r = -100.;
	}
    return r;
    
}

// 0-reward (except forbidden state)
float zero_reward_onecar_policy(State s, Action a, State t) {

    // Constraint  the car to stay in the track
    if (a.get_type() == 2 && ( (a.get_offset() < 0 && s.get_lane() == 3) || (a.get_offset() > 0 && s.get_lane() == 0) ) ) {
	return - 1000.;
    }  
    // Do not set speed to current speed (= useless)
    else if (a.get_type() == 1 && (a.get_speed() == s.get_speed())) {
	return - 1000.;
    }

    return 0;    
}


// Apply best/epsilon-best action and update policy based on previous state
int apply_policy_trainingmode(AnkiHandle h, camera_localization_t c, float learning_rate, float discount_factor, float epsilondecay, int distance_reward) {
    // Get state
    State s(centroids_list[c.centroid], c.real_speed);
    run.push_back(s);

    //If not first state, update policy
    if (run.size() > 0) {

	int reward = 0;
	if (distance_reward) {
	  reward = distance_reward_onecar_policy(previous_state, previous_action, s);
	} else {
	  reward = zero_reward_onecar_policy(previous_state, previous_action, s)
	}

	pi.set_score(previous_state, previous_action, pi.get_score(previous_state, previous_action) * (1. - learning_rate) + learning_rate * discount_factor * pi.get_best_score(s) + learning_rate * reward);
    }

    fprintf(stderr, "Reward %f", reward);

    //Choose best action (epsilon greedy with decay)  
    if (epsilon > 0.1) {
	epsilon *= epsilondecay;
    } else {
	epsilon = 0.1;
    }
    float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    if (r < epsilon) {
	previous_action = pi.get_random_action(s);        
    } else {
    	previous_action = pi.get_next_action(s);
    }

    // Store state and action
    previous_state = s;
    run_actions.push_back(previous_action);
    return previous_action.apply(h);
}


// Update the policy value by backpropagation once a whole lap is finished
void update_policy_trainingmode_afterlap(float laptime, AnkiHandle h, camera_localization_t c, float learning_rate, float discount_factor, float epsilondecay) {
    // Set laptime
    float reward = - laptime;
    fprintf(stderr, "Reward %f\n", reward);

    //update q-values for all steps of this run [Back propagation]
    int i;
    for(i = run.size() - 1; i >= 0; i--) {
       float new_score = learning_rate * reward; // + pi.get_score(run[i], run_actions[i]) * (1. - learning_rate) ;
       pi.set_score(run[i], run_actions[i], new_score);
    }
}


/*
// Apply best/epsilon-best action and update policy based on previous state
int apply_policy_trainingmode_afterlap(AnkiHandle h, camera_localization_t c, float learning_rate, float discount_factor, float epsilondecay) {
    // Get state
    State s(centroids_list[c.centroid], c.real_speed);
    run.push_back(s);

    //If not first state, update policy
    //if (run.size() > 0) {
//	pi.set_score(previous_state, previous_action, pi.get_score(previous_state, previous_action) * (1. - learning_rate) + learning_rate * discount_factor * pi.get_best_score(s) + learning_rate * reward_onecar_policy(previous_state, previous_action, s));
//    }
 //   fprintf(stderr, "Reward %f", reward_onecar_policy(previous_state, previous_action, s));
    //Choose best action (epsilon greedy with decay) 
    
    //check if lap is finished
    float thislaptime = get_laptime();
    float reward = -thislaptime;
	if (thislaptime > 2.5){
		fprintf(stderr, "Reward %f\n", reward);
		int i;
		for(i = 0; i < run.size(); i++) {
			//update q-values for all steps of this run
			float newscore=pi.get_score(run[i], run_actions[i]) * (1. - learning_rate) +  learning_rate * reward;
			pi.set_score(run[i],run_actions[i], newscore ); //correct?
			fprintf(stderr, "Step %d: score %f \n", i,newscore);
		}
	}
    
    if (epsilon > 0.1) {
			epsilon *= epsilondecay;
    } else {
	epsilon = 0.1;
    }
    float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    printf(" eps: %f ",epsilon);
    if (r < epsilon) { // might cause seg faults?
		printf(" random ");
	    previous_action = pi.get_random_action(s);        
    } else {
    	previous_action = pi.get_next_action(s);
    	//printf("next action type: %d\n",previous_action.get_type()); 
    }
    printf("\n");
    previous_state = s;
    run_actions.push_back(previous_action);
    return previous_action.apply(h);
}
*/
