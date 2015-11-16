#include "state.hpp"
#include <iterator>

// Centroid
int Centroid::get_id() { 
    return id;
}

float Centroid::get_x() {
    return x;
}

float Centroid::get_y() {
    return y;
}

int Centroid::get_start() { 
    return startline;
}

int Centroid::get_lane() { 
    return lane;
}

float Centroid::get_stra() { 
    return straight;
}

float Centroid::get_distance_squared(float a, float b) {
    return (x - a) * (x - a) + (y - b) * (y - b);
}


// State
Centroid State::get_car() {
    return car;
}

float State::get_speed() {
    return speed;
}

float State::get_stra() {
    return car.get_stra();
}

int State::get_lane() {
    return car.get_lane();
}

int State::get_carid() {
    return carid;
}


/*
 * General policies
 */

//find action that maximizes the score for state s
Action Policy::get_next_action(State s) {
    float max = 0;
    Action best;
    for(std::map<Action, float>::iterator iterator = qscores[s].begin(); iterator != qscores[s].end(); iterator++) {
	if (iterator->second > max) {
	    max = iterator->second;
	    best = iterator->first;
	}
    }
    return best;
}

float Policy::get_best_score(State s) {
    float max = 0;
    for(std::map<Action, float>::iterator iterator = qscores[s].begin(); iterator != qscores[s].end(); iterator++) {
	if (iterator->second > max) {
	    max = iterator->second;
	}
    }
    return max;
}

void Policy::set_score(State s, Action a, float value) {
    qscores[s][a] = value;
}

float Policy::get_score(State s, Action a) {
    return qscores[s][a];
}


/*
 * Deterministic One car policy
 */
Action DetOneCarPolicy::get_next_action(State s) {
    //int centid=s.get_car().get_id();
    //int speed1=1000;
    //if(centid>50){speed1=200;}
    //return ActionSpeed(speed1,2000);

    // Do nothing if last action
     int step = 1;
     int begin = 0;
     int cid = s.get_carid();
     float curv = 0;
	for (int i = begin; i <= begin + step; i ++) {
	curv += centroids_list[(centroids_list.size() + cid - 3 * i) % centroids_list.size()].get_stra();
	}
     curv /= (step + 1);
	//printf("curv: %f \n", curv);
    // In straight part, go on inside lane and increase speed
    if (curv > curve_threshold) {
	if (curv > 0.92 && s.get_lane() < straight_lane && last_action_type != 2) {
	    last_action_type = 2;
	    return Action(-laneoffset, 300, accel);
	}
	if (s.get_speed() != max_speed_straight) {
	    last_action_type = 1;
	    return Action(max_speed_straight, accel);
	}
    }
    // In curve parts, go on middle lane and slightly decrease speed
    else {
	if (curv > 0.65 && s.get_lane() > curve_lane && last_action_type != 2) {
	    last_action_type = 2;
	    return Action(laneoffset, 100, accel);
	}
	if (s.get_speed() != max_speed_curve) {
	    last_action_type = 1;
	    return Action(max_speed_curve, accel);
	}
    }	
    last_action_type = 0;
    return Action();
};
