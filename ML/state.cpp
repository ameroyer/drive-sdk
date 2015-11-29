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

int Centroid::get_vseg() {
    return vseg;
}

float Centroid::get_stra() { 
    return straight;
}

float Centroid::get_distance_squared(float a, float b) {
    return (x - a) * (x - a) + (y - b) * (y - b);
}

std::vector<Centroid> centroids_list;

void get_centroid_direction(int id, float direction[2], int clockwise) {
    Centroid centroid = centroids_list[id];
    Centroid nxt_centroid;
    if (clockwise) {
	nxt_centroid = centroids_list[(id - 4 + centroids_list.size()) % centroids_list.size()];
    } else {
	nxt_centroid = centroids_list[(id + 4) % centroids_list.size()];
    }
    float dst = sqrt(centroid.get_distance_squared(nxt_centroid.get_x(), nxt_centroid.get_y()));
    direction[0] = (nxt_centroid.get_x() - centroid.get_x()) / dst;
    direction[1] = (nxt_centroid.get_y() - centroid.get_y()) / dst;
}

float get_distance_vseg(Centroid b, Centroid c) {
    // if cross start line
    if ((c.get_id() > floor(centroids_list.size() * 0.75) && b.get_id() < floor(centroids_list.size() * 0.25)) || (b.get_id() > floor(centroids_list.size() * 0.75) && c.get_id() < floor(centroids_list.size() * 0.25))) {	    
	return abs(c.get_id() - b.get_id() + centroids_list.size()) / 4 + 1;
    } else {
	return abs(c.get_id() - b.get_id()) / 4 + 1;
    }
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

Action Policy::get_random_action(State s) {
    int index = rand() % qscores[s].size();
    for(std::map<Action, float>::iterator iterator = qscores[s].begin(); iterator != qscores[s].end(); iterator++) {
	index -= 1;
	if (index <= 0) {
	   return iterator->first;
	}
    }
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

    // Do nothing if last action
    int step = 2;
    float previous = centroids_list[(s.get_carid() + 4 * step)% centroids_list.size()].get_stra();
    float after = centroids_list[(centroids_list.size() + s.get_carid() - 4 * step)% centroids_list.size()].get_stra();
    float now = s.get_stra();
    printf("centroid : %d, lane %d, vseg: %d, curv: %f \n", s.get_carid(), s.get_car().get_lane(), s.get_car().get_vseg(),now);
    // In straight part, go on inside lane and increase speed
    if (now < curve_threshold && after >= curve_threshold) {
	if (s.get_speed() != max_speed_straight) {
	    last_action_type = 1;
	    return Action(max_speed_straight, accel);
	}
	/*if (s.get_lane() < straight_lane && last_action_type!=2) {
	  last_action_type = 2;
	  return Action(-laneoffset, 100, accel);
	  }*/
    }
    // In curve parts, go on middle lane and slightly decrease speed
    else if (now > curve_threshold && after <= curve_threshold) {
	if (s.get_speed() != max_speed_curve) {
	    last_action_type = 1;
	    return Action(max_speed_curve, accel);
	}
	/*if (s.get_lane() > curve_lane  && last_action_type!=2) {
	  last_action_type = 2;
	  return Action(laneoffset, 100, accel);
	  }*/
    }	
    last_action_type = 0;
    return Action();
};
