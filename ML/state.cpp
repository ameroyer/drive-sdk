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

    // In straight part, go on inside lane and increase speed
    if (s.get_stra() > curve_threshold) {
	if (s.get_lane() > straight_lane) {
	    return ActionLane(laneoffset, max_speed_straight, accel);
	}
	if (s.get_speed() < max_speed_straight) {
	    return ActionSpeed(max_speed_straight, accel);
	}
    }
    else {
	if (s.get_lane() > curve_lane) {
	    return ActionLane(- laneoffset, max_speed_curve, accel);
	}
	if (s.get_lane() < curve_lane) {
	    return ActionLane(laneoffset, max_speed_curve, accel);
	}
	if (s.get_speed() < max_speed_curve) {
	    return ActionSpeed(max_speed_curve, accel);
	}
    }	
};
