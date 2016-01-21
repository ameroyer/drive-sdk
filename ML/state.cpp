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


// Return the direction vector of the car given the current centroid position and the rotation orientation of the car (clockwise or counter clockwise)
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


// Return the distance travelled by a car between centroid b and c, with a negative sign if going in the opposite direction of the race
// Clockwise indicate wether the RACE is clockwise or not
float get_distance_vseg(Centroid b, Centroid c, int clockwise) {

	//if clockwise rotation sense
	if (c.get_vseg() == b.get_vseg()) {
	   return 0;
	}
	if (clockwise) {
    		if (c.get_id() > floor(centroids_list.size() * 0.75) && b.get_id() < floor(centroids_list.size() * 0.25) ) {	    
			return (b.get_id() - c.get_id() + centroids_list.size()) / 4 + 1;
    		} else {
			return (b.get_id() - c.get_id()) / 4 + 1;
    		}
	} else {
    		if (b.get_id() > floor(centroids_list.size() * 0.75) && c.get_id() < floor(centroids_list.size() * 0.25)) {	    
			return (c.get_id() - b.get_id() + centroids_list.size()) / 4 + 1;
    		} else {
			return (c.get_id() - b.get_id()) / 4 + 1;
    		}
	}
}



/***
* State class
**/
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

// Find action that maximizes the score for state s
Action Policy::get_next_action(State s) {
    float max = -100;
    std::vector<Action> best;
    for(std::map<Action, float>::iterator iterator = qscores[s].begin(); iterator != qscores[s].end(); iterator++) {
	if (iterator->second > max) {
	    max = iterator->second;
	    best.clear();
	    best.push_back(iterator->first);
	} else if (iterator->second == max) {
	    best.push_back(iterator->first);
	}
    }

    //std::cout << "Best score: " << max <<  "\n"; //" and Action: " << best.at(0).to_string() << "\n";
    // If several actions with equal scores, choose at random
    if (best.size() == 0) {
	return best.at(0);
    } else {
	return best.at(rand() % best.size());
    }
}

// Return a random action for a given state
Action Policy::get_random_action(State s) { 
    int index = rand() % (qscores[s].size()); //+1 important so that change lane outside get selected too
    for(std::map<Action, float>::iterator iterator = qscores[s].begin(); iterator != qscores[s].end(); iterator++) {
	index --;
	if (index < 0) {
	    return iterator->first;
	}
    }
}

// Return the best score for a given state (maximum over actions)
float Policy::get_best_score(State s) {
    float max = 0;
    for(std::map<Action, float>::iterator iterator = qscores[s].begin(); iterator != qscores[s].end(); iterator++) {
	if (iterator->second > max) {
	    max = iterator->second;
	}
    }
    return max;
}

// Set the q value for a given (state, action) pair
void Policy::set_score(State s, Action a, float value) {
    qscores[s][a] = value;
}

// Retrieve the q value for a given (state, action) pair
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
