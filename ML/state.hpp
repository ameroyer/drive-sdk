#include "../examples/simple-c-interface/anki-simplified.h"
#include <map>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>



/*
 * Class centroid
 */
class Centroid {
public:
    //Constructors
    Centroid() {};
    Centroid(int id_, float x_, float y_, float straight_, int vseg_, int lane_, int startline_) {
	id = id_; 
	x = x_; 
	y = y_; 
	straight = straight_; 
	vseg=vseg_; 
	startline=startline_; 
	lane = lane_; 
    };
    //Getters
    int get_id();
    float get_x();
    float get_y();
    int get_start();
    int get_lane();
    float get_stra();

    //get distance between (a,b) and centroid
    float get_distance_squared(float a, float b);

private:
    int id;
    float x;
    float y;
    int vseg;
    int lane;
    int startline;
    float straight;
};


extern std::vector<Centroid> centroids_list;

/*
 * Class describing a state in the policy
 */ 
class State {
public:
    //Constructors
    State() {};
    State(Centroid car_,float speed_) {
	car=car_; 
	speed=speed_;
	carid = car.get_id();
    }
    //Getters
    Centroid get_car();
    int get_lane();
    float get_speed();
    float get_stra();
	int get_carid();
    
    //Ovveride < to use State as a key
    bool operator <(const State& rhs) const {
	return carid < rhs.carid && speed < rhs.speed;
    }

private:
    Centroid car;
    float speed;
    int carid;
    //Centroid* opponents; // for later
};


/*
 * Actions
 */

class Action {
protected:
    int type;
    float offset;
    float speed;
    float accel;
public:
    //Constructors

    //type 0: null action
    Action() {type = 0;}

    //type1: speed action
    Action(float speed_, float accel_) {
	speed = speed_; 
	accel = accel_;
	type = 1;
    };

    //type2: accel action
    Action(float offset_, float speed_, float accel_) {
	offset = offset_; 
	speed = speed_; 
	accel = accel_;
	type = 2;
    };
    int apply(AnkiHandle h) {
	if (type == 0) {
	    std::cerr << "DEBUG: No action\n";
	    return 0;
	} else if (type == 1) {
	    anki_s_set_speed(h, speed, accel);
	    std::cerr << "DEBUG: Set speed " << speed << "\n";
	    return speed;
	} else {
	    anki_s_change_lane(h, offset, speed, accel);
	    std::cout << "DEBUG: Change lane with offset " << offset << "\n";
	    return 0;
	}
    };

    //Ovveride < to use Actione as a key
    bool operator <(const Action& rhs) const {
        return (type < rhs.type)||(type == 1 && rhs.type == 1 && speed < rhs.speed && accel < rhs.accel)||(type == 2 && rhs.type == 2 && speed < rhs.speed && accel < rhs.accel && offset < rhs.offset);
    }
};


/*
 * General policy class
 */
class Policy {
private:
    std::map<State, std::map<Action, float> > qscores;
public:
    Policy() {};
    Action get_next_action(State s);
    float get_best_score(State s);
    void set_score(State s, Action a, float value);
    float get_score(State s, Action a);
};

/*
 * One car deterministic policy
 */
class DetOneCarPolicy: public Policy {
private:
    float max_speed_straight;
    float max_speed_curve;
    float accel;
    int straight_lane;
    int curve_lane;
    float laneoffset;
    float curve_threshold;
    int last_action_type; // "cool down": Use to forbid two change lane actions in a row
public:
    // Constructor
    DetOneCarPolicy(float max_speed_straight_ = 1500., float max_speed_curve_ = 1100.,  float accel_ = 2000., int straight_lane_ = 2, int curve_lane_ = 1, float laneoffset_ = 1000., float curve_threshold_ = 0.8) {
	max_speed_straight = max_speed_straight_;
	max_speed_curve = max_speed_curve_;
	accel = accel_;
	straight_lane = straight_lane_;
	curve_lane = curve_lane_;
	laneoffset = laneoffset_;
	curve_threshold = curve_threshold_;
	last_action_type = 0;
    }
    //@Override
    Action get_next_action(State s);
};

