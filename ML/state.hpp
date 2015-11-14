#include "../examples/simple-c-interface/anki-simplified.h"
#include <map>
#include <iostream>
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

//Mother dummy  class should not be used directly
class Action {
public:
    int apply(AnkiHandle h) {return 0;};

    //Ovveride < to use State as a key
    bool operator <(const Action& rhs) const {
        return 1;
    }
};

// Set speed
class ActionSpeed: public Action {
private:
    float speed;
    float accel;
public:
    // Constructor
    ActionSpeed(float speed_, float accel_) {
	speed = speed_; 
	accel = accel_;
    };
    //Function 
    int apply(AnkiHandle h) {	
	anki_s_set_speed(h, speed, accel);
	std::cout << "DEBUG: Set speed " << speed << "\n";
	return speed;
    };
    bool operator <(const ActionSpeed& rhs) const {
        return speed < rhs.speed && accel < rhs.accel;
    }
};

// Set lane
class ActionLane: public Action {
private:
    float offset;
    float speed;
    float accel;
public:
    //Constructor
    ActionLane(float offset_, float speed_, float accel_) {
	offset = offset_; 
	speed = speed_; 
	accel = accel_;
    };
    //Functions
    int apply(AnkiHandle h) { 
	anki_s_change_lane(h, speed, accel, offset);
	std::cout << "DEBUG: Change lane with offset " << offset << "\n";
	return 0;
    };
    bool operator <(const ActionLane& rhs) const {
        return offset < rhs.offset && speed < rhs.speed && accel < rhs.accel;
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
    int max_speed_straight;
    int max_speed_curve;
    int accel;
    int straight_lane;
    int curve_lane;
    int laneoffset;
    float curve_threshold;
public:
    DetOneCarPolicy(int max_speed_straight_ = 1800, int max_speed_curve_ = 1400,  int accel_ = 2000, int straight_lane_ = 2, int curve_lane_ = 1, int laneoffset_ = 1000, float curve_threshold_ = 0.85) {
	max_speed_straight = max_speed_straight_;
	max_speed_curve = max_speed_curve_;
	accel = accel_;
	straight_lane = straight_lane_;
	curve_lane = curve_lane_;
	laneoffset = laneoffset_;
	curve_threshold = curve_threshold_;
    }
    //@Override
    Action get_next_action(State s);
};

