#include "../examples/simple-c-interface/anki-simplified.h"

// Centroid
class Centroid {
public:
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
    int get_id();
    float get_x();
    float get_y();
    int get_start();
    int get_lane();
    float get_stra();

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


// State for the policy
class State {
public:
    State() {};
    State(Centroid car_,float speed_) {
	car=car_; 
	speed=speed_;
    }
    Centroid get_car();
    int get_lane() {
	return car.get_lane();
    }
    float get_speed() {
	return speed;
    }
    float get_stra() {
	return car.get_stra();
    }

private:
    Centroid car;
    float speed;
    //Centroid* opponents; // for later
};



// Define a class for actions
class Action {
public:
    int apply(AnkiHandle h) {return 0;};
};

// Set speed
class ActionSpeed: public Action {
private:
    float speed;
    float accel;
public:
    ActionSpeed(float speed_, float accel_) {
	speed = speed_; 
	accel = accel_;
    };
    int apply(AnkiHandle h) {	
	anki_s_set_speed(h, speed, accel);
	return speed;
    };
};

// Set lane
//TODO cqncel lane ?
class ActionLane: public Action {
private:
    float offset;
    float speed;
    float accel;
public:
    ActionLane(float offset_, float speed_, float accel_) {
	offset = offset_; 
	speed = speed_; 
	accel = accel_;
    };
    int apply(AnkiHandle h) { 
	anki_s_change_lane(h, speed, accel, offset);
	return 0;
    };
};


// Define Policies
class Policy {
public:
    Action get_next_action(State s) {return Action();};
};


// Here create the deterministic policy for one car(see cpp)
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
    Action get_next_action(State s);
};

