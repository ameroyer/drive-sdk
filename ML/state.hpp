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
	anki_s_change_lane(h, offset, speed, accel);
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
public:
    Action get_next_action(State s);
};

