#include "../examples/simple-c-interface/anki-simplified.h"


//TODO: replace State by centroids
class Centroid {
public:
    Centroid(){};
    Centroid(int id_, float x_, float y_, float straight_, int vseg_, int lane_, int startline_) {id = id_; x = x_; y = y_; straight = straight_; vseg=vseg_; startline=startline_; lane = lane_; };
    int get_id() { return id;};
    float get_x() {return x;};
    float get_y() {return y;};
    int get_start() { return startline;};
    int get_lane() { return lane;};
    float get_stra() { return straight;};

    float get_distance_squared(float a, float b);
    
    
    

private:
    int id;
    float straight; //if 1 then the state is in a straight part of the track, if curve then 0
    int vseg;
    int startline;
    int lane; // current lane (only 3 in the descritzed state space)
    float x;
    float y;
};


// Define a clas state -= position + current speed of the car + position of the other cars
class State {
public:
    State(float x, float y, int id);
	State(Centroid car_,float speed_) {car=car_; speed=speed_;}
	Centroid get_car(){return car;}

private:
    Centroid car;
    Centroid* opponents;
    float speed;
};



// Define a class for actions
class Action {
public:
    void apply(AnkiHandle h) {return;};
};


class ActionSpeed: public Action {
private:
    float speed;
    float accel;
public:
    ActionSpeed(float speed_, float accel_) {speed = speed_; accel = accel_;};
    void apply(AnkiHandle h) { anki_s_set_speed(h, speed, accel);}
};


class ActionLane: public Action {
private:
    float offset;
    float speed;
    float accel;
public:
    ActionLane(float offset_, float speed_, float accel_) {offset = offset_; speed = speed_; accel = accel_;};
    void apply(AnkiHandle h) { anki_s_change_lane(h, offset, speed, accel);}
};


// Define a virtualclass for policies
class Policy {
 public:
    Action get_next_action(State s) {return Action();};
};


// Here create the deterministic polycy for one car
class DetOneCarPolicy: public Policy {
 public:
    Action get_next_action(State s) {
		int centid=s.get_car().get_id();
		int speed1=1000;
		if(centid>50){speed1=200;}
		return ActionSpeed(speed1,2000);
	}; //TODO
};


//...
