#include "../examples/simple-c-interface/anki-simplified.h"


//TODO: replace State by centroids
class Centroid {
public:
    Centroid(int id_, float x_, float y_, int straight_, int lane_) {id = id_; x = x_; y = y_; straight = straight_; lane = lane_; };
    int get_id() { return id;};
    float get_x() {return x;};
    float get_y() {return y;};

    float get_distance_squared(float a, float b);
    
    
    

private:
    int id;
    int straight; //if 1 then the state is in a straight part of the track, if curve then 0
    int lane; // current lane (only 3 in the descritzed state space)
    float x;
    float y;
};


// Define a clas state -= position + current speed of the car + position of the other cars
class State {
public:
    State(float x, float y, int id);

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
    Action get_next_action(State s) {return ActionLane(0,0,0);}; //TODO
};


//...
