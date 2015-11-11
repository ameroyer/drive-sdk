#include "state.hpp"

float Centroid::get_distance_squared(float a, float b) {
    return (x - a) * (x - a) + (y - b) * (y - b);
}

