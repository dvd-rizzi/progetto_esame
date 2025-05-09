#include "boids.hpp"
#include <random>
#include <cmath>

namespace project {

std::random_device r;
std::default_random_engine eng(r());
std::uniform_real_distribution<double> angle(0.0, 2.0 * M_PI);
std::uniform_real_distribution<double> x_position(-20., 20.);
std::uniform_real_distribution<double> y_position(-20., 20.);
std::normal_distribution<double> speed(1., 0.1);

boid boid_initialize() {
    double theta = angle(eng);
    double v = speed(eng);
    return {x_position(eng), y_position(eng), v * std::cos(theta), v* std::sin(theta)};
}

void boids_flock::flock_formation() {
    flock_.clear();
    for(int i{0}; i <= N_; ++i) {
        flock_.push_back(boid_initialize());
    }
}

bool boids_flock::minimum_distance(boid a, boid b) {
    double distance;
    distance = std::sqrt((a.x_position - b.x_position) + (a.y_position - b.y_position));
    return d_ < distance;
}



}