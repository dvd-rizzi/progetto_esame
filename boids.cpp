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


bool operator!=(boid a, boid b) {
    return a.x_position != b.x_position && a.y_position != b.y_position && a.v_x != b.v_x && a.v_y != b.v_y; 
}

boid boids_flock::boid_initialize() {
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

bool boids_flock::upper_distance(boid a, boid b) {
    double distance;
    distance = std::sqrt(std::pow((a.x_position - b.x_position), 2) + std::pow((a.y_position - b.y_position), 2));
    return distance < d_;
}

bool boids_flock::lower_distance(boid a, boid b) {
    double distance;
    distance = std::sqrt(std::pow((a.x_position - b.x_position), 2) + std::pow((a.y_position - b.y_position), 2));
    return distance > ds_;
}

double boids_flock::separation_rule_x(boid a, boid b) {
    int N_ = flock_.size();
    double v_1x;
    for (auto const& a : flock_) {
        for (auto const& b : flock_) {
            if (a != b && lower_distance(a, b) == false && upper_distance(a, b) == true) {
                double reciprocal_distance = std::abs(a.x_position - b.x_position);
                v_1x += reciprocal_distance;
            }
        }
    }
    return -s_ * v_1x;
}

double boids_flock::separation_rule_y(boid a, boid b) {
    int N_ = flock_.size();
    double v_1y;
    for (auto const& a : flock_) {
        for (auto const& b : flock_) {
            if (a != b && lower_distance(a, b) == false && upper_distance(a, b) == true) {
                double reciprocal_distance = std::abs(a.y_position - b.y_position);
                v_1y += reciprocal_distance;
            }
        }
    }
    return -s_ * v_1y;
}

}