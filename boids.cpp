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

const std::vector<boid>& boids_flock::get_flock() const {
    return flock_;
}

bool operator!=(boid a, boid b) {
    return a.x_position != b.x_position || a.y_position != b.y_position || a.v_x != b.v_x || a.v_y != b.v_y; 
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

double boids_flock::reciprocal_distance_x(boid a, boid b) {
    return b.x_position - a.x_position;
}

double boids_flock::reciprocal_distance_y(boid a, boid b) {
    return b.y_position - a.y_position;
}

double boids_flock::separation_rule_x(boid a) {
    double v_1x{0.};
    for (auto const& b : flock_) {
        if (a != b && lower_distance(a, b) == false && upper_distance(a, b) == true) {
            v_1x += reciprocal_distance_x(a, b);
        }
    }
    return -s_ * v_1x;
}

double boids_flock::separation_rule_y(boid a) {
    double v_1y{0.};
    for (auto const& b : flock_) {
        if (a != b && lower_distance(a, b) == false && upper_distance(a, b) == true) {
            v_1y += reciprocal_distance_y(a, b);
        }
    }
    return -s_ * v_1y;
}


double boids_flock::alignment_rule_x(boid a) {
    int const& N_ = flock_.size();
    double v_2x;
    for (auto const& b : flock_) {
        if (a != b && lower_distance(a, b) == true && upper_distance(a, b) == true) {
            double mean_velocity =+ (b.v_x)/(N_ -1);
            v_2x = mean_velocity - a.v_x;
        }
    }
    return a_ * v_2x;
}

double boids_flock::alignment_rule_y(boid a) {
    int const& N_ = flock_.size();
    double v_2y;
    for (auto const& b : flock_) {
        if (a != b && lower_distance(a, b) == true && upper_distance(a, b) == true) {
            double mean_velocity =+ (b.v_y)/(N_ -1);
            v_2y = mean_velocity - a.v_y;

        }
    }
    return a_ * v_2y;
}

double boids_flock::center_of_mass_x() {
    int const& N_ = flock_.size();
    double sum;
    for (auto const& a : flock_) {
        sum+=a.x_position;
    }
    return sum/N_;
}


double boids_flock::center_of_mass_y() {
    int const& N_ = flock_.size();
    double sum;
    for (auto const& a : flock_) {
        sum+=a.y_position;
    }
    return sum/N_;
}

double boids_flock::cohesion_rule_x(boid a) {
    return c_ * (center_of_mass_x() - a.x_position);
}

double boids_flock::cohesion_rule_y(boid a) {
    return c_ * (center_of_mass_y() - a.y_position);
}

void boids_flock::corner_behaviour() {
    for(auto& a : flock_) {
        if(a.x_position <= -20 || a.x_position >= 20) {
            a.v_x = -a.v_x;
        }
        if(a.y_position <= -20 || a.y_position >= 20) {
            a.v_y = -a.v_y;
        }
    }
}

}