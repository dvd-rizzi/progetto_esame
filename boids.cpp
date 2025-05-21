#include "boids.hpp"
#include <random>
#include <cmath>
#include <algorithm>
#include <stdexcept>

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
    double distance{0.};
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
    double v_2x{0.};
    double sum_vx{0.};
    int nearby_boids{0};
    for (auto const& b : flock_) {
        if (a != b && lower_distance(a, b) == true && upper_distance(a, b) == true) {
            sum_vx += b.v_x;
            nearby_boids += 1;
        }
    }
    v_2x = (sum_vx / nearby_boids) - a.v_x;
    return a_ * v_2x;
}

double boids_flock::alignment_rule_y(boid a) {
    double v_2y{0.};
    double sum_vy{0.};
    int nearby_boids{0};
    for (auto const& b : flock_) {
        if (a != b && lower_distance(a, b) == true && upper_distance(a, b) == true) {
            sum_vy += b.v_y;
            nearby_boids += 1;
        }
    }
    v_2y = (sum_vy / nearby_boids) - a.v_y;
    return a_ * v_2y;
}

double boids_flock::center_of_mass_x_nearby(boid a) {
    double sum{0.};
    int nearby_boids{0};
    for (auto const& b : flock_) {
        if(lower_distance(a, b) == true && upper_distance(a, b) == true){
            sum+=b.x_position;
            nearby_boids += 1;
        }
    }
    if (nearby_boids==0) {
        throw std::runtime_error{"no boids are between lower and upper distances"};
    };
    return sum/nearby_boids;
}


double boids_flock::center_of_mass_y_nearby(boid a) {
    double sum{0.};
    int nearby_boids{0};
    for (auto const& b : flock_) {
        if(lower_distance(a, b) == true && upper_distance(a, b) == true) {
            sum+=b.y_position;
            nearby_boids += 1;
        }
    }
    if (nearby_boids==0) {
        throw std::runtime_error{"no boids are between lower and upper distances"};
    };
    return sum/nearby_boids;
}

double boids_flock::cohesion_rule_x(boid a) {
    return c_ * (center_of_mass_x_nearby(a) - a.x_position);
}

double boids_flock::cohesion_rule_y(boid a) {
    return c_ * (center_of_mass_y_nearby(a) - a.y_position);
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

double boids_flock::mean_velocity() {
    double sum_vx{0.};
    double sum_vy{0.};
    double v_tot{0.};
    sum_vx = std::accumulate(flock_.begin(), flock_.end(), 0, [] (double accumulate, boid const& c) {return accumulate + c.v_x;});
    sum_vy = std::accumulate(flock_.begin(), flock_.end(), 0, [] (double accumulate, boid const& c) {return accumulate + c.v_y;});
    v_tot = std::sqrt(std::pow(sum_vx, 2) + std::pow(sum_vy, 2));
    return v_tot / N_;
}

double boids_flock::velocity_st_deviation(){
    return 0;
}

double boids_flock::mean_distance(){
    return 0;
}
}