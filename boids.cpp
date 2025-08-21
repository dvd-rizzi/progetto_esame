#include "boids.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>

namespace project {

std::random_device r;
std::default_random_engine eng(r());
std::uniform_real_distribution<double> angle(0.0, 2.0 * M_PI);
std::uniform_real_distribution<double> x_position(-150., 150.);
std::uniform_real_distribution<double> y_position(-150., 150.);
std::normal_distribution<double> speed(15., 1.);

const std::vector<boid>& boids_flock::get_flock() const { return flock_; }

bool operator!=(boid a, boid b) {
  return a.x_position != b.x_position || a.y_position != b.y_position ||
         a.v_x != b.v_x || a.v_y != b.v_y;
}

module operator+(module a, module b) {
  return {a.x+b.x,a.y+b.y};
}

module operator-(module a, module b){
  return {a.x-b.x,a.y-b.y};
}

module operator*(double a, module b){
  return {a*b.x,a*b.y};
}

module operator/(module a, double b){
  return {a.x/b,a.y/b};
}

bool operator==(module a, module b) {
  return (a.x==b.x && a.y==b.y);
}

bool boids_flock::upper_distance(boid const& a, boid const& b) const {
  double distance;
  distance = std::sqrt(std::pow((a.x_position - b.x_position), 2) +
                       std::pow((a.y_position - b.y_position), 2));
  return distance < d_;
}

bool boids_flock::get_upper_distance(boid const& a, boid const& b) const {
  return boids_flock::upper_distance(a, b);
};

bool boids_flock::lower_distance(boid const& a, boid const& b) const {
  double distance{0.};
  distance = std::sqrt(std::pow((a.x_position - b.x_position), 2) +
                       std::pow((a.y_position - b.y_position), 2));
  return distance > ds_;
}

bool boids_flock::get_lower_distance(boid const& a, boid const& b) const {
  return boids_flock::lower_distance(a, b);
};

//funzione nuova da non eliminare, è bellina e carina e non dipende da x e y
module boids_flock::reciprocal_distance(boid const& a, boid const& b) const {
  return {b.x_position-a.x_position,b.y_position-a.y_position};
}

module boids_flock::get_reciprocal_distance(boid const& a, boid const& b) const {
  return boids_flock::reciprocal_distance(a,b);
}


module boids_flock::center_of_mass_nearby(boid const& a) const {
  module sum{0.,0.};
  int nearby_boids{0};
  for (auto const& b : flock_) {
    if (lower_distance(a, b) == true && upper_distance(a, b) == true) {
      sum.x += b.x_position;
      sum.y += b.y_position;
      nearby_boids += 1;
    }
  }
  if (nearby_boids == 0) {
    return {a.x_position, a.y_position};
  };
  return {sum.x/nearby_boids,sum.y/nearby_boids};
}

module boids_flock::get_center_of_mass_nearby(boid const& a) const {
  return boids_flock::center_of_mass_nearby(a);
}

boid boids_flock::boid_initialize() {
  double theta = angle(eng);
  double v = speed(eng);
  return {x_position(eng), y_position(eng), v * std::cos(theta),
          v * std::sin(theta)};
}

void boids_flock::flock_formation() {
  flock_.clear();
  for (int i{0}; i < N_; ++i) {
    flock_.push_back(boid_initialize());
  }
}

void boids_flock::addBoid(boid const& a) {
  if (flock_.size() == static_cast<std::vector<project::boid>::size_type>(N_)) {
    N_ = N_ + 1;
  };
  flock_.push_back(a);
}


module boids_flock::separation_rule(boid const& a) const{
  module v_1{0.,0.};
  for (auto const& b : flock_) {
    if (a != b && lower_distance(a, b) == false &&
        upper_distance(a, b) == true) {
      v_1 = v_1 + reciprocal_distance(a, b);
    }
  }
  return -s_ * v_1;
}

module boids_flock::alignment_rule(boid const& a) const{
  module v_2{0.,0.};
  module sum_v{0.,0.};
  int nearby_boids{0};
  for (auto const& b : flock_) {
    if (a != b && lower_distance(a, b) == true &&
        upper_distance(a, b) == true) {
      sum_v.x += b.v_x;
      sum_v.y += b.v_y;
      nearby_boids += 1;
    }
  }
  if (nearby_boids == 0) {
    return {0.,0.};
  };
  module av{a.v_x,a.v_y};
  v_2 = (sum_v / nearby_boids) - av;
  return a_ * v_2;
}

module boids_flock::cohesion_rule(boid const& a) const {
  module ap{a.x_position,a.y_position};
  return c_ * (center_of_mass_nearby(a) - ap);
}

void boids_flock::corner_force() {
  double margin{30.};
  double turn_factor{1.5};

  for (auto& a : flock_) {
    if (a.x_position < -150 + margin) a.v_x += turn_factor;
    if (a.x_position > 150 - margin) a.v_x -= turn_factor;
    if (a.y_position < -150 + margin) a.v_y += turn_factor;
    if (a.y_position > 150 - margin) a.v_y -= turn_factor;
  }
}

double boids_flock::flock_velocity() {
  double sum_vx{0.};
  double sum_vy{0.};
  double mean_vx{0.};
  double mean_vy{0.};
  double v{0.};
  sum_vx = std::accumulate(flock_.begin(), flock_.end(), 0.,
                      [](double acc, boid const& b) { return acc + b.v_x; });
  sum_vy = std::accumulate(flock_.begin(), flock_.end(), 0.,
                      [](double acc, boid const& b) { return acc + b.v_y; });
  mean_vx = sum_vx / N_;
  mean_vy = sum_vy / N_;
  v = std::sqrt(mean_vx * mean_vx + mean_vy * mean_vy);
  return v;
}

double boids_flock::mean_velocity() {
  double sum_v{0.};
  sum_v = std::accumulate(flock_.begin(), flock_.end(), 0.,
                          [](double acc, boid const& b) {
                            double v = std::sqrt(b.v_x * b.v_x + b.v_y * b.v_y);
                            return acc + v;
                          });
  return sum_v / static_cast<double>(flock_.size());
}

double boids_flock::velocity_st_deviation() {
  double v_module{0.};
  double sum{0.};
  double mean_v = mean_velocity();
  for (auto const& b : flock_) {
    v_module = std::sqrt(b.v_x * b.v_x + b.v_y * b.v_y);
    sum += std::pow((v_module - mean_v), 2);
  }
  return std::sqrt(sum / (N_ - 1));
}

double boids_flock::mean_distance() {
  double sum_distance_modules{0.};
  int number_distances = ((N_ * (N_ - 1)));
  for (auto const& a : flock_) {
    for (auto const& b : flock_) {
      if (a != b) {
        sum_distance_modules +=
            std::sqrt(std::pow(reciprocal_distance(a, b).x, 2) +
                      std::pow(reciprocal_distance(a, b).y, 2));
      }
    }
  }
  return sum_distance_modules / number_distances;
}


module boids_flock::external(boid const& a) {
  module v{0.,0.};
  v= v+boids_flock::alignment_rule(a)+boids_flock::cohesion_rule(a)+boids_flock::separation_rule(a);
  return v;
}


void boids_flock::velocities() {
  std::vector<module> result;
  const double min_speed{10.};
  const double max_speed{20.};
  for (auto& a : flock_) {
    result.push_back(boids_flock::external(a));
  }
  long unsigned int i = 0;
  for (auto& a : flock_) {
    a.v_x += result[i].x;
    a.v_y += result[i].y;
    double v = std::sqrt(a.v_x * a.v_x + a.v_y * a.v_y);

    if (v < min_speed) {
      a.v_x = (a.v_x / v) * min_speed;
      a.v_y = (a.v_y / v) * min_speed;
    } else if (v > max_speed) {
      a.v_x = (a.v_x / v) * max_speed;
      a.v_y = (a.v_y / v) * max_speed;
    }
    i++;
  }
}

void boids_flock::position_update() {
  const double dt = 0.015;
  for (auto& a : flock_) {
    a.x_position += a.v_x * dt;
    a.y_position += a.v_y * dt;
  }
}

}  // namespace project