#ifndef BOIDS_HPP
#define BOIDS_HPP
#include <vector>

namespace project {

struct boid {
  double x_position;
  double y_position;
  double v_x;
  double v_y;
};

bool operator!=(boid a, boid b);

struct components {
  double x;
  double y;
};

components operator+(components a, components b);

components operator-(components a, components b);

components operator*(double a, components b);

components operator/(components a, double b);

bool operator==(components a, components b);

class boids_flock {
  std::vector<boid> flock_;
  int N_;
  double d_;
  double ds_;
  double s_;
  double a_;
  double c_;

  bool upper_distance(boid const& a, boid const& b) const;

  bool lower_distance(boid const& a, boid const& b) const;

  components reciprocal_distance(boid const& a, boid const& b) const;

  components center_of_mass_nearby(boid const& a) const;

 public:
  boids_flock(int N, double d, double ds, double s, double a, double c) : N_{N}, d_{d}, ds_{ds}, s_{s}, a_{a}, c_{c} {}

  const std::vector<boid>& get_flock() const;

  bool get_upper_distance(boid const& a, boid const& b) const;

  bool get_lower_distance(boid const& a, boid const& b) const;

  components get_reciprocal_distance(boid const& a, boid const& b) const;

  components get_center_of_mass_nearby(boid const& a) const;

  static boid boid_initialize();

  void flock_formation();

  void addBoid(boid const& a);

  components separation_rule(boid const& a) const;

  components alignment_rule(boid const& a) const;

  components cohesion_rule(boid const& a) const;

  void corner_force();

  double flock_velocity();

  double mean_velocity();

  double velocity_st_deviation();

  double mean_distance();

  components external_effects(boid const& a);

  void velocities_update();

  void position_update();

};

}  // namespace project

#endif