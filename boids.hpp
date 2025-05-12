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

class boids_flock {
  int N_;
  std::vector<boid> flock_;
  double d_;
  double ds_;
  double s_;
  double a_;
  double c_;

 public:
  boids_flock(int N, std::vector<boid> flock, double d, double ds, double s, double a, double c) : N_{N}, flock_{flock}, d_{d}, ds_{ds}, s_{s}, a_{a}, c_{c} {}

  boid boid_initialize();

  void flock_formation();

  bool upper_distance(boid a, boid b);

  bool lower_distance(boid a, boid b);

  double reciprocal_distance_x(boid a, boid b);

  double reciprocal_distance_y(boid a, boid b);

  double separation_rule_x(boid a);

  double separation_rule_y(boid a);

  double alignment_rule_x(boid a, boid b);

  double alignment_rule_y(boid a, boid b);
};

}  // namespace project

#endif