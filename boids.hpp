#ifndef BOIDS_HPP
#define BOIDS_HPP
#include <vector>

namespace project {

struct boid {
  double x_position;
  double y_position;
  double v_x;
  double v_y;

  /*
  //sto solo droppando qua l'idea attuale del metodo

  sai che non sono sicuro che vada bene mettere funzioni in una struct
  */
};

struct mean_velocity_vector {
  double mean_velocity;
  double theta;
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

  bool upper_distance(boid const& a, boid const& b) const;

  bool lower_distance(boid const& a, boid const& b) const;

  double reciprocal_distance_x(boid const& a, boid const& b) const;

  double reciprocal_distance_y(boid const& a, boid const& b) const;

  double center_of_mass_x_nearby(boid const& a) const;

  double center_of_mass_y_nearby(boid const& a) const;

 public:
  boids_flock(int N, std::vector<boid> flock, double d, double ds, double s, double a, double c) : N_{N}, flock_{flock}, d_{d}, ds_{ds}, s_{s}, a_{a}, c_{c} {}

  const std::vector<boid>& get_flock() const;

  boid boid_initialize();

  void flock_formation();

  bool get_upper_distance(boid const& a, boid const& b) const;

  bool get_lower_distance(boid const& a, boid const& b) const;

  double get_reciprocal_distance_x(boid const& a, boid const& b) const;

  double get_reciprocal_distance_y(boid const& a, boid const& b) const;

  double separation_rule_x(boid const& a) const;

  double separation_rule_y(boid const& a) const;

  double alignment_rule_x(boid const& a) const;

  double alignment_rule_y(boid const& a) const;

  double get_center_of_mass_x_nearby(boid const& a) const;

  double get_center_of_mass_y_nearby(boid const& a) const;

  double cohesion_rule_x(boid const& a) const;

  double cohesion_rule_y(boid const& a) const;

  void corner_behaviour();

  mean_velocity_vector mean_velocity();

  double velocity_st_deviation();

  double mean_distance();

  void external_effects(boid* a);

  void velocities_update();

  void update_flock_loop();
};

}  // namespace project

#endif