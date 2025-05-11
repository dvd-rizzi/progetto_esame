#ifndef BOIDS_HPP
#define BOIDS_HPP
#include <vector>


namespace project{

struct boid{
    double x_position;
    double y_position;
    double v_x;
    double v_y;
};


class boids_flock{
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

    double separation_rule(boid a, boid b);
};

}

#endif