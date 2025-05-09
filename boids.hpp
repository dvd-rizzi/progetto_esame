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

boid boid_initialize();

class boids_flock{
    int N_;
    std::vector<boid> flock_;
    double d_;
    double ds_;
    double s_;
    double a_;
    double c_;

    public:

    void flock_formation();

    bool minimum_distance(boid a, boid b);


};

}

#endif