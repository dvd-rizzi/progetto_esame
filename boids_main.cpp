#include "boids.hpp"
#include "boids_sfml.hpp"
#include <chrono>
#include <iostream>
#include <cassert>

int main() {
    int N_;
    std::cout << "inserisci il numero di boid: ";
    std::cin >> N_;
    std::cout << "\n";
    assert(N_>0);

    double d_;
    std::cout << "inserisci la distanza massima tra i boid: ";
    std::cin >> d_;
    std::cout << "\n";
    assert(d_>0);

    double ds_;
    std::cout << "inserisci la distanza minima tra i boid: ";
    std::cin >> ds_;
    std::cout << "\n";
    assert(0 < ds_ && ds_< d_);

    double s_;
    std::cout << "inserisci il fattore di separazione: ";
    std::cin >> s_;
    std::cout << "\n";
    assert(s_>0);

    double a_;
    std::cout << "inserisci il fattore di allineamento (<1): ";
    std::cin >> a_;
    std::cout << "\n";
    assert(0 < a_ && a_ < 1);

    double c_;
    std::cout << "inserisci il fattore di coesione: ";
    std::cin >> c_;
    std::cout << "\n";
    assert(c_>0);

    std::vector<project::boid> flock_vector;
    project::boids_flock flock(N_,flock_vector,d_,ds_,s_,a_,c_);
    
    for (int i{0}; i<N_; ++i) {
        project::boid b=flock.boid_initialize();
        flock_vector.push_back(b);
    }
}