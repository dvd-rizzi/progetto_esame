#include "boids.hpp"
#include "boids_sfml.hpp"
#include <iostream>
#include <cassert>
#include <chrono>
#include <thread>

int main() {
    int N_;
    std::cout << "inserisci il numero di boid: " << '\n';
    std::cin >> N_;
    assert(N_>0);

    double d_;
    std::cout << "inserisci la distanza massima tra i boid: " << '\n';
    std::cin >> d_;
    assert(d_>0);

    double ds_;
    std::cout << "inserisci la distanza minima tra i boid: " << '\n';
    std::cin >> ds_;
    assert(0 < ds_ && ds_< d_);

    double s_;
    std::cout << "inserisci il fattore di separazione: " << '\n';
    std::cin >> s_;
    assert(s_>0);

    double a_;
    std::cout << "inserisci il fattore di allineamento (<1): " << '\n';
    std::cin >> a_;
    assert(0 < a_ && a_ < 1);

    double c_;
    std::cout << "inserisci il fattore di coesione: " << '\n';
    std::cin >> c_;
    assert(c_>0);

    std::vector<project::boid> flock_vector;
    project::boids_flock flock(N_,flock_vector,d_,ds_,s_,a_,c_);
    
    boids_display::initialize(flock);
    std::chrono::milliseconds tick(15); 
    auto next_tick = std::chrono::steady_clock::now();
    
    while (boids_display::window.isOpen()) {
        sf::Event event;
        while (boids_display::window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                boids_display::window.close();
            }
        }
        // aggiorna simulazione
        flock.velocities_update();
        flock.position_update_loop();
        flock.corner_behaviour();
        // disegna lo stato corrente
        boids_display::draw(flock.get_flock());

        next_tick += tick;
        std::this_thread::sleep_until(next_tick);
    }


}