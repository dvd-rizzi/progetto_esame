#ifndef BOIDS_SFML
#define BOIDS_SFML
#include <SFML/Graphics.hpp>
#include "boids.hpp"

namespace boids_display {

    inline sf::RenderWindow window(sf::VideoMode(800, 600), "Test");

    void initialize(project::boids_flock& flock_ref);

    void run();

    void draw(const std::vector<project::boid>& flock);
}

#endif