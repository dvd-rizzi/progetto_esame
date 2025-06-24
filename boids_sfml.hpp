#ifndef BOIDS_SFML
#define BOIDS_SFML
#include <SFML/Graphics.hpp>
#include "boids.hpp"

namespace boids_display {

    extern sf::RenderWindow window;

    void initialize(project::boids_flock& flock_ref);

    void draw(const std::vector<project::boid>& flock);
}

#endif