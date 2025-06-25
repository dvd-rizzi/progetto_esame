#include "boids_sfml.hpp"
#include "boids.hpp"
#include <vector>

namespace boids_display {
    static project::boids_flock* flock_ptr = nullptr;
    sf::RenderWindow window;
    
    sf::View view(sf::FloatRect(-150.f, -150.f, 300.f, 300.f));
    
    void initialize(project::boids_flock& flock_ref) {
        window.create(sf::VideoMode(1000, 800), "Boids");
        window.setView(view);
        flock_ptr = &flock_ref;
        flock_ptr->project::boids_flock::flock_formation();
    }
    
    void draw(const std::vector<project::boid>& flock) {
    window.clear(sf::Color::White);

    for (const auto& a : flock) {
        sf::CircleShape boid_shape(1.f);
        boid_shape.setFillColor(sf::Color::Black);
        boid_shape.setPosition(static_cast<float>(a.x_position), static_cast<float>(a.y_position));
        boid_shape.setOrigin(1.f, 1.f);
        window.draw(boid_shape);
    }

    window.display();
}

}