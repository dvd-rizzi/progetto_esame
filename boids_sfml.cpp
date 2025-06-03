#include "boids_sfml.hpp"
#include "boids.hpp"
#include <vector>

namespace boids_display {
    static project::boids_flock* flock_ptr = nullptr;

    //sf::RenderWindow window(sf::VideoMode(800, 600), "Test");
    sf::CircleShape boid; 
    
    sf::View view(sf::FloatRect(-20.f, -20.f, 40.f, 40.f));
    
    void initialize(project::boids_flock& flock_ref) {
        window.setView(view);
        flock_ptr = &flock_ref;
        flock_ptr->project::boids_flock::flock_formation();
    }
    
    /*void run() {
        while (window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed){
                    window.close();
                }
            }
            window.clear(sf::Color::White);
            for(project::boid const& a : flock_ptr->get_flock()) {
                 sf::CircleShape boid_shape(0.5f);
                boid_shape.setFillColor(sf::Color::Black);
                boid_shape.setPosition(static_cast<float>(a.x_position), static_cast<float>(a.y_position));
                boid_shape.setOrigin(0.5f, 0.5f);
                window.draw(boid_shape);
            }
        window.display();
        }
    }*/

    void draw(const std::vector<project::boid>& flock) {
    window.clear(sf::Color::White);

    for (const auto& a : flock) {
        sf::CircleShape boid_shape(0.5f);
        boid_shape.setFillColor(sf::Color::Black);
        boid_shape.setPosition(static_cast<float>(a.x_position), static_cast<float>(a.y_position));
        boid_shape.setOrigin(0.5f, 0.5f);
        window.draw(boid_shape);
    }

    window.display();
}

}