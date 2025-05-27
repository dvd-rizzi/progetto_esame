#include "boids_sfml.hpp"
#include "boids.hpp"
#include <SFML/Graphics.hpp>
#include <vector>

namespace boids_display {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Test");
    sf::CircleShape boid; 
    std::vector<project::boid> flockvect;
    project::boids_flock flock{30, flockvect, 15., 1., 2., 0.5, 1.};

    sf::View view(sf::FloatRect(-20.f, -20.f, 40.f, 40.f));
    
    void initialize() {
        window.setView(view);
        flock.project::boids_flock::flock_formation();
    }
    
    void run() {
        initialize();
        while (window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed){
                    window.close();
                }
                window.clear(sf::Color::White);
                for(project::boid const& a : flock.get_flock()) {
                     sf::CircleShape boid_shape(0.5f);
                    boid_shape.setFillColor(sf::Color::Black);
                    boid_shape.setPosition(static_cast<float>(a.x_position), static_cast<float>(a.y_position));
                    boid_shape.setOrigin(0.5f, 0.5f);
                    window.draw(boid_shape);
                }
            }
        window.display();
        }
    }

}