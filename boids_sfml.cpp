#include "boids_sfml.hpp"

#include <stdexcept>
#include <vector>

#include "boids.hpp"
namespace boids_display {
static project::boids_flock* flock_ptr = nullptr;
sf::RenderWindow window;

sf::View view(sf::FloatRect(-150.f, -150.f, 300.f, 300.f));

sf::Texture background_texture;
sf::Texture boid_texture;
sf::Sprite background_sprite;

void initialize(project::boids_flock& flock_ref) {
  window.create(sf::VideoMode(1000, 1000), "Boids");
  window.setView(view);
  flock_ptr = &flock_ref;
  flock_ptr->project::boids_flock::flock_formation();

  if (!background_texture.loadFromFile("assets/reverias.jpg")) {
    throw std::runtime_error("Failed to load background texture");
  }
  if (!boid_texture.loadFromFile("assets/rondine.png")) {
    throw std::runtime_error("Failed to load boid texture");
  }

  background_sprite.setTexture(background_texture);

  sf::Vector2u texture_size = background_texture.getSize();
  float scale_x = 300.f / static_cast<float>(texture_size.x);
  float scale_y = 300.f / static_cast<float>(texture_size.y);
  background_sprite.setScale(scale_x, scale_y);

  background_sprite.setOrigin(static_cast<float>(texture_size.x) / 2.f,
                              static_cast<float>(texture_size.y) / 2.f);

  background_sprite.setPosition(0.f, 0.f);
}

void draw(const std::vector<project::boid>& flock) {
  window.clear(sf::Color::White);
  window.draw(background_sprite);

  for (const auto& a : flock) {
    sf::Sprite boid_sprite(boid_texture);
    boid_sprite.setPosition(static_cast<float>(a.x_position),
                            static_cast<float>(a.y_position));
    boid_sprite.setOrigin(static_cast<float>(boid_texture.getSize().x) / 2.f,
                          static_cast<float>(boid_texture.getSize().x) / 2.f);
    boid_sprite.setScale(0.009f, 0.009f);
    window.draw(boid_sprite);
  }

  window.display();
}

} 