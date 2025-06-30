#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>

#include "boids.hpp"
#include "boids_sfml.hpp"

int main() {
  int N_;
  std::cout << "inserisci il numero di boid: " << '\n';
  std::cin >> N_;
  assert(N_ > 1);

  double d_;
  std::cout << "inserisci la distanza massima tra i boid: " << '\n';
  std::cin >> d_;
  assert(d_ > 0);

  double ds_;
  std::cout << "inserisci la distanza minima tra i boid: " << '\n';
  std::cin >> ds_;
  assert(0 < ds_ && ds_ < d_);

  double s_;
  std::cout << "inserisci il fattore di separazione: " << '\n';
  std::cin >> s_;
  assert(s_ > 0);

  double a_;
  std::cout << "inserisci il fattore di allineamento (<1): " << '\n';
  std::cin >> a_;
  assert(0 < a_ && a_ < 1);

  double c_;
  std::cout << "inserisci il fattore di coesione: " << '\n';
  std::cin >> c_;
  assert(c_ > 0);

  project::boids_flock flock(N_, d_, ds_, s_, a_, c_);

  boids_display::initialize(flock);
  std::chrono::milliseconds tick(15);
  auto next_tick = std::chrono::steady_clock::now();

  auto last_log_time = std::chrono::steady_clock::now();
  std::chrono::seconds log_interval(5);

  while (boids_display::window.isOpen()) {
    sf::Event event;
    while (boids_display::window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        boids_display::window.close();
      }
      if (event.type == sf::Event::MouseButtonReleased) {
        sf::Vector2i pixelPos(event.mouseButton.x, event.mouseButton.y);
        sf::Vector2f worldPos =
            boids_display::window.mapPixelToCoords(pixelPos);
        double clickx = worldPos.x;
        double clicky = worldPos.y;
        project::boid NewBoid = project::boids_flock::boid_initialize();
        NewBoid.x_position = clickx;
        NewBoid.y_position = clicky;
        flock.addBoid(NewBoid);
      }
    }
    flock.velocities_update();
    flock.position_update();
    flock.corner_force();
    boids_display::draw(flock.get_flock());

    auto now = std::chrono::steady_clock::now();
    if (now - last_log_time >= log_interval) {
      std::cout << "Velocity: " << flock.mean_velocity() << " +- "
                << flock.velocity_st_deviation() << '\n';
      std::cout << "Mean Distance between boids: " << flock.mean_distance()
                << '\n';
      std::cout << "Flock Velocity: " << flock.flock_velocity()
                << '\n';
      std::cout << '\n';

      last_log_time = now;
    }

    next_tick += tick;
    std::this_thread::sleep_until(next_tick);
  }
}