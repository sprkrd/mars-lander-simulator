#include <iostream>

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "mars-lander.hpp"


double k_pixel_meter_ratio = 0.114;


int main(int argc, char* argv[]) {

  mars::Context context;
  context.dt = 1.0/30;
  context.surface = std::vector<mars::Vector2D>{{0,350}, {1000,600}, {5000,700}, {7000,0}};

  mars::LanderState state({3500,1500}, {0,0}, 0, 0, 350, &context);

  int screen_width = context.width*k_pixel_meter_ratio;
  int screen_height = context.height*k_pixel_meter_ratio;

  sf::RenderWindow window(sf::VideoMode(screen_width,screen_height), "Mars lander");
  window.setVerticalSyncEnabled(true);

  sf::Clock clock;

  double accumulator = 0;

  while (window.isOpen()) {

    accumulator += clock.restart().asSeconds();

    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        window.close();
    }

    while (accumulator >= context.dt) {
      state.step({0,0});
      accumulator -= context.dt;
    }

    window.clear(sf::Color::Black);

    sf::VertexArray surface(sf::LinesStrip, context.surface.size());
    for (size_t i = 0; i < context.surface.size(); ++i) {
      const auto& point = context.surface[i];
      surface[i].position.x = point.x*k_pixel_meter_ratio;
      surface[i].position.y = screen_height - point.y*k_pixel_meter_ratio - 1;
      surface[i].color = sf::Color(238, 53, 14);
    }
    window.draw(surface);

    const auto& lander_pos = state.getPosition();
    sf::CircleShape lander(3);
    lander.setOrigin(3, 3);
    lander.setPosition(lander_pos.x*k_pixel_meter_ratio, screen_height - lander_pos.y*k_pixel_meter_ratio - 1);
    window.draw(lander);

    window.display();
  }
}

