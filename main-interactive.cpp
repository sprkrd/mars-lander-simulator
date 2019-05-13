#include <iostream>

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "environment.hpp"

double k_pixel_meter_ratio = 0.2; // 0.114;

void setSpriteWidth(sf::Sprite& sprite, float width) {
  float s = width / sprite.getGlobalBounds().width;
  sprite.setScale({s,s});
}

void setOriginAtBottomCenter(sf::Sprite& sprite) {
  auto bbox = sprite.getGlobalBounds();
  sprite.setOrigin({bbox.width/2, bbox.height});
}

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  mars::Environment environment;
  environment.setSurface({{0,350}, {1000,600}, {5000,700}, {7000,0}});
  //environment.setGravity(100);

  mars::Lander lander;
  lander.setPosition({3500,1500});
  lander.setFuel(100);
  lander.setRotation(45);
  lander.setPower(4);
  environment.registerLander(lander);

  int screen_width = environment.getWidth()*k_pixel_meter_ratio;
  int screen_height = environment.getHeight()*k_pixel_meter_ratio;

  sf::RenderWindow window(sf::VideoMode(screen_width,screen_height), "Mars lander");
  sf::Texture texture;
  if (not texture.loadFromFile("playerShip3_red.png")) {
    return -1;
  }
  sf::Sprite ship;
  ship.setTexture(texture);
  setOriginAtBottomCenter(ship);
  setSpriteWidth(ship, 30.0);
  window.setVerticalSyncEnabled(true);

  sf::Clock clock;
  double accumulator = 0;
  while (window.isOpen()) {

    accumulator += 10*clock.restart().asSeconds();

    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        window.close();
    }

    while (accumulator >= environment.getTimeStep()) {
      //environment.step({{200, 0}}, false);
      environment.stepIdle();
      accumulator -= environment.getTimeStep();
    }

    window.clear(sf::Color::Black);

    sf::VertexArray surface(sf::LinesStrip, environment.getSurface().size());
    for (size_t i = 0; i < environment.getSurface().size(); ++i) {
      const auto& point = environment.getSurface()[i];
      surface[i].position.x = point.x*k_pixel_meter_ratio;
      surface[i].position.y = screen_height - point.y*k_pixel_meter_ratio;
      surface[i].color = sf::Color(238, 53, 14);
    }
    window.draw(surface);

    const auto& lander_pos = environment.getLander(0).getPosition();
    //sf::CircleShape lander(3);
    //lander.setOrigin(3, 3);
    //lander.setPosition(lander_pos.x*k_pixel_meter_ratio, screen_height - lander_pos.y*k_pixel_meter_ratio - 1);
    ship.setPosition(lander_pos.x*k_pixel_meter_ratio, screen_height-lander_pos.y*k_pixel_meter_ratio);
    ship.setRotation(-environment.getLander(0).getRotation());
    window.draw(ship);

    window.display();
  }
}

