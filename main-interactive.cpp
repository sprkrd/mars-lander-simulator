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

class EnvironmentRender;

class EnvironmentRender {
  public:
    EnvironmentRender() {
      if (not _ship_texture.loadFromFile("sprites/ship.png")) {
        throw "Texture cannot be loaded!";
      }
      _ship_texture.setSmooth(true);
      _ship_sprite.setTexture(_ship_texture);
      setOriginAtBottomCenter(_ship_sprite);
      setSpriteWidth(_ship_sprite, 40);
    }

    void init(const mars::Environment& environment, sf::RenderWindow& window) {
      _screen_width = environment.getWidth()*k_pixel_meter_ratio;
      _screen_height = environment.getHeight()*k_pixel_meter_ratio;
      window.create(sf::VideoMode(_screen_width, _screen_height), "Mars Lander simulator");
      window.setVerticalSyncEnabled(true);
    }

    void render(const mars::Environment& environment, sf::RenderWindow& window) {
      window.clear(sf::Color::Black);
      sf::VertexArray surface(sf::LinesStrip, environment.getSurface().size());
      for (size_t i = 0; i < environment.getSurface().size(); ++i) {
        const auto& point = environment.getSurface()[i];
        surface[i].position.x = point.x*k_pixel_meter_ratio;
        surface[i].position.y = _screen_height - point.y*k_pixel_meter_ratio;
        surface[i].color = sf::Color(238, 53, 14);
      }
      window.draw(surface);
      const auto& lander_pos = environment.getLander(0).getPosition();
      _ship_sprite.setPosition(
          lander_pos.x*k_pixel_meter_ratio,
          _screen_height-lander_pos.y*k_pixel_meter_ratio);
      _ship_sprite.setRotation(-environment.getLander(0).getRotation());
      window.draw(_ship_sprite);
      window.display();
    }

  private:
    int _screen_width, _screen_height;
    sf::Texture _ship_texture;
    sf::Sprite _ship_sprite;
};

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  mars::Environment environment;
  environment.setSurface({{0,350}, {1000,600}, {5000,700}, {7000,0}});
  //environment.setGravity(100);
  
  auto& lander = environment.createLander();
  std::cout << lander.getId() << std::endl;
  lander.setPosition({3500,1500});
  lander.setFuel(350);
  lander.setRotation(45);
  lander.setPower(4);

  sf::RenderWindow window;
  EnvironmentRender environment_render;
  environment_render.init(environment, window);

  sf::Clock clock;
  double accumulator = 0;
  while (window.isOpen()) {

    accumulator += clock.restart().asSeconds();

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

    environment_render.render(environment, window);

  }
}

