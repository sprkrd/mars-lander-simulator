#include "environment.hpp"

mars::Environment::Environment() :
  _width(k_default_width),
  _height(k_default_height),
  _gravity(k_default_gravity),
  _total_time(0),
  _time_step(k_default_time_step) {}

void mars::Environment::setWidth(double width) {
  _width = width;
}

void mars::Environment::setHeight(double height) {
  _height = height;
}

void mars::Environment::setGravity(double gravity) {
  _gravity = gravity;
}

void mars::Environment::setTimeStep(double time_step) {
  _time_step = time_step;
}

void mars::Environment::setSurface(const std::vector<Vector2D>& surface) {
  _surface = surface;
}

void mars::Environment::reset() {
  _landers.clear();
  _total_time = 0;
}

mars::Lander& mars::Environment::createLander() {
  _landers.emplace_back();
  _landers.back().assignEnvironment(this, _landers.size()-1);
  return _landers.back();
}

mars::Lander& mars::Environment::getLander(int id) {
  return _landers[id];
}

const mars::Lander& mars::Environment::getLander(int id) const {
  return _landers[id];
}

int mars::Environment::getNumberOfLanders() const {
  return _landers.size();
}

double mars::Environment::getWidth() const {
  return _width;
}

double mars::Environment::getHeight() const {
  return _height;
}

double mars::Environment::getGravity() const {
  return _gravity;
}

double mars::Environment::getTotalTime() const {
  return _total_time;
}

double mars::Environment::getTimeStep() const {
  return _time_step;
}

const std::vector<mars::Vector2D>& mars::Environment::getSurface() const {
  return _surface;
}

bool mars::Environment::stepIdle() {
  std::vector<Action> action_v(_landers.size());
  for (unsigned i = 0; i < _landers.size(); ++i)
    action_v[i] = {_landers[i].getPower(), _landers[i].getRotation()};
  return step(action_v);
}

bool mars::Environment::step(const std::vector<Action>& action_v, bool check_valid) {
  bool done = true;
  for (unsigned i = 0; i < _landers.size(); ++i) {
    if (_landers[i].getStatus() == LanderStatus::FALLING)
      done = _landers[i].step(action_v[i], check_valid) and done;
  }
  return done;
}

