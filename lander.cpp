#include "lander.hpp"
#include "environment.hpp"
#include <cmath>

mars::Lander::Lander() :
  _environment(nullptr),
  _id(-1),
  _status(LanderStatus::FALLING),
  _position{0, 0},
  _velocity{0, 0},
  _rotation(0),
  _power(0),
  _fuel(0),
  _max_pow_inc(k_default_max_pow_inc),
  _max_rot_inc(k_default_max_rot_inc),
  _max_power(k_default_max_power),
  _rot_limit(k_default_rot_limit),
  _max_vy(k_default_max_vy),
  _max_vx(k_default_max_vx) {}

void mars::Lander::setStatus(LanderStatus status) {
  _status = status;
}

void mars::Lander::setPosition(const Vector2D& position) {
  _position = position;
}

void mars::Lander::setRotation(double rotation) {
  _rotation = rotation;
}

void mars::Lander::setPower(double power) {
  _power = power;
}

void mars::Lander::setFuel(double fuel) {
  _fuel = fuel;
}

void mars::Lander::setMaxPowerIncrement(double max_pow_inc) {
  _max_pow_inc = max_pow_inc;
}

void mars::Lander::setMaxRotationIncrement(double max_rot_inc) {
  _max_rot_inc = max_rot_inc;
}

void mars::Lander::setMaxPower(double max_power) {
  _max_power = max_power;
}

void mars::Lander::setRotationLimit(double rot_limit) {
  _rot_limit = rot_limit;
}

void mars::Lander::setMaxVyForLanding(double max_vy) {
  _max_vy = max_vy;
}

void mars::Lander::setMaxVxForLanding(double max_vx) {
  _max_vx = max_vx;
}

void mars::Lander::assignEnvironment(const Environment* environment, int id) {
  _environment = environment;
  _id = id;
}

const mars::Environment* mars::Lander::getEnvironment() const {
  return _environment;
}

int mars::Lander::getId() const {
  return _id;
}

mars::LanderStatus mars::Lander::getStatus() const {
  return _status;
}

const mars::Vector2D& mars::Lander::getPosition() const {
  return _position;
}

const mars::Vector2D& mars::Lander::getVelocity() const {
  return _velocity;
}

mars::Vector2D mars::Lander::getAcceleration() const {
  double rotation_rad = _rotation * M_PI/180.0;
  return { -std::sin(rotation_rad)*_power,
            std::cos(rotation_rad)*_power - _environment->getGravity() };
}

double mars::Lander::getRotation() const {
  return _rotation;
}

double mars::Lander::getPower() const {
  return _power;
}

double mars::Lander::getFuel() const {
  return _fuel;
}

double mars::Lander::getMaxPowerIncrement() const {
  return _max_pow_inc;
}

double mars::Lander::getMaxRotationIncrement() const {
  return _max_rot_inc;
}

double mars::Lander::getMaxPower() const {
  return _max_power;
}

double mars::Lander::getRotationLimit() const {
  return _rot_limit;
}

double mars::Lander::getMaxVyForLanding() const {
  return _max_vy;
}

double mars::Lander::getMaxVxForLanding() const {
  return _max_vx;
}

bool mars::Lander::isValidAction(const Action& action, const char** msg) const {
  double max_pow_inc = _environment->getTimeStep()*_max_pow_inc;
  double max_rot_inc = _environment->getTimeStep()*_max_rot_inc;
  if (not isBetween(-max_pow_inc, max_pow_inc, action.power - _power)) {
    if (msg) *msg = "Power increment not in legal range";
    return false;
  }
  if (not isBetween(0, std::min(_max_power, _fuel), action.power)) {
    if (msg) *msg = "Power not in legal range";
    return false;
  }
  if (not isBetween(-max_rot_inc, max_rot_inc, action.rotation-_rotation)) {
    if (msg) *msg = "Rotation increment not in legal range";
    return false;
  }
  if (not isBetween(-_rot_limit, _rot_limit, action.rotation)) {
    if (msg) *msg = "Rotation not in legal range";
    return false;
  }
  return true;
}

bool mars::Lander::isLost() const {
  return not isBetween(0, _environment->getWidth(), _position.x) or
         not isBetween(0, _environment->getHeight(), _position.y);
}

bool mars::Lander::step(const Action& action, bool check_valid) {
  if (check_valid) {
    const char* msg;
    if (not isValidAction(action, &msg))
      throw std::invalid_argument(msg);
  }
  double ts = _environment->getTimeStep();
  _power = action.power;
  _rotation = action.rotation;
  ParabolicTrajectory trajectory{getAcceleration(), _velocity, _position, ts};
  Collision collision;
  if (trajectory.collidesWith(_environment->getSurface(), collision)) {
    _velocity = collision.velocity;
    _position = collision.position;
    _fuel -= _power*collision.t;
    _status = getStatusAfterCollision(collision);
  }
  else {
    _velocity = trajectory.velocity(ts);
    _position = trajectory.position(ts);
    _fuel -= _power*ts;
    if (_fuel < _power)
      _power = _fuel;
    if (isLost())
      _status = LanderStatus::LOST;
  }
  return _status != LanderStatus::FALLING;
}

mars::LanderStatus mars::Lander::getStatusAfterCollision(const Collision& collision) const {
  bool is_good_landing = collision.segment.isHorizontal() and
         isBetween(-_max_vx, _max_vx, _velocity.x) and _velocity.y >= -_max_vy;
  return is_good_landing? LanderStatus::LANDED : LanderStatus::CRASHED;
}

