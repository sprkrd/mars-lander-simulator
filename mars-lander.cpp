#include "mars-lander.hpp"
#include <cassert>
#include <cmath>

mars::Interval mars::Interval::createSortedInterval(double a, double b) {
  return a < b? mars::Interval{a,b} : mars::Interval{b,a};
}

bool mars::Interval::intersects(const Interval& other) const {
  return ( lo >= other.lo and lo <= other.hi ) or
         ( other.lo >= lo and other.lo <= hi );
}

bool mars::Interval::contains(double x) const {
  return isBetween(lo, hi, x);
}

mars::Vector2D mars::Vector2D::operator*(double scalar) const {
  return Vector2D{x*scalar, y*scalar};
}

mars::Vector2D mars::Vector2D::operator+(const Vector2D& vec) const {
  return Vector2D{x+vec.x, y+vec.y};
}

void mars::Vector2D::operator+=(const Vector2D& vec) {
  x += vec.x;
  y += vec.y;
}

bool mars::Segment::isHorizontal() const {
  return isBetween(-1e-6, 1e-6, end.y - begin.y);
}

double mars::Parabola::operator()(double t) const {
  return c + ( b + a*t )*t;
}

mars::Interval mars::Parabola::getBoundingInterval(double t0, double tf) const {
  Interval range{t0,tf};
  double tv = -b / (2*a);
  double v0 = (*this)(t0);
  double vf = (*this)(tf);
  double vv = range.contains(tv)? (*this)(tv) : vf;
  return Interval::createSortedInterval(
      std::max( std::max(v0, vf), vv ),
      std::min( std::min(v0, vf), vv ));
}

mars::Context::Context() :
  width(k_default_width),
  height(k_default_height),
  gravity(k_default_gravity),
  max_pow_inc(k_default_max_pow_inc),
  max_rot_inc(k_default_max_rot_inc),
  max_power(k_default_max_power),
  rot_limit(k_default_rot_limit),
  max_vy(k_default_max_vy),
  max_vx(k_default_max_vx),
  dt(k_default_dt) {
}

mars::LanderState::LanderState(const Vector2D& position,
    const Vector2D& velocity, double rotation, double power, double fuel,
    const Context* context) :
  _context(context), _status(FALLING), _position(position), _velocity(velocity),
  _rotation(rotation), _power(power), _fuel(fuel), _time(0) {
  
}

const mars::Context* mars::LanderState::getContext() const {
  return _context;
}

mars::LanderState::Status mars::LanderState::getStatus() const {
  return _status;
}

const mars::Vector2D& mars::LanderState::getPosition() const {
  return _position;
}

const mars::Vector2D& mars::LanderState::getVelocity() const {
  return _velocity;
}

mars::Vector2D mars::LanderState::getAcceleration() const {
  double rotation_rad = _rotation*M_PI/180.0;
  return { -std::sin(rotation_rad)*_power,
            std::cos(rotation_rad)*_power - _context->gravity };
}

double mars::LanderState::getRotation() const {
  return _rotation;
}

double mars::LanderState::getPower() const {
  return _power;
}

double mars::LanderState::getFuel() const {
  return _fuel;
}

double mars::LanderState::getTime() const {
  return _time;
}

bool mars::LanderState::isValidAction(const Action& action, const char** msg) const {
  double max_pow_inc = _context->max_pow_inc * _context->dt;
  if (not isBetween(-max_pow_inc, max_pow_inc, action.power-_power)) {
    if (msg) *msg = "Power increment not in legal range";
    return false;
  }
  if (not isBetween(0, std::min(_context->max_power, _fuel), action.power)) {
    if (msg) *msg = "Power value not in legal range";
    return false;
  }
  double max_rot_inc = _context->max_rot_inc * _context->dt;
  if (not isBetween(-max_rot_inc, max_rot_inc, action.rotation-_rotation)) {
    if (msg) *msg = "Rotation increment not in legal range";
    return false;
  }
  if (not isBetween(-_context->rot_limit, _context->rot_limit, action.rotation)) {
    if (msg) *msg = "Rotation value not in legal range";
    return false;
  }
  return true;
}

void mars::LanderState::step(const Action& action, bool check_valid) {
  if (check_valid) {
    const char* msg;
    if (not isValidAction(action, &msg))
      throw std::invalid_argument(msg);
  }
  _power = action.power;
  _rotation = action.rotation;
  Collision collision;
  if (collisionTest(&collision)) {
    _position = collision.position;
    _velocity = collision.velocity;
    _fuel -= _power*collision.t;
    _time += collision.t;
    bool is_good_landing = collision.segment.isHorizontal() and
                           isBetween(-_context->max_vx, _context->max_vx, _velocity.x) and
                           isBetween(-_context->max_vy, _context->max_vy, _velocity.y);
    _status = is_good_landing? LANDED : CRASHED;
  }
  else {
    auto acceleration = getAcceleration();
    _position += ( _velocity + acceleration*(0.5*_context->dt) )*_context->dt;
    _velocity += acceleration*_context->dt;
    _fuel -= _power*_context->dt;
    _time += _context->dt;
    if (_fuel < _power)
      _power = _fuel;
    bool within_bounds = isBetween(0, _context->width, _position.x) and
                         isBetween(0, _context->height, _position.y);
    if (not within_bounds)
      _status = LOST;
  }
}

mars::LanderState mars::LanderState::getSuccessor(const Action& action, bool check_valid) const {
  LanderState successor(*this);
  successor.step(action, check_valid);
  return successor;
}

std::vector<mars::Segment> mars::LanderState::fastCollisionFiltering() const {
  auto acceleration = getAcceleration();
  auto traj_bound_x = Parabola{0.5*acceleration.x, _velocity.x, _position.x}
                      .getBoundingInterval(0, _context->dt);
  auto traj_bound_y = Parabola{0.5*acceleration.y, _velocity.y, _position.y}
                      .getBoundingInterval(0, _context->dt);
  std::vector<Segment> possible_collisions;
  for (size_t i = 1; i < _context->surface.size(); ++i) {
    const auto& point0 = _context->surface[i-1];
    const auto& point1 = _context->surface[i];
    auto seg_bound_x = Interval::createSortedInterval(point0.x, point1.x);
    auto seg_bound_y = Interval::createSortedInterval(point0.y, point1.y);
    if (traj_bound_x.intersects(seg_bound_x) and traj_bound_y.intersects(seg_bound_y))
      possible_collisions.push_back({point0, point1});
  }
  return possible_collisions;
}

bool mars::LanderState::collisionTest(Collision* collision) const {
  bool collides = false;
  auto acceleration = getAcceleration();
  for (const Segment& segment : fastCollisionFiltering()) {
    double dx = segment.end.x - segment.begin.x;
    double dy = segment.end.y - segment.begin.y;
    double a = .5*(acceleration.x*dy - acceleration.y*dx);
    double b = ( _velocity.x*dy - _velocity.y*dx ) / a;
    double c = ( (_position.x-segment.begin.x)*dy - (_position.y-segment.begin.y)*dx ) / a;
    double delta = b*b-4*c;
    if (delta >= 0) {
      double sqrt_delta = std::sqrt(delta);
      double t = ( -b - sqrt_delta ) / 2;
      double u = ( _position.x + (_velocity.x + 0.5*acceleration.x*t)*t - segment.begin.x )/dx;
      if (not isBetween(0, _context->dt, t) or not isBetween(0, 1, u)) {
        t += sqrt_delta;
        u = ( _position.x + (_velocity.x + 0.5*acceleration.x*t)*t - segment.begin.x )/dx;
      }
      if (isBetween(0, _context->dt, t) and isBetween(0, 1, u)) {
        if (collision) {
          collision->t = t;
          collision->segment = segment;
          collision->position = _position + (_velocity + acceleration*(.5*t))*t;
          collision->velocity = _velocity + acceleration*t;
        }
        collides = true;
        break;
      }
    }
  }
  return collides;
}

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

void mars::Environment::setSurface(const std::vector<Vector2D>& surface) {
  _surface = surface;
}

void mars::Environment::setTimeStep(double time_step) {
  _time_step = time_step;
}

void mars::Environment::setLanderState(const State& lander_state) {
}


bool mars::isBetween(double lo, double hi, double x, double epsilon) {
  return x > lo-epsilon and x < hi+epsilon;
}

std::ostream& mars::operator<<(std::ostream& out, const Vector2D& vector) {
  return out << '(' << vector.x << ',' << vector.y << ')';
}

std::ostream& mars::operator<<(std::ostream& out, const Segment& segment) {
  return out << segment.begin << "--" << segment.end;
}

std::ostream& mars::operator<<(std::ostream& out, const Interval& interval) {
  return out << '[' << interval.lo << ',' << interval.hi << ']';
}

std::ostream& mars::operator<<(std::ostream& out, const Parabola& parabola) {
  return out << parabola.c << " + " << parabola.b << "*t + " << parabola.c << "*t^2";
}

std::ostream& mars::operator<<(std::ostream& out, LanderState::Status status) {
  switch (status) {
    case LanderState::FALLING: out << "FALLING"; break;
    case LanderState::CRASHED: out << "CRASHED"; break;
    case LanderState::LOST:    out << "LOST"; break;
    case LanderState::LANDED:  out << "LANDED"; break;
  }
  return out;
}

std::ostream& mars::operator<<(std::ostream& out, const LanderState& state) {
 return out << "{time: "      << state.getTime()
            << ", status: "   << state.getStatus()
            << ", position: " << state.getPosition()
            << ", velocity: " << state.getVelocity()
            << ", rotation: " << state.getRotation()
            << ", power: "    << state.getPower()
            << ", fuel: "     << state.getFuel() << '}';
}

