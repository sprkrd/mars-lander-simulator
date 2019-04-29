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
  return lo <= x and x <= hi;
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

double mars::Parabola::operator()(double t) const {
  return c + ( b + a*t )*t;
}

mars::Interval mars::Parabola::getBoundingInterval(double t0, double tf) const {
  Interval range{t0,tf};
  double tv = -b / (2*a);
  double v0 = (*this)(t0);
  double vf = (*this)(tf);
  double vv = range.contains(tv)? (*this)(tv) : vf;
  return Interval{std::max( std::max(v0, vf), vv ),
                  std::min( std::min(v0, vf), vv )};
}

mars::World::World() :
  width(k_default_width),
  height(k_default_height),
  gravity(k_default_gravity) {

}

mars::LanderConstraints::LanderConstraints() :
  max_power_increment(k_default_max_power_increment),
  max_rotation_increment(k_default_max_rotation_increment),
  power_limit(k_default_power_limit),
  rotation_limit(k_default_power_limit) {

}

mars::Context::Context(double simulation_step) : dt(simulation_step) {
}

mars::Vector2D mars::LanderState::getAcceleration() const {
  double rotation_rad = rotation*M_PI/180.0;
  return {std::sin(rotation_rad)*power,
          context->world.gravity + std::cos(rotation_rad)*power};
}

bool mars::LanderState::isValidAction(const Action& action, const char** msg) const {
  double dt = context->dt;
  double max_pow_inc = dt*context->lander_constraints.max_power_increment;
  double max_rot_inc = dt*context->lander_constraints.max_rotation_increment;
  double pow_lim = context->lander_constraints.power_limit;
  double rot_lim = context->lander_constraints.rotation_limit;
  Interval pow_inc_range{-max_pow_inc, max_pow_inc};
  if (not pow_inc_range.contains(action.power - power)) {
    if (msg) *msg = "Power increment not in legal range";
    return false;
  }
  Interval pow_range{0, pow_lim};
  if (not pow_range.contains(action.power)) {
    if (msg) *msg = "Power not in legal range";
    return false;
  }
  Interval rot_inc_range{-max_rot_inc, max_rot_inc};
  if (not rot_inc_range.contains(action.rotation - rotation)) {
    if (msg) *msg = "Rotation increment not in legal range";
    return false;
  }
  Interval rot_range{-rot_lim, rot_lim};
  if (not rot_range.contains(action.rotation)) {
    if (msg) *msg = "Rotation not in legal range";
    return false;
  }
  return true;
}

void mars::LanderState::step(const Action& action, bool check_valid_action) {
  if (check_valid_action) {
    const char* msg;
    if (not isValidAction(action, &msg))
      throw std::invalid_argument(msg);
  }
  double dt = context->dt;
  power = action.power;
  rotation = action.rotation;
  auto acceleration = getAcceleration();
  position += ( speed + acceleration*(0.5*dt) ) * dt;
  speed += acceleration*dt;
}

mars::LanderState mars::LanderState::getSuccessor(const Action& action, bool check_valid_action) const {
  LanderState successor = *this;
  successor.step(action, check_valid_action);
  return successor;
}

namespace {

bool fastCollisionTest(const LanderState& state, const Segment& segment) {
  using namespace mars;
  double dt = state.context->dt;
  auto acceleration = state.getAcceleration();
  auto traj_bound_x_x = Parabola{0.5*acceleration.x, state.speed.x,
    state.position.x}.getBoundingInterval(0, dt);
  auto traj_bound_y = Parabola{0.5*acceleration.y, state.speed.y,
    state.position.y}.getBoundingInterval(0, dt);
  auto seg_bound_x = Interval::createSortedInterval{segment.begin.x, segment.end.x};
  auto seg_bound_y = Interval::createSortedInterval{segment.begin.y, segment.end.y};
  return traj_bound_x.intersects(seg_bound_x) and
         traj_bound_y.intersects(seg_bound_y);
}

bool collisionTest(const LanderState& state, const Segment& segment,
                   Collision& collision);

bool collisionTest(const LanderState& state, Collision& collision);

}

//bool fastCollisionTest(const LanderState& state, const Segment& segment) {
  //auto acceleration = state
  //Parabola parabola_x{0.};
//}

//bool mars::collisionTest(const LanderState& state,
    //const Segment& segment, double dt, Collision* collision) {
  //auto acceleration = state.getAcceleration();
  //double dx = segment.end.x - segment.begin.x;
  //double dy = segment.end.y - segment.begin.y;
  //double a = .5*(acceleration.x*dy - acceleration.y*dx);
  //double b = ( state.speed.x*dy - state.speed.y*dx ) / a;
  //double c = ( (state.position.x - segment.begin.x)*dy -
             //(state.position.y - segment.begin.y)*dx ) / a;
  //double delta = b*b-4*c;
  //if (delta < 0)
    //return false;
  //double sqrt_delta = std::sqrt(delta);
  //double t = ( -b - sqrt_delta ) / 2;
  //double u = ( state.position.x + ( state.speed.x + .5*acceleration.x*t )*t -
               //segment.begin.x ) / dx;

  //Interval t_interval{0, dt};
  //Interval u_interval{0, 1};

  //if (not t_interval.contains(t) or not u_interval.contains(u)) {
    //t += sqrt_delta;
    //u = ( state.position.x + ( state.speed.x + .5*acceleration.x*t )*t -
          //segment.begin.x ) / dx;
  //}
  //if (not t_interval.contains(t) or not u_interval.contains(u))
    //return false;
  //if (collision) {
    //collision->t = t;
    //collision->segment = segment;
    //collision->position = segment.begin + Vector2D{dx,dy}*u;
    //collision->speed = state.speed + acceleration*t;
  //}
  //return true;
//}

//bool mars::collisionTest(const LanderState& state,
    //const std::vector<Vector2D>& surface, double dt, Collision* collision) {

//}

std::ostream& mars::operator<<(std::ostream& out, const Vector2D& vector) {
  return out << '(' << vector.x << ',' << vector.y << ')';
}

std::ostream& mars::operator<<(std::ostream& out, const Segment& segment) {
  return out << segment.begin << "->" << segment.end;
}

std::ostream& mars::operator<<(std::ostream& out, const Interval& interval) {
  return out << '[' << interval.lo << ',' << interval.hi << ']';
}

std::ostream& mars::operator<<(std::ostream& out, const Parabola& parabola) {
  return out << parabola.c << " + " << parabola.b << "*t + " << parabola.c << "*t^2";
}

std::ostream& mars::operator<<(std::ostream& out, LanderStatus status) {
  switch (status) {
    case LanderStatus::FALLING: out << "FALLING"; break;
    case LanderStatus::CRASHED: out << "CRASHED"; break;
    case LanderStatus::LOST: out << "LOST"; break;
    case LanderStatus::LANDED: out << "LANDED"; break;
  }
  return out;
}

std::ostream& mars::operator<<(std::ostream& out, const LanderState& state) {
 return out << "{status: " << state.status << ", position: " << state.position
            << ", speed: " << state.speed << ", rotation: " << state.rotation
            << ", power: " << state.power << ", fuel: " << state.fuel << '}';
}

