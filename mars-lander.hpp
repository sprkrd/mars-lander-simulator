#ifndef _MARS_LANDER_HPP_
#define _MARS_LANDER_HPP_

#include <vector>
#include <ostream>

namespace mars {

const double k_default_width = 7000;
const double k_default_height = 3000;
const double k_default_gravity = 3.711;
const double k_default_max_power_increment = 1;
const double k_default_max_rotation_increment = 15;
const double k_default_power_limit = 4;
const double k_default_rotation_limit = 90;

struct Interval {
  double lo,hi;
  static Interval createSortedInterval(double a, double b);
  bool intersects(const Interval& other) const;
  bool contains(double x) const;
};

struct Vector2D {
  double x, y;
  Vector2D operator*(double scalar) const;
  Vector2D operator+(const Vector2D& vec) const;
  void operator+=(const Vector2D& vec);
};

struct Segment {
  Vector2D begin, end;
};

struct Parabola {
  double a, b, c;
  double operator()(double t) const;
  Interval getBoundingInterval(double t0, double tf) const;
};

struct World {
  double width, height, gravity;
  std::vector<Vector2D> surface;
  World();
};

struct LanderConstraints {
  double max_power_increment, max_rotation_increment,
         power_limit, rotation_limit;
  LanderConstraints();
};

struct Context {
  World world;
  LanderConstraints lander_constraints;
  double dt; // simulation step
  Context(double simulation_step = 1);
};

enum class LanderStatus { FALLING, CRASHED, LOST, LANDED };

struct Action {
  double rotation;
  double power;
};

struct Collision {
  double t;
  Segment segment;
  Vector2D position, speed;
};

struct LanderState {
  const Context* context;
  LanderStatus status;
  Vector2D position, speed;
  double rotation, power, fuel;
  Vector2D getAcceleration() const;
  bool isValidAction(const Action& action, const char** msg = nullptr) const;
  void step(const Action& action, bool check_valid_action = true);
  LanderState getSuccessor(const Action& action, bool check_valid_action = true) const;
};

//bool fastCollisionTest(const LanderState& state, const Segment& segment);

//bool collisionTest(const LanderState& state, const Segment& segment, double dt,
    //Collision* collision = nullptr);

//bool collisionTest(const LanderState& state,
    //const std::vector<Vector2D>& surface, double dt,
    //Collision* collision = nullptr);

std::ostream& operator<<(std::ostream& out, const Vector2D& vector);

std::ostream& operator<<(std::ostream& out, const Segment& segment);

std::ostream& operator<<(std::ostream& out, const Interval& interval);

std::ostream& operator<<(std::ostream& out, const Parabola& parabola);

std::ostream& operator<<(std::ostream& out, LanderStatus status);

std::ostream& operator<<(std::ostream& out, const LanderState& state);

} // mars namespace

#endif

