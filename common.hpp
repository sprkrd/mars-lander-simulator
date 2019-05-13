#ifndef _MARS_COMMON_HPP_
#define _MARS_COMMON_HPP_

#include <ostream>
#include <vector>

namespace mars {

struct Interval {
  double lo,hi;

  static Interval createSortedInterval(double a, double b);

  bool intersects(const Interval& other) const;
};

struct Vector2D {
  double x, y;

  Vector2D operator*(double scalar) const;

  Vector2D operator+(const Vector2D& vec) const;

  void operator+=(const Vector2D& vec);
};

typedef std::vector<Vector2D> Surface;

struct Rectangle {
  Interval interval_x, interval_y;

  bool intersects(const Rectangle& other) const;
};

struct Segment {
  Vector2D begin, end;

  bool isHorizontal() const;

  Rectangle getBoundingBox() const;
};

struct Parabola {
  double a, b, c;

  double evaluate(double t) const;

  Interval getBoundingInterval(double t0, double tf) const;

  int solveSecondDegreeEquation(double solutions[]) const;
};

struct Collision {
  Segment segment;
  Vector2D position, velocity;
  double t;
};

struct ParabolicTrajectory {
  Vector2D acceleration, velocity_0, position_0;
  double tmax;

  Vector2D position(double t) const;

  Vector2D velocity(double t) const;

  Rectangle getBoundingBox() const;

  bool fastCollisionTest(const Segment& segment) const;

  bool collidesWith(const Surface& surface, Collision& collision) const;
};

bool isBetween(double lo, double hi, double x, double epsilon = 1e-12);

std::ostream& operator<<(std::ostream& out, const Vector2D& vector);

std::ostream& operator<<(std::ostream& out, const Segment& segment);

std::ostream& operator<<(std::ostream& out, const Interval& interval);

std::ostream& operator<<(std::ostream& out, const Parabola& parabola);

} // mars namespace

#endif

