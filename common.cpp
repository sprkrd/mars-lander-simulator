#include "common.hpp"
#include <algorithm>
#include <cmath>

mars::Interval mars::Interval::createSortedInterval(double a, double b) {
  return a < b? mars::Interval{a,b} : mars::Interval{b,a};
}

bool mars::Interval::intersects(const Interval& other) const {
  return ( lo >= other.lo and lo <= other.hi ) or
         ( other.lo >= lo and other.lo <= hi );
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
  return isBetween(0, 0, end.y - begin.y, 1e-12);
}

mars::Rectangle mars::Segment::getBoundingBox() const {
  return Rectangle{
    Interval::createSortedInterval(begin.x, end.x),
    Interval::createSortedInterval(begin.y, end.y)
  };
}

bool mars::Rectangle::intersects(const Rectangle& other) const {
  return interval_x.intersects(other.interval_x) and interval_y.intersects(other.interval_y);
}

double mars::Parabola::evaluate(double t) const {
  return c + ( b + a*t )*t;
}

mars::Interval mars::Parabola::getBoundingInterval(double t0, double tf) const {
  double v0 = evaluate(t0);
  double vf = evaluate(tf);
  Interval interval = Interval::createSortedInterval(v0, vf);
  if (a != 0) {
    double tv = -b / (2*a);
    if (isBetween(t0, tf, tv)) {
      double vv = evaluate(tv);
      interval.lo = std::min(interval.lo, vv);
      interval.hi = std::max(interval.hi, vv);
    }
  }
  return interval;
}

int mars::Parabola::solveSecondDegreeEquation(double solutions[]) const {
  if (isBetween(0, 0, a, 1e-12)) {
    if (isBetween(0, 0, b, 1e-12))
      return 0;
    solutions[0] = -c / b;
    return 1;
  }
  double delta = b*b - 4*a*c;
  if (delta < 0)
    return 0;
  double sqrt_delta = std::sqrt(delta);
  solutions[0] = (-b - sqrt_delta) / (2*a);
  solutions[1] = (-b + sqrt_delta) / (2*a);
  if (solutions[1] < solutions[0])
    std::swap(solutions[0], solutions[1]);
  return 2;
}

mars::Vector2D mars::ParabolicTrajectory::position(double t) const {
  return position_0 + ( velocity_0 + acceleration*(0.5*t) )*t;
}

mars::Vector2D mars::ParabolicTrajectory::velocity(double t) const {
  return velocity_0 + acceleration*t;
}

mars::Rectangle mars::ParabolicTrajectory::getBoundingBox() const {
  return Rectangle{
    Parabola{0.5*acceleration.x, velocity_0.x, position_0.x}.getBoundingInterval(0, tmax),
    Parabola{0.5*acceleration.y, velocity_0.y, position_0.y}.getBoundingInterval(0, tmax)
  };
}

bool mars::ParabolicTrajectory::fastCollisionTest(const Segment& segment) const {
  return getBoundingBox().intersects(segment.getBoundingBox());
}

bool mars::ParabolicTrajectory::collidesWith(const Surface& surface, Collision& collision) const {
  for (unsigned i = 1; i < surface.size(); ++i) {
    Segment segment{surface[i-1], surface[i]};
    if (not fastCollisionTest(segment))
      continue;
    double dx = segment.end.x - segment.begin.x;
    double dy = segment.end.y - segment.begin.y;
    double a = .5*(acceleration.x*dy - acceleration.y*dx);
    double b = velocity_0.x*dy - velocity_0.y*dx;
    double c = (position_0.x-segment.begin.x)*dy - (position_0.y-segment.begin.y)*dx;
    double t[2];
    Parabola parabola{a,b,c};
    int num_solutions = parabola.solveSecondDegreeEquation(t);
    for (int j = 0; j < num_solutions; ++j) {
      if (not isBetween(0, tmax, t[j]))
        continue;
      Vector2D position_t = position(t[j]);
      double u = ( position_t.x - segment.begin.x ) / dx;
      if (isBetween(0, 1, u)) {
        collision.segment = segment;
        collision.position = position_t;
        collision.velocity = velocity(t[j]);
        collision.t = t[j];
        return true;
      }
    }
  }
  return false;
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

