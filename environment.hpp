#ifndef _MARS_ENVIRONMENT_HPP_
#define _MARS_ENVIRONMENT_HPP_

#include "lander.hpp"
#include <vector>

namespace mars {

const double k_default_width = 7000;
const double k_default_height = 3000;
const double k_default_gravity = 3.711;
const double k_default_time_step = 1.0/30;

class Environment {
  public:
    Environment();

    Environment(const Environment& other) = delete;

    Environment(Environment&& other) = delete;

    Environment& operator=(const Environment& other) = delete;

    Environment& operator=(Environment&& other) = delete;

    void setWidth(double width);

    void setHeight(double height);

    void setGravity(double gravity);

    void setTimeStep(double time_step);

    void setSurface(const std::vector<Vector2D>& surface);

    void reset();

    Lander& createLander();

    Lander& getLander(int id);

    const Lander& getLander(int id) const;

    int getNumberOfLanders() const;

    double getWidth() const;

    double getHeight() const;

    double getGravity() const;

    double getTotalTime() const;

    double getTimeStep() const;

    const std::vector<Vector2D>& getSurface() const;

    bool stepIdle();

    bool step(const std::vector<Action>& action_v, bool check_valid = true);

  private:
    std::vector<Lander> _landers;
    double _width, _height, _gravity, _total_time, _time_step;
    std::vector<Vector2D> _surface;
};
  

} // mars namespace

#endif

