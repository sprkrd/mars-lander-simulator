#ifndef _MARS_LANDER_HPP_
#define _MARS_LANDER_HPP_

#include "common.hpp"
#include <stdexcept>

namespace mars {

const double k_default_max_pow_inc = 1;
const double k_default_max_rot_inc = 15;
const double k_default_max_power = 4;
const double k_default_rot_limit = 90;
const double k_default_max_vy = 40;
const double k_default_max_vx = 20;

class Environment;

enum class LanderStatus { FALLING, CRASHED, LOST, LANDED };

struct Action {
  double power, rotation;
};

class Lander {
  public:
    Lander();

    void setStatus(LanderStatus status);

    void setPosition(const Vector2D& position);

    void setVelocity(const Vector2D& velocity);

    void setRotation(double rotation);

    void setPower(double power);

    void setFuel(double fuel);

    void setMaxPowerIncrement(double max_pow_inc);

    void setMaxRotationIncrement(double max_rot_inc);

    void setMaxPower(double max_power);

    void setRotationLimit(double rot_limit);

    void setMaxVyForLanding(double max_vy);

    void setMaxVxForLanding(double max_vx);

    void assignEnvironment(const Environment* environment, int id);

    const Environment* getEnvironment() const;

    int getId() const;

    LanderStatus getStatus() const;

    const Vector2D& getPosition() const;

    const Vector2D& getVelocity() const;

    Vector2D getAcceleration() const;

    double getRotation() const;

    double getPower() const;

    double getFuel() const;

    double getMaxPowerIncrement() const;

    double getMaxRotationIncrement() const;

    double getMaxPower() const;

    double getRotationLimit() const;

    double getMaxVyForLanding() const;

    double getMaxVxForLanding() const;

    bool isValidAction(const Action& action, const char** msg = nullptr) const;

    bool step(const Action& action, bool check_valid = true);

  private:

    bool isLost() const;

    LanderStatus getStatusAfterCollision(const Collision& collision) const;

    const Environment* _environment;
    int _id;
    LanderStatus _status;
    Vector2D _position, _velocity;
    double _rotation, _power, _fuel, _max_pow_inc, _max_rot_inc, _max_power,
           _rot_limit, _max_vy, _max_vx;
};

} // mars namespace

#endif

