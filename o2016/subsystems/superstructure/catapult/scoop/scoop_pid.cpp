#include "scoop_pid.h"

namespace o2016 {

namespace catapult {

using namespace ::frc1678::scoop;

ScoopPid::ScoopPid() {
  controller_ = muan::PidController(18., 0., 1.);
  done = false;
}

Voltage ScoopPid::Update(Angle goal, Angle sensor_value) {
  angle_ = sensor_value;
  done = (goal - sensor_value) < angle_tolerance && (goal - sensor_value) > -angle_tolerance &&
          controller_.GetDerivative() < velocity_tolerance && controller_.GetDerivative() > -velocity_tolerance;
  return controller_.Calculate(.05, goal - sensor_value);
}

Angle ScoopPid::get_angle() const {
  return angle_;
}

AngularVelocity ScoopPid::get_angular_velocity() const {
  return controller_.GetDerivative();
}

void ScoopPid::set_angle(Angle theta) {
  angle_ = theta;
}

bool ScoopPid::is_done() const {
  return done;
}

} // catapult

} // o2016
