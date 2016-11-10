#include "stop_pid.h"
#include <iostream>

namespace o2016 {

namespace catapult {

StopPid::StopPid() {
  controller_ = muan::PidController(100., 0., 1.);
  done = false;
}

Voltage StopPid::Update(Angle goal, Angle sensor_value) {
  angle_ = sensor_value;
  done = (goal - sensor_value) < angle_tolerance && (goal - sensor_value) > -angle_tolerance &&
          controller_.GetDerivative() < velocity_tolerance && controller_.GetDerivative() > -velocity_tolerance;
  return controller_.Calculate(.05, goal - sensor_value);
}

Angle StopPid::get_angle() const {
  return angle_;
}

AngularVelocity StopPid::get_angular_velocity() const {
  return controller_.GetDerivative();
}

void StopPid::set_angle(Angle theta) {
  angle_ = theta;
}

bool StopPid::is_done() const {
  return done;
}

} // catapult

} // o2016
