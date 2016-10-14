#include "stop.h"

using namespace muan::control;
using namespace frc1678::stop;

CatapultStop::CatapultStop() {
  auto plant = StateSpacePlant<1, 2, 1> (controller::A(), controller::B(), controller::C());
  controller_ = StateSpaceController<1, 2, 1> (controller::K());
  controller_.u_min() = Eigen::Matrix<double, 1, 1>::Ones() * -12;
  controller_.u_max() = Eigen::Matrix<double, 1, 1>::Ones() * 12;
  observer_ = StateSpaceObserver<1, 2, 1> (plant, controller::L());
  done = false;
}

Voltage CatapultStop::Update(Angle goal, Angle sensor_value) {
  Eigen::Matrix<double, 2, 1> r;
  auto y = (Eigen::Matrix<double, 1, 1>() << sensor_value).finished();
  r = (Eigen::Matrix<double, 2, 1>() << goal, 0.0).finished();
  auto u = controller_.Update(observer_.x(), r);
  observer_.Update(u, y);
  auto abs_error = (r - observer_.x()).cwiseAbs();
  done = abs_error(0, 0) < angle_tolerance && abs_error(1, 0) < velocity_tolerance;
  return u(0, 0);
}

Angle CatapultStop::get_angle() const {
  return observer_.x()(0, 0);
}

AngularVelocity CatapultStop::get_angular_velocity() const {
  return observer_.x()(1, 0);
}

void CatapultStop::set_angle(Angle theta) {
  observer_.x()(0, 0) = theta;
}

bool CatapultStop::is_done() const {
  return done;
}
