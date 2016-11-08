#include "intake_controller.h"

using namespace muan::control;
using namespace muan::units;

namespace o2016 {

namespace intake {

using namespace ::muan::control;
using namespace ::frc1678::intake_controller;

IntakeController::IntakeController() : calibration_(0.0){

  auto ss_plant = StateSpacePlant<1, 3, 1> (controller::A(), controller::B(), controller::C());
  controller_ = StateSpaceController<1, 3, 1> (controller::K());
  controller_.u_min() = Eigen::Matrix<double, 1, 1>::Ones() * -12.0;
  controller_.u_max() = Eigen::Matrix<double, 1, 1>::Ones() * 12.0;
  observer_ = StateSpaceObserver<1, 3, 1> (ss_plant, controller::L());
 
  angle_tolerance_ = 1 * deg;
  velocity_tolerance_ = 5 * deg / s;

  at_goal_ = false;

}

muan::units::Voltage IntakeController::Update(muan::units::Angle goal, muan::units::Angle sensor_input, bool index_click) {
  Eigen::Matrix<double, 3, 1> r_;

  auto y = (Eigen::Matrix<double, 1, 1>() << sensor_input).finished();
  r_ = (Eigen::Matrix<double, 3, 1>() << goal, 0.0, 0.0).finished();
  
  y(0) = calibration_.Update(sensor_input, index_click) - .31;
  
  auto u = calibration_.is_calibrated() ? controller_.Update(observer_.x(), r_)(0, 0) : -1.0 * V;

  observer_.Update((Eigen::Matrix<double, 1, 1>() << u).finished(), y);

  auto absolute_error = (r_ - observer_.x()).cwiseAbs();

  at_goal_ = (absolute_error(0, 0) < angle_tolerance_) && (absolute_error(1, 0) < velocity_tolerance_);

  return u;
}

muan::units::Angle IntakeController::GetAngle() const {
  return observer_.x()(0, 0);
}

void IntakeController::SetAngle(Angle theta) {
  observer_.x()(0, 0) = theta;
}

bool IntakeController::AtGoal() const {
  return at_goal_;
}

bool IntakeController::is_calibrated() const {
  return calibration_.is_calibrated();
}

}

}
