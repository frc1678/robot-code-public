#include "c2017/subsystems/superstructure/shooter/shooter_controller.h"

namespace c2017 {

namespace shooter {

using namespace ::muan::control;
using namespace ::frc1678::shooter_controller;

ShooterController::ShooterController() {
  auto ss_plant = StateSpacePlant<1, 3, 1>(controller::A(), controller::B(), controller::C());
  controller_ = StateSpaceController<1, 3, 1>(controller::K());
  controller_.u_min() = Eigen::Matrix<double, 1, 1>::Ones() * -12.0;
  controller_.u_max() = Eigen::Matrix<double, 1, 1>::Ones() * 12.0;
  observer_ = StateSpaceObserver<1, 3, 1>(ss_plant, controller::L());

  at_goal_ = false;

  angle_tolerance_ = 2;     // Radians
  velocity_tolerance_ = 2;  // Radians per second
}

c2017::shooter::ShooterOutputProto ShooterController::Update(c2017::shooter::ShooterInputProto input,
                                                             muan::wpilib::DriverStationProto ds) {
  Eigen::Matrix<double, 3, 1> r_;

  bool disabled = ds->mode() == RobotMode::DISABLED || ds->mode() == RobotMode::ESTOP;

  auto y = (Eigen::Matrix<double, 1, 1>() << input->encoder_position()).finished();
  r_ = (Eigen::Matrix<double, 3, 1>() << 0.0, goal_->goal_velocity(), 0.0).finished();

  y(0) = (input->encoder_position());

  auto u = controller_.Update(observer_.x(), r_)(0, 0);

  if (disabled) {
    u = 0.0;
  }

  observer_.Update((Eigen::Matrix<double, 1, 1>() << u).finished(), y);

  auto absolute_error = r_ - observer_.x().cwiseAbs();

  at_goal_ = (absolute_error(0, 0) < angle_tolerance_) && (absolute_error(1, 0) < velocity_tolerance_);

  c2017::shooter::ShooterOutputProto output;
  if (goal_->goal_velocity() > 0) {
    output->set_voltage(u);
  } else {
    output->set_voltage(0);
  }
  status_->set_observed_velocity(observer_.x()(1, 0));

  return output;
}

c2017::shooter::ShooterStatusProto ShooterController::get_status() { return status_; }

void ShooterController::SetGoal(c2017::shooter::ShooterGoalProto goal) { goal_ = goal; }
}
}
