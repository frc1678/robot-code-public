#include "c2017/subsystems/superstructure/shooter/shooter_controller.h"

namespace c2017 {

namespace shooter {

ShooterController::ShooterController()
    : shooter_status_queue_{QueueManager::GetInstance().shooter_status_queue()} {
  auto ss_plant = muan::control::StateSpacePlant<1, 3, 1>(frc1678::shooter_controller::controller::A(),
                                                          frc1678::shooter_controller::controller::B(),
                                                          frc1678::shooter_controller::controller::C());
  controller_ = muan::control::StateSpaceController<1, 3, 1>(frc1678::shooter_controller::controller::K());
  controller_.u_min() = Eigen::Matrix<double, 1, 1>::Ones() * -12.0;
  controller_.u_max() = Eigen::Matrix<double, 1, 1>::Ones() * 12.0;
  observer_ =
      muan::control::StateSpaceObserver<1, 3, 1>(ss_plant, frc1678::shooter_controller::controller::L());

  at_goal_ = false;

  angle_tolerance_ = 2;     // Radians
  velocity_tolerance_ = 2;  // Radians per second
}

c2017::shooter::ShooterOutputProto ShooterController::Update(c2017::shooter::ShooterInputProto input,
                                                             muan::wpilib::DriverStationProto ds) {
  Eigen::Matrix<double, 3, 1> r_;

  bool disabled = ds->mode() == RobotMode::DISABLED || ds->mode() == RobotMode::ESTOP;

  auto y = (Eigen::Matrix<double, 1, 1>() << input->encoder_position()).finished();
  r_ = (Eigen::Matrix<double, 3, 1>() << 0.0, goal_velocity_, 0.0).finished();

  y(0) = (input->encoder_position());

  auto u = controller_.Update(observer_.x(), r_)(0, 0);

  if (disabled || goal_velocity_ == 0) {
    u = 0.0;
  }

  observer_.Update((Eigen::Matrix<double, 1, 1>() << u).finished(), y);

  auto absolute_error = r_ - observer_.x().cwiseAbs();

  at_goal_ = (absolute_error(0, 0) < angle_tolerance_) && (absolute_error(1, 0) < velocity_tolerance_);

  c2017::shooter::ShooterOutputProto output;

  output->set_voltage(u);

  status_->set_observed_velocity(observer_.x()(1, 0));

  shooter_status_queue_.WriteMessage(status_);

  return output;
}

void ShooterController::SetGoal(c2017::shooter::ShooterGoalProto goal) {
  goal_velocity_ = goal->goal_velocity();
  shot_mode_ = goal->goal_mode();
}
}
}
