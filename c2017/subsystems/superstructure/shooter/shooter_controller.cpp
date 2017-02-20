#include "c2017/subsystems/superstructure/shooter/shooter_controller.h"
#include <cmath>

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

  velocity_tolerance_ = 100;  // Radians per second
}

c2017::shooter::ShooterOutputProto ShooterController::Update(c2017::shooter::ShooterInputProto input,
                                                             bool outputs_enabled) {
  Eigen::Matrix<double, 3, 1> r_;

  auto y = (Eigen::Matrix<double, 1, 1>() << input->encoder_position()).finished();
  r_ = (Eigen::Matrix<double, 3, 1>() << 0.0, update_profiled_goal_velocity(unprofiled_goal_velocity_), 0.0)
           .finished();

  y(0) = (input->encoder_position());

  auto u = controller_.Update(observer_.x(), r_)(0, 0);

  if (!outputs_enabled || unprofiled_goal_velocity_ <= 0) {
    u = 0.0;
  }

  observer_.Update((Eigen::Matrix<double, 1, 1>() << u).finished(), y);

  auto absolute_error =
      ((Eigen::Matrix<double, 3, 1>() << 0.0, unprofiled_goal_velocity_, 0.0).finished() - observer_.x())
          .cwiseAbs();

  at_goal_ = absolute_error(1, 0) < velocity_tolerance_;

  c2017::shooter::ShooterOutputProto output;

  output->set_voltage(u);

  if (shot_mode_ == ShotMode::FENDER) {
    output->set_hood_solenoid(false);
  } else {
    output->set_hood_solenoid(true);
  }

  status_->set_observed_velocity(observer_.x()(1, 0));
  status_->set_at_goal(at_goal_);
  status_->set_currently_running(std::fabs(unprofiled_goal_velocity_) >= 1e-3);
  status_->set_voltage(u);
  status_->set_profiled_goal_velocity(profiled_goal_velocity_);
  status_->set_unprofiled_goal_velocity(unprofiled_goal_velocity_);
  status_->set_voltage_error(observer_.x(2));
  QueueManager::GetInstance().shooter_status_queue().WriteMessage(status_);

  return output;
}

void ShooterController::SetGoal(c2017::shooter::ShooterGoalProto goal) {
  unprofiled_goal_velocity_ = goal->goal_velocity();
  shot_mode_ = goal->goal_mode();
}

double ShooterController::update_profiled_goal_velocity(double unprofiled_goal_velocity) {
  profiled_goal_velocity_ =
      std::min(profiled_goal_velocity_ + kShooterAcceleration, unprofiled_goal_velocity);
  return profiled_goal_velocity_;
}

}  // namespace shooter
}  // namespace c2017
