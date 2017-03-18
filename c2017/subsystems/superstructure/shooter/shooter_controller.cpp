#include "c2017/subsystems/superstructure/shooter/shooter_controller.h"
#include <cmath>
#include <limits>

namespace c2017 {
namespace shooter {

ShooterController::ShooterController()
    : shooter_status_queue_{QueueManager::GetInstance().shooter_status_queue()} {
  auto shooter_plant = muan::control::StateSpacePlant<1, 3, 1>(frc1678::shooter_controller::controller::A(),
                                                               frc1678::shooter_controller::controller::B(),
                                                               frc1678::shooter_controller::controller::C());

  shooter_controller_ = muan::control::StateSpaceController<1, 3, 1>(
      frc1678::shooter_controller::controller::K(), frc1678::shooter_controller::controller::Kff(),
      frc1678::shooter_controller::controller::A(),
      Eigen::Matrix<double, 1, 1>::Ones() * -std::numeric_limits<double>::infinity(),
      Eigen::Matrix<double, 1, 1>::Ones() * std::numeric_limits<double>::infinity());

  shooter_observer_ =
      muan::control::StateSpaceObserver<1, 3, 1>(shooter_plant, frc1678::shooter_controller::controller::L());

  accelarator_controller_ = muan::control::StateSpaceController<1, 2, 1>(
      frc1678::accelarator_controller::controller::K(), frc1678::accelarator_controller::controller::Kff(),
      frc1678::accelarator_controller::controller::A(),
      Eigen::Matrix<double, 1, 1>::Ones() * -std::numeric_limits<double>::infinity(),
      Eigen::Matrix<double, 1, 1>::Ones() * std::numeric_limits<double>::infinity());

  auto accelarator_plant = muan::control::StateSpacePlant<1, 2, 1>(
      frc1678::accelarator_controller::controller::A(), frc1678::accelarator_controller::controller::B(),
      frc1678::accelarator_controller::controller::C());
  accelarator_observer_ = muan::control::StateSpaceObserver<1, 2, 1>(
      accelarator_plant, frc1678::accelarator_controller::controller::L());

  at_goal_ = false;

  velocity_tolerance_ = 30;  // Radians per second
}

c2017::shooter::ShooterOutputProto ShooterController::Update(c2017::shooter::ShooterInputProto input,
                                                             bool outputs_enabled) {
  Eigen::Matrix<double, 3, 1> r_;
  Eigen::Matrix<double, 2, 1> accelarator_r_;

  auto y = (Eigen::Matrix<double, 1, 1>() << input->shooter_encoder_position()).finished();
  r_ = (Eigen::Matrix<double, 3, 1>() << 0.0, UpdateProfiledGoalVelocity(unprofiled_goal_velocity_), 0.0)
           .finished();

  auto accelarator_y = (Eigen::Matrix<double, 1, 1>() << input->accelarator_encoder_postition()).finished();
  accelarator_r_ = (Eigen::Matrix<double, 2, 1>() << 0.0, 0.5 * unprofiled_goal_velocity_).finished();

  shooter_controller_.r() = r_;
  accelarator_controller_.r() = accelarator_r_;

  auto u = shooter_controller_.Update(shooter_observer_.x())(0, 0);

  auto accelarator_u = accelarator_controller_.Update(accelarator_observer_.x())(0, 0);

  if (!outputs_enabled || unprofiled_goal_velocity_ <= 0) {
    u = 0.0;
    accelarator_u = 0.0;
    unprofiled_goal_velocity_ = 0.0;
  } else {
    status_->set_uncapped_u(u);
    u = CapU(u, outputs_enabled);
  }

  shooter_observer_.Update((Eigen::Matrix<double, 1, 1>() << u).finished(), y);
  accelarator_observer_.Update((Eigen::Matrix<double, 1, 1>() << accelarator_u). finished(), y);

  auto absolute_error = ((Eigen::Matrix<double, 3, 1>() << 0.0, unprofiled_goal_velocity_, 0.0).finished() -
                         shooter_observer_.x())
                            .cwiseAbs();

  at_goal_ = absolute_error(1, 0) < velocity_tolerance_;

  c2017::shooter::ShooterOutputProto output;

  output->set_shooter_voltage(u);
  output->set_accelarator_voltage(accelarator_u);
  status_->set_observed_velocity(shooter_observer_.x()(1, 0));
  status_->set_accelarator_observed_velocity(accelarator_observer_.x()(1, 0));
  status_->set_at_goal(at_goal_);
  status_->set_currently_running(std::fabs(unprofiled_goal_velocity_) >= 1e-3);
  status_->set_voltage(u);
  status_->set_profiled_goal_velocity(profiled_goal_velocity_);
  status_->set_unprofiled_goal_velocity(unprofiled_goal_velocity_);
  status_->set_voltage_error(shooter_observer_.x(2));
  QueueManager::GetInstance().shooter_status_queue().WriteMessage(status_);

  return output;
}

double ShooterController::CapU(double u, bool outputs_enabled) {
  double k2 = shooter_controller_.K(0, 1);
  double x2 = shooter_observer_.x(1);
  double k3 = shooter_controller_.K(0, 2);
  double x3 = shooter_observer_.x(2);

  double u_max = 12;
  double u_min = -12;

  if (!outputs_enabled) {
    profiled_goal_velocity_ = 0;
    return 0;
  }

  if (u > u_max) {
    profiled_goal_velocity_ = (u_max + k2 * x2 + k3 * x3) / k2;
    u = u_max;
  } else if (u < u_min) {
    profiled_goal_velocity_ = (u_min + k2 * x2 + k3 * x3) / k2;
    u = u_min;
  }

  return u;
}

void ShooterController::SetGoal(c2017::shooter::ShooterGoalProto goal) {
  unprofiled_goal_velocity_ = goal->goal_velocity();
  shot_mode_ = goal->goal_mode();
}

double ShooterController::UpdateProfiledGoalVelocity(double unprofiled_goal_velocity) {
  profiled_goal_velocity_ =
      std::min(profiled_goal_velocity_ + kShooterAcceleration, unprofiled_goal_velocity);
  return profiled_goal_velocity_;
}

}  // namespace shooter
}  // namespace c2017
