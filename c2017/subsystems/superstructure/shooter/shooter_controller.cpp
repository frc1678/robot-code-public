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

  accelerator_controller_ = muan::control::StateSpaceController<1, 2, 1>(
      frc1678::accelerator_controller::controller::K(), frc1678::accelerator_controller::controller::Kff(),
      frc1678::accelerator_controller::controller::A(),
      Eigen::Matrix<double, 1, 1>::Ones() * -std::numeric_limits<double>::infinity(),
      Eigen::Matrix<double, 1, 1>::Ones() * std::numeric_limits<double>::infinity());

  auto accelerator_plant = muan::control::StateSpacePlant<1, 2, 1>(
      frc1678::accelerator_controller::controller::A(), frc1678::accelerator_controller::controller::B(),
      frc1678::accelerator_controller::controller::C());
  accelerator_observer_ = muan::control::StateSpaceObserver<1, 2, 1>(
      accelerator_plant, frc1678::accelerator_controller::controller::L());
  plant_ = muan::control::StateSpacePlant<1, 3, 1>(frc1678::shooter_controller::controller::A(),
                                                   frc1678::shooter_controller::controller::B(),
                                                   frc1678::shooter_controller::controller::C());
}

c2017::shooter::ShooterOutputProto ShooterController::Update(c2017::shooter::ShooterInputProto input,
                                                             bool outputs_enabled) {
  outputs_enabled_ = outputs_enabled;
  Eigen::Matrix<double, 3, 1> shooter_r_;
  Eigen::Matrix<double, 2, 1> accelerator_r_;

  auto shooter_y = (Eigen::Matrix<double, 1, 1>() << input->shooter_encoder_position()).finished();
  shooter_r_ = (Eigen::Matrix<double, 3, 1>() << 0.0, UpdateProfiledGoalVelocity(unprofiled_goal_velocity_),
                0.0).finished();

  auto accelerator_y = (Eigen::Matrix<double, 1, 1>() << input->accelerator_encoder_position()).finished();
  accelerator_r_ = (Eigen::Matrix<double, 2, 1>() << 0.0, 0.5 * unprofiled_goal_velocity_).finished();

  shooter_controller_.r() = shooter_r_;
  accelerator_controller_.r() = accelerator_r_;

  auto shooter_u = shooter_controller_.Update(shooter_observer_.x())(0, 0);

  auto accelerator_u = accelerator_controller_.Update(accelerator_observer_.x())(0, 0);

  if (!outputs_enabled_ || unprofiled_goal_velocity_ <= 0) {
    shooter_u = 0.0;
    accelerator_u = 0.0;
    unprofiled_goal_velocity_ -= kShooterAcceleration;
    if (unprofiled_goal_velocity_ < 0.0) {
      unprofiled_goal_velocity_ = 0.0;
    }
  } else {
    if (!encoder_fault_detected_) {
      status_->set_uncapped_u(shooter_u);
      shooter_u = CapU(shooter_u);
      accelerator_u = CapU(accelerator_u);
    } else {
      status_->set_uncapped_u(kShooterOpenLoopU);
      shooter_u = CapU(kShooterOpenLoopU);
      accelerator_u = CapU(kAcceleratorOpenLoopU);
    }
    if (plant_.x()(1, 0) > kMinimalWorkingVelocity && old_pos_ == input->shooter_encoder_position()) {
      num_encoder_fault_ticks_++;
      if (num_encoder_fault_ticks_ > kEncoderFaultTicksAllowed) {
        encoder_fault_detected_ = true;
      }
    }
  }
  old_pos_ = input->shooter_encoder_position();

  shooter_observer_.Update((Eigen::Matrix<double, 1, 1>() << shooter_u).finished(), shooter_y);
  accelerator_observer_.Update((Eigen::Matrix<double, 1, 1>() << accelerator_u).finished(), accelerator_y);
  plant_.Update((Eigen::Matrix<double, 1, 1>() << shooter_u).finished());

  auto absolute_error = ((Eigen::Matrix<double, 3, 1>() << 0.0, unprofiled_goal_velocity_, 0.0).finished() -
                         shooter_observer_.x()).cwiseAbs();

  if (unprofiled_goal_velocity_ == 0.0) {
    state_ = IDLE;
  }

  switch (state_) {
    case IDLE:
      if (unprofiled_goal_velocity_ != 0.0) {
        state_ = SPINUP;
      }
      break;
    case SPINUP:
      if (absolute_error(1, 0) < kSpinupVelocityTolerance || encoder_fault_detected_) {
        state_ = AT_GOAL;
      }
      break;
    case AT_GOAL:
      if (absolute_error(1, 0) > kSteadyStateVelocityTolerance && !encoder_fault_detected_) {
        state_ = SPINUP;
      }
      break;
  }

  c2017::shooter::ShooterOutputProto output;

  output->set_shooter_voltage(shooter_u);
  output->set_accelerator_voltage(accelerator_u);
  status_->set_observed_velocity(shooter_observer_.x()(1, 0));
  status_->set_accelerator_observed_velocity(accelerator_observer_.x()(1, 0));
  status_->set_state(state_);
  status_->set_currently_running(std::fabs(unprofiled_goal_velocity_) >= 1e-3);
  status_->set_voltage(shooter_u);
  status_->set_profiled_goal_velocity(profiled_goal_velocity_);
  status_->set_unprofiled_goal_velocity(unprofiled_goal_velocity_);
  status_->set_voltage_error(shooter_observer_.x(2));
  status_->set_encoder_fault_detected(encoder_fault_detected_);
  QueueManager::GetInstance().shooter_status_queue().WriteMessage(status_);

  return output;
}

double ShooterController::CapU(double u) {
  double k2 = shooter_controller_.K(0, 1);
  double x2 = shooter_observer_.x(1);
  double k3 = shooter_controller_.K(0, 2);
  double x3 = shooter_observer_.x(2);

  double u_max = 12;
  double u_min = -12;

  if (!outputs_enabled_) {
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
  if (outputs_enabled_) {
    unprofiled_goal_velocity_ = goal->goal_velocity();
  }
}

double ShooterController::UpdateProfiledGoalVelocity(double unprofiled_goal_velocity) {
  profiled_goal_velocity_ =
      std::min(profiled_goal_velocity_ + kShooterAcceleration, unprofiled_goal_velocity);
  return profiled_goal_velocity_;
}

}  // namespace shooter
}  // namespace c2017
