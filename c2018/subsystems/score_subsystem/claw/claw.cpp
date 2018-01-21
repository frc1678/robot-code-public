#include "c2018/subsystems/score_subsystem/claw/claw.h"
#include <cmath>

namespace c2018 {
namespace score_subsystem {
namespace claw {

ClawController::ClawController()
    : trapezoidal_motion_profile_{::std::chrono::milliseconds(5)},
      hall_calibration_{kHallMagnetPosition} {
  auto claw_plant = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::claw_controller::controller::A(),
      frc1678::claw_controller::controller::B(),
      frc1678::claw_controller::controller::C());

  claw_controller_ = muan::control::StateSpaceController<1, 3, 1>(
      frc1678::claw_controller::controller::K(),
      frc1678::claw_controller::controller::Kff(),
      frc1678::claw_controller::controller::A(),
      Eigen::Matrix<double, 1, 1>::Ones() * -12,
      Eigen::Matrix<double, 1, 1>::Ones() * 12);

  claw_observer_ = muan::control::StateSpaceObserver<1, 3, 1>(
      claw_plant, frc1678::claw_controller::controller::L());

  trapezoidal_motion_profile_.set_maximum_acceleration(kMaxClawAcceleration);
  trapezoidal_motion_profile_.set_maximum_velocity(kMaxClawVelocity);
}

void ClawController::SetGoal(double angle, IntakeMode mode) {
  if (claw_state_ == C_IDLE || claw_state_ == C_MOVING) {
    unprofiled_goal_position_ = muan::utils::Cap(angle, -M_PI / 2, M_PI / 2);
    claw_state_ = C_MOVING;
  }

  intake_mode_ = mode;
}

void ClawController::Update(ScoreSubsystemInputProto input,
                            ScoreSubsystemOutput* output,
                            ScoreSubsystemStatusProto* status,
                            bool outputs_enabled) {
  double calibrated_encoder =
      hall_calibration_.Update(input->wrist_encoder(), input->wrist_hall());
  auto claw_y =
      (Eigen::Matrix<double, 1, 1>() << calibrated_encoder).finished();

  double claw_voltage = 0;

  if (!outputs_enabled) {
    claw_voltage = 0;
    claw_state_ = C_DISABLED;
  } else if (claw_state_ == C_DISABLED) {
    claw_state_ = C_IDLE;
  } else if (!encoder_fault_detected_) {
    claw_voltage = CapU(claw_voltage, outputs_enabled);
  }

  if (claw_state_ == C_INITIALIZING) {
    claw_state_ = C_CALIBRATING;
  } else if (claw_state_ == C_CALIBRATING) {
    claw_voltage = kCalibVoltage;
    if (hall_calibration_.is_calibrated()) {
      claw_state_ = C_IDLE;
    }
  } else if (claw_state_ == C_IDLE || claw_state_ == C_MOVING) {
    // Run the controller
    Eigen::Matrix<double, 3, 1> claw_r;
    claw_r = (Eigen::Matrix<double, 3, 1>()
              << UpdateProfiledGoal(unprofiled_goal_position_, outputs_enabled))
                 .finished();
    claw_r.block<2, 1>(0, 0) =
        trapezoidal_motion_profile_.Update(unprofiled_goal_position_, 0.0);
    claw_r(2) = 0;

    claw_voltage = claw_controller_.Update(claw_observer_.x(), claw_r)(0);
  }

  // Check for encoder faults
  if (old_pos_ == input->wrist_encoder()) {
    num_encoder_fault_ticks_++;
    if (num_encoder_fault_ticks_ > kEncoderFaultTicksAllowed) {
      encoder_fault_detected_ = true;
    }
  }
  old_pos_ = input->wrist_encoder();

  claw_observer_.Update(
      (Eigen::Matrix<double, 1, 1>() << claw_voltage).finished(), claw_y);

  // Start of intake
  double roller_voltage = 0;
  bool claw_pinch = false;

  if (outputs_enabled) {
    switch (intake_mode_) {
      case INTAKE:
        roller_voltage = 12;
        claw_pinch = true;
        break;
      case OUTTAKE:
        roller_voltage = -12;
        claw_pinch = true;
        break;
      case IDLE:
        roller_voltage = 0;
        claw_pinch = false;
        break;
      case HOLD:
        roller_voltage = 0;
        claw_pinch = true;
        break;
    }
  } else {
    roller_voltage = 0;
    claw_pinch = false;
  }

  output->set_roller_voltage(roller_voltage);
  output->set_wrist_voltage(claw_voltage);
  output->set_claw_pinch(claw_pinch);
  (*status)->set_wrist_position(wrist_position_);
}

double ClawController::CapU(double claw_voltage, bool outputs_enabled) {
  double voltage;
  if (!outputs_enabled) {
    voltage = 0;
  } else if (claw_voltage > 12) {
    voltage = 12;
  } else if (claw_voltage < -12) {
    voltage = -12;
  } else {
    voltage = claw_voltage;
  }

  return voltage;
}


Eigen::Matrix<double, 2, 1> ClawController::UpdateProfiledGoal(
    double unprofiled_goal_, bool outputs_enabled) {
  if (outputs_enabled) {
    profiled_goal_ = trapezoidal_motion_profile_.Update(unprofiled_goal_, 0.0);
  }

  return profiled_goal_;
}

}  // namespace claw
}  // namespace score_subsystem
}  // namespace c2018
