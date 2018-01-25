#include "c2018/subsystems/score_subsystem/claw/claw.h"
#include <cmath>

namespace c2018 {
namespace score_subsystem {
namespace claw {

WristController::WristController()
    : trapezoidal_motion_profile_{::std::chrono::milliseconds(5)},
      hall_calibration_{kHallMagnetPosition} {
  auto wrist_plant = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::wrist_controller::controller::A(),
      frc1678::wrist_controller::controller::B(),
      frc1678::wrist_controller::controller::C());

  wrist_controller_ = muan::control::StateSpaceController<1, 3, 1>(
      frc1678::wrist_controller::controller::K(),
      frc1678::wrist_controller::controller::Kff(),
      frc1678::wrist_controller::controller::A(),
      Eigen::Matrix<double, 1, 1>::Ones() * -12,
      Eigen::Matrix<double, 1, 1>::Ones() * 12);

  wrist_observer_ = muan::control::StateSpaceObserver<1, 3, 1>(
      wrist_plant, frc1678::wrist_controller::controller::L());

  trapezoidal_motion_profile_.set_maximum_acceleration(kMaxWristAcceleration);
  trapezoidal_motion_profile_.set_maximum_velocity(kMaxWristVelocity);
}

void WristController::SetGoal(double angle, IntakeMode mode) {
  if (claw_state_ == SYSTEM_IDLE || claw_state_ == MOVING) {
    unprofiled_goal_position_ = muan::utils::Cap(angle, -M_PI / 2, M_PI / 2);
    claw_state_ = MOVING;
  }

  intake_mode_ = mode;
}

void WristController::Update(ScoreSubsystemInputProto input,
                            ScoreSubsystemOutput* output,
                            ScoreSubsystemStatusProto* status,
                            bool outputs_enabled) {
  double calibrated_encoder =
      hall_calibration_.Update(input->wrist_encoder(), input->wrist_hall());
  auto wrist_y =
      (Eigen::Matrix<double, 1, 1>() << calibrated_encoder).finished();

  double wrist_voltage = 0;

  if (!outputs_enabled) {
    wrist_voltage = 0;
    claw_state_ = DISABLED;
  } else if (claw_state_ == DISABLED) {
    claw_state_ = SYSTEM_IDLE;
  } else if (!encoder_fault_detected_) {
    wrist_voltage = CapU(wrist_voltage);
  }

  double roller_voltage = 0;

  switch (claw_state_) {
    case SYSTEM_IDLE:
      wrist_voltage = 0;
      roller_voltage = 0;
      break;
    case ENCODER_FAULT:
      wrist_voltage = 0;
      roller_voltage= 0;
      break;
    case DISABLED:
      wrist_voltage = 0;
      roller_voltage = 0;
      break;
    case INITIALIZING:
      claw_state_ = CALIBRATING;
      break;
    case CALIBRATING:
      wrist_voltage = kCalibVoltage;
      if (hall_calibration_.is_calibrated()) {
        claw_state_ = SYSTEM_IDLE;
      }
      break;
    case MOVING:
    // Run the controller
    Eigen::Matrix<double, 3, 1> wrist_r;
    wrist_r = (Eigen::Matrix<double, 3, 1>()
              << UpdateProfiledGoal(unprofiled_goal_position_, outputs_enabled))
                 .finished();
    wrist_r.block<2, 1>(0, 0) =
        trapezoidal_motion_profile_.Update(unprofiled_goal_position_, 0.0);
    wrist_r(2) = 0;

    wrist_voltage = wrist_controller_.Update(wrist_observer_.x(), wrist_r)(0);
    break;
    }

  // Check for encoder faults
  if (old_pos_ == input->wrist_encoder()) {
    num_encoder_fault_ticks_++;
    if (num_encoder_fault_ticks_ > kEncoderFaultTicksAllowed) {
      encoder_fault_detected_ = true;
    }
  }
  old_pos_ = input->wrist_encoder();

  wrist_observer_.Update(
      (Eigen::Matrix<double, 1, 1>() << wrist_voltage).finished(), wrist_y);

  // Start of intake
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
  output->set_wrist_voltage(wrist_voltage);
  output->set_claw_pinch(claw_pinch);
  (*status)->set_wrist_calibrated(hall_calibration_.is_calibrated());
  (*status)->set_wrist_position(wrist_position_);
}

double WristController::CapU(double wrist_voltage) {
  return muan::utils::Cap(wrist_voltage, -12, 12);
}


Eigen::Matrix<double, 2, 1> WristController::UpdateProfiledGoal(
    double unprofiled_goal_, bool outputs_enabled) {
  if (outputs_enabled) {
    profiled_goal_ = trapezoidal_motion_profile_.Update(unprofiled_goal_, 0.0);
  }

  return profiled_goal_;
}

}  // namespace claw
}  // namespace score_subsystem
}  // namespace c2018
