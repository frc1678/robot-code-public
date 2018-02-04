#include <cmath>
#include "c2018/subsystems/score_subsystem/wrist/wrist.h"

namespace c2018 {
namespace score_subsystem {
namespace wrist {

using muan::queues::QueueManager;

WristController::WristController()
    : trapezoidal_motion_profile_{::std::chrono::milliseconds(5)},
      status_queue_{QueueManager<ScoreSubsystemStatusProto>::Fetch()},
      output_queue_{QueueManager<ScoreSubsystemOutputProto>::Fetch()} {
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

void WristController::SetGoal(double wrist_angle, IntakeMode intake_mode) {
  unprofiled_goal_ = wrist_angle;
  intake_mode_ = intake_mode;
}

void WristController::Update(ScoreSubsystemInputProto input,
                             ScoreSubsystemOutputProto* output,
                             ScoreSubsystemStatusProto* status,
                             bool outputs_enabled) {
  double calibrated_encoder =
      hall_calibration_.Update(input->wrist_encoder(), input->wrist_hall());
  auto wrist_y =
      (Eigen::Matrix<double, 1, 1>() << calibrated_encoder).finished();

  double wrist_voltage = 0.0;

  if (!outputs_enabled) {
    wrist_voltage = 0.0;
    trapezoidal_motion_profile_.MoveCurrentState(
        wrist_observer_.x().block<2, 1>(0, 0));
  }

  // Start of intake
  bool wrist_solenoid_close = false;
  bool wrist_solenoid_open = false;

  if (outputs_enabled) {
    switch (intake_mode_) {
      case INTAKE:
        intake_voltage = kIntakeVoltage;
        wrist_solenoid_close = false;
        wrist_solenoid_open = false;
        break;
      case OUTTAKE:
        intake_voltage = kOuttakeVoltage;
        wrist_solenoid_close = false;
        wrist_solenoid_open = false;
        break;
      case IDLE:
        intake_voltage = 0;
        wrist_solenoid_close = true;
        wrist_solenoid_open = false;
        break;
    }
  } else {
    intake_voltage = 0;
  }

  if (!hall_calibration_.is_calibrated()) {
    wrist_voltage = kCalibVoltage;
  } else {
    plant_.x()(0, 0) += hall_calibration_.offset();
  }

  Eigen::Matrix<double, 3, 1> wrist_r =
      (Eigen::Matrix<double, 3, 1>()
           << UpdateProfiledGoal(unprofiled_goal_, outputs_enabled)(0, 0),
       0.0, 0.0)
          .finished();

  wrist_controller_.r() = wrist_r;

  wrist_voltage = wrist_controller_.Update(wrist_observer_.x(), wrist_r)(0, 0);

  // Check for encoder faults
  if (old_pos_ == input->wrist_encoder() && std::abs(wrist_voltage) > 2) {
    num_encoder_fault_ticks_++;
    if (num_encoder_fault_ticks_ > kEncoderFaultTicksAllowed) {
      encoder_fault_detected_ = true;
    }
  } else {
    num_encoder_fault_ticks_ = 0;
  }
  old_pos_ = input->wrist_encoder();

  if (!encoder_fault_detected_) {
    wrist_voltage = muan::utils::Cap(wrist_voltage, -kMaxVoltage, kMaxVoltage);
  } else {
    wrist_voltage = 0;
  }

  wrist_observer_.Update(
      (Eigen::Matrix<double, 1, 1>() << wrist_voltage).finished(), wrist_y);

  current_monitor_.Update(input->intake_current(), input->intake_current());

  if (current_monitor_.is_at_thresh()) {
    intake_voltage = 0;
    (*status)->set_has_cube(true);
  } else {
    (*status)->set_has_cube(false);
  }

  (*output)->set_intake_voltage(intake_voltage);
  (*output)->set_wrist_voltage(wrist_voltage);
  (*output)->set_wrist_solenoid_open(wrist_solenoid_open);
  (*output)->set_wrist_solenoid_close(wrist_solenoid_close);
  (*status)->set_wrist_calibrated(hall_calibration_.is_calibrated());
  (*status)->set_wrist_angle(wrist_observer_.x()(0, 0));
  (*status)->set_has_cube(input->intake_current() > kStallCurrent);
}  // namespace wrist

Eigen::Matrix<double, 2, 1> WristController::UpdateProfiledGoal(
    double unprofiled_goal_, bool outputs_enabled) {
  if (outputs_enabled) {
    profiled_goal_ = trapezoidal_motion_profile_.Update(unprofiled_goal_, 0.0);
  }

  return profiled_goal_;
}

}  // namespace wrist
}  // namespace score_subsystem
}  // namespace c2018
