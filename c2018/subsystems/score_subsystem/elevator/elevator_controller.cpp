#include "c2018/subsystems/score_subsystem/elevator/elevator_controller.h"
#include <cmath>
#include <limits>
#include "muan/logging/logger.h"
#include "muan/utils/math_utils.h"
namespace c2018 {

namespace score_subsystem {

namespace elevator {

ElevatorController::ElevatorController() {
  auto elevator_plant = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::elevator_controller::controller::first_stage_integral::A(),
      frc1678::elevator_controller::controller::first_stage_integral::B(),
      frc1678::elevator_controller::controller::first_stage_integral::C());

  elevator_controller_ = muan::control::StateSpaceController<1, 3, 1>(
      frc1678::elevator_controller::controller::first_stage_integral::K(),
      frc1678::elevator_controller::controller::first_stage_integral::Kff(),
      frc1678::elevator_controller::controller::first_stage_integral::A(),
      Eigen::Matrix<double, 1, 1>::Ones() *
          -std::numeric_limits<double>::infinity(),
      Eigen::Matrix<double, 1, 1>::Ones() *
          std::numeric_limits<double>::infinity());

  elevator_observer_ = muan::control::StateSpaceObserver<1, 3, 1>(
      elevator_plant,
      frc1678::elevator_controller::controller::first_stage_integral::L());

  plant_ = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::elevator_controller::controller::first_stage_integral::A(),
      frc1678::elevator_controller::controller::first_stage_integral::B(),
      frc1678::elevator_controller::controller::first_stage_integral::C());

  trapezoid_profile_.set_maximum_acceleration(kElevatorAcceleration);
  trapezoid_profile_.set_maximum_velocity(kElevatorVelocity);
}

void ElevatorController::Update(const ScoreSubsystemInputProto& input,
                                ScoreSubsystemOutputProto* output,
                                ScoreSubsystemStatusProto* status,
                                bool outputs_enabled) {
  Eigen::Matrix<double, 3, 1> elevator_r_;

  bool was_calibrated = hall_calib_.is_calibrated();

  auto elevator_y = (Eigen::Matrix<double, 1, 1>() << hall_calib_.Update(
                         input->elevator_encoder(), input->elevator_hall()))
                        .finished();

  if (hall_calib_.is_calibrated()) {
    SetWeights(elevator_observer_.x()(0, 0) >= 1.0, (*status)->has_cube());
  } else {
    SetWeights(false, false);
    LOG_P("Hall effect sensor not calibrated");
  }

  if (!outputs_enabled) {
    trapezoid_profile_.MoveCurrentState(
        elevator_observer_.x().block<2, 1>(0, 0));
  }

  if (hall_calib_.is_calibrated() && !was_calibrated) {
    elevator_observer_.x(0) += hall_calib_.offset();
    trapezoid_profile_.MoveCurrentState(
        elevator_observer_.x().block<2, 1>(0, 0));
  }

  UpdateProfiledGoal(unprofiled_goal_, outputs_enabled);
  elevator_r_ = (Eigen::Matrix<double, 3, 1>() << profiled_goal_(0),
                 profiled_goal_(1), 0.0)
                    .finished();

  auto elevator_u =
      elevator_controller_.Update(elevator_observer_.x(), elevator_r_)(0, 0);

  if (!outputs_enabled) {
    elevator_u = CapU(0);
    LOG_P("Elavator outputs not enabled!");
  } else if (!hall_calib_.is_calibrated()) {
    elevator_u = kCalibrationVoltage;
  } else if (encoder_fault_detected_) {
    elevator_u = 2.0;
    LOG_P("Encoder fault detected, setting voltage to 2.0");
  }

  if (old_pos_ == input->elevator_encoder() &&
      std::abs(elevator_u) >= kEncoderFaultMinVoltage) {
    num_encoder_fault_ticks_++;
    if (num_encoder_fault_ticks_ > kEncoderFaultTicksAllowed) {
      encoder_fault_detected_ = true;
      LOG_P("Encoder fault detected due to offset velocity");
    }
  } else {
    num_encoder_fault_ticks_ = 0;
  }

  (*status)->set_elevator_uncapped_voltage(elevator_u);

  elevator_u = CapU(elevator_u);

  old_pos_ = input->elevator_encoder();

  elevator_observer_.Update(
      (Eigen::Matrix<double, 1, 1>() << elevator_u).finished(), elevator_y);

  plant_.Update((Eigen::Matrix<double, 1, 1>() << elevator_u).finished());

  elevator_observer_.x(0) =
      muan::utils::Cap(elevator_observer_.x(0), 0, kElevatorMaxHeight);

  (*output)->set_elevator_voltage(elevator_u);
  (*status)->set_elevator_actual_height(elevator_observer_.x()(0, 0));
  (*status)->set_elevator_voltage_error(elevator_observer_.x()(2, 0));
  (*status)->set_estimated_velocity(elevator_observer_.x()(0, 0));
  (*status)->set_elevator_calibrated(hall_calib_.is_calibrated());
  (*status)->set_elevator_profiled_goal(profiled_goal_(0, 0));
  (*status)->set_elevator_unprofiled_goal(unprofiled_goal_);
  (*status)->set_elevator_at_top((*status)->elevator_actual_height() >=
                                 kElevatorMaxHeight - 0.01);
  (*status)->set_elevator_encoder_fault_detected(encoder_fault_detected_);
}

void ElevatorController::SetGoal(double goal) {
  unprofiled_goal_ = muan::utils::Cap(goal, 0, kElevatorMaxHeight);
}

Eigen::Matrix<double, 2, 1> ElevatorController::UpdateProfiledGoal(
    double unprofiled_goal_, bool outputs_enabled) {
  if (outputs_enabled) {
    profiled_goal_ = trapezoid_profile_.Update(unprofiled_goal_, 0);
  } else {
    profiled_goal_ = elevator_observer_.x().block<2, 1>(0, 0);
    LOG_P("Elavator controlled outputs not enabled");
  }
  return profiled_goal_;
}

double ElevatorController::CapU(double elevator_u) {
  return muan::utils::Cap(elevator_u, -kElevatorMaxVoltage,
                          kElevatorMaxVoltage);
}

void ElevatorController::SetWeights(bool second_stage, bool has_cube) {
  if (second_stage && has_cube) {
    elevator_controller_.A() = frc1678::elevator_controller::controller::
        second_stage_cube_integral::A();
    elevator_controller_.K() = frc1678::elevator_controller::controller::
        second_stage_cube_integral::K();
    elevator_controller_.Kff() = frc1678::elevator_controller::controller::
        second_stage_cube_integral::Kff();

    elevator_observer_.L() = frc1678::elevator_controller::controller::
        second_stage_cube_integral::L();

    plant_.A() = frc1678::elevator_controller::controller::
        second_stage_cube_integral::A();
    plant_.B() = frc1678::elevator_controller::controller::
        second_stage_cube_integral::B();
    plant_.C() = frc1678::elevator_controller::controller::
        second_stage_cube_integral::C();
  } else if (second_stage && !has_cube) {
    elevator_controller_.A() =
        frc1678::elevator_controller::controller::second_stage_integral::A();
    elevator_controller_.K() =
        frc1678::elevator_controller::controller::second_stage_integral::K();
    elevator_controller_.Kff() =
        frc1678::elevator_controller::controller::second_stage_integral::Kff();

    elevator_observer_.L() =
        frc1678::elevator_controller::controller::second_stage_integral::L();

    plant_.A() =
        frc1678::elevator_controller::controller::second_stage_integral::A();
    plant_.B() =
        frc1678::elevator_controller::controller::second_stage_integral::B();
    plant_.C() =
        frc1678::elevator_controller::controller::second_stage_integral::C();
  } else if (!second_stage && has_cube) {
    elevator_controller_.A() = frc1678::elevator_controller::controller::
        first_stage_cube_integral::A();
    elevator_controller_.K() = frc1678::elevator_controller::controller::
        first_stage_cube_integral::K();
    elevator_controller_.Kff() = frc1678::elevator_controller::controller::
        first_stage_cube_integral::Kff();

    elevator_observer_.L() = frc1678::elevator_controller::controller::
        first_stage_cube_integral::L();

    plant_.A() = frc1678::elevator_controller::controller::
        first_stage_cube_integral::A();
    plant_.B() = frc1678::elevator_controller::controller::
        first_stage_cube_integral::B();
    plant_.C() = frc1678::elevator_controller::controller::
        first_stage_cube_integral::C();
  } else if (!second_stage && !has_cube) {
    elevator_controller_.A() =
        frc1678::elevator_controller::controller::first_stage_integral::A();
    elevator_controller_.K() =
        frc1678::elevator_controller::controller::first_stage_integral::K();
    elevator_controller_.Kff() =
        frc1678::elevator_controller::controller::first_stage_integral::Kff();

    elevator_observer_.L() =
        frc1678::elevator_controller::controller::first_stage_integral::L();

    plant_.A() =
        frc1678::elevator_controller::controller::first_stage_integral::A();
    plant_.B() =
        frc1678::elevator_controller::controller::first_stage_integral::B();
    plant_.C() =
        frc1678::elevator_controller::controller::first_stage_integral::C();
  }
}

}  // namespace elevator

}  // namespace score_subsystem

}  // namespace c2018
