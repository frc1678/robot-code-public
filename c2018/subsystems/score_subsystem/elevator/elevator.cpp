#include "c2018/subsystems/score_subsystem/elevator/elevator.h"

namespace c2018 {
namespace score_subsystem {
namespace elevator {

ElevatorController::ElevatorController() {
  plant_ = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::elevator::controller::first_stage_integral::A(),
      frc1678::elevator::controller::first_stage_integral::B(),
      frc1678::elevator::controller::first_stage_integral::C());

  controller_ = muan::control::StateSpaceController<1, 3, 1>(
      frc1678::elevator::controller::first_stage_integral::K(),
      frc1678::elevator::controller::first_stage_integral::Kff(),
      frc1678::elevator::controller::first_stage_integral::A(),
      Eigen::Matrix<double, 1, 1>::Ones() *
          -std::numeric_limits<double>::infinity(),
      Eigen::Matrix<double, 1, 1>::Ones() *
          std::numeric_limits<double>::infinity());

  observer_ = muan::control::StateSpaceObserver<1, 3, 1>(
      plant_, frc1678::elevator::controller::first_stage_integral::L());
}

void ElevatorController::Update(const ScoreSubsystemInputProto& input,
                                ScoreSubsystemOutputProto* output,
                                ScoreSubsystemStatusProto* status,
                                bool outputs_enabled) {
  bool was_calibrated = hall_calibration_.is_calibrated();

  Eigen::Matrix<double, 1, 1> y =
      (Eigen::Matrix<double, 1, 1>() << hall_calibration_.Update(
           input->elevator_encoder(), input->elevator_hall()))
          .finished();

  // Gains Scheduling based on position and cube
  if (hall_calibration_.is_calibrated()) {
    SetWeights(observer_.x(0) >= 1.0, (*status)->has_cube());
  } else {
    SetWeights(false, false);
  }

  // Keeps trapezoidal motion profile notified about all the things while
  // disabled
  if (!outputs_enabled) {
    profiled_goal_ = {observer_.x(0), observer_.x(1)};
  }

  // The first calibrate is the only one that does anything to the model
  if (hall_calibration_.is_calibrated() && !was_calibrated) {
    observer_.x(0) += hall_calibration_.offset();
    profiled_goal_ = {observer_.x(0), observer_.x(1)};
  }

  // Update the trapezoidal motion profile
  UpdateProfiledGoal(outputs_enabled);

  // Make an R matrix from the new profiled goal
  controller_.r() = (Eigen::Matrix<double, 3, 1>() << profiled_goal_.position,
                     profiled_goal_.velocity, 0.0)
                        .finished();

  // Get output in volts from the controller
  auto elevator_u = controller_.Update(observer_.x(), controller_.r())(0, 0);

  if (!outputs_enabled) {
    // Make elevator volts reflect reality
    elevator_u = CapU(0);
  } else if (!hall_calibration_.is_calibrated()) {
    // Calibrate if not calibrated with manual volts
    elevator_u = kCalibrationVoltage;
  } else if (encoder_fault_detected_) {
    elevator_u = 2.0;
    LOG(WARNING, "Encoder fault detected, setting voltage to 2.0");
  } else if (profiled_goal_.position <= 1e-5) {
    // If we're trying to stay at 0, set 0 voltage automatically
    elevator_u = 0.0;
  }

  // Encoder falt checking
  if (old_pos_ == input->elevator_encoder() &&
      std::abs(elevator_u) >= kEncoderFaultMinVoltage) {
    num_encoder_fault_ticks_++;
    if (num_encoder_fault_ticks_ > kEncoderFaultTicksAllowed) {
      encoder_fault_detected_ = true;
      LOG(WARNING, "Encoder fault detected due to offset velocity");
    }
  } else if (old_pos_ != input->elevator_encoder()) {
    // Reset the encoder fault checking so it doesn't build up
    num_encoder_fault_ticks_ = 0;
    encoder_fault_detected_ = false;
  }

  // Keep the voltage that the controller wanted for logging
  (*status)->set_elevator_uncapped_voltage(elevator_u);

  // Cap the voltage so it reflects what's in reality so the observer and plant
  // don't get scared
  elevator_u = CapU(elevator_u);

  // For the next iteration when we use this in encoder fault checking
  old_pos_ = input->elevator_encoder();

  // Updates plant and observer with capped voltage
  observer_.Update((Eigen::Matrix<double, 1, 1>() << elevator_u).finished(), y);
  plant_.Update((Eigen::Matrix<double, 1, 1>() << elevator_u).finished());

  // Writes to output and status protos
  (*output)->set_elevator_voltage(elevator_u);
  (*status)->set_elevator_height(observer_.x(0));
  (*status)->set_elevator_voltage_error(observer_.x(2));
  (*status)->set_elevator_velocity(observer_.x(1));
  (*status)->set_elevator_calibrated(hall_calibration_.is_calibrated());
  (*status)->set_elevator_profiled_goal(profiled_goal_.position);
  (*status)->set_elevator_unprofiled_goal(unprofiled_goal_.position);
  (*status)->set_elevator_at_top((*status)->elevator_height() >=
                                 kElevatorMaxHeight - 0.01);
  (*status)->set_elevator_encoder_fault_detected(encoder_fault_detected_);
  (*status)->set_elevator_calibration_offset(hall_calibration_.offset());
}

void ElevatorController::SetGoal(double goal) {
  // Cap goal to actual possible height so things don't break
  unprofiled_goal_ = {
      .position =
          muan::utils::Cap(goal, 0., kElevatorMaxHeight) * muan::units::m,
      .velocity = 0.};
}

muan::control::MotionProfilePosition ElevatorController::UpdateProfiledGoal(
    bool outputs_enabled) {
  muan::control::TrapezoidalMotionProfile profile =
      muan::control::TrapezoidalMotionProfile(kElevatorConstraints,
                                              unprofiled_goal_, profiled_goal_);
  if (outputs_enabled) {
    // Figure out what the trapezoid profile wants
    profiled_goal_ = profile.Calculate(5 * muan::units::ms);
  } else {
    // Keep the trapezoid profile updated on the (sadly) disabled robot
    profiled_goal_ = profile.Calculate(0.);
  }
  return profiled_goal_;
}

muan::units::Time ElevatorController::TimeLeftUntil(double target,
                                                    double final_goal) {
  if (profiled_goal_.position > target) {
    return 0.;
  }
  muan::control::TrapezoidalMotionProfile profile =
      muan::control::TrapezoidalMotionProfile(kElevatorConstraints,
                                              {final_goal, 0}, profiled_goal_);
  return profile.TimeLeftUntil(target);
}

double ElevatorController::CapU(double elevator_u) {
  // Cap voltage to what's possible (SKY's the limit)
  // SKY = +/- 12
  return muan::utils::Cap(elevator_u, -kElevatorMaxVoltage,
                          kElevatorMaxVoltage);
}

void ElevatorController::SetWeights(bool second_stage, bool has_cube) {
  // Change the gains based on where it is using stuff generated from the python
  // controller for all the scenarios
  if (second_stage && has_cube) {
    controller_.A() =
        frc1678::elevator::controller::second_stage_cube_integral::A();
    controller_.K() =
        frc1678::elevator::controller::second_stage_cube_integral::K();
    controller_.Kff() =
        frc1678::elevator::controller::second_stage_cube_integral::Kff();

    observer_.L() =
        frc1678::elevator::controller::second_stage_cube_integral::L();

    plant_.A() = frc1678::elevator::controller::second_stage_cube_integral::A();
    plant_.B() = frc1678::elevator::controller::second_stage_cube_integral::B();
    plant_.C() = frc1678::elevator::controller::second_stage_cube_integral::C();
  } else if (second_stage && !has_cube) {
    controller_.A() = frc1678::elevator::controller::second_stage_integral::A();
    controller_.K() = frc1678::elevator::controller::second_stage_integral::K();
    controller_.Kff() =
        frc1678::elevator::controller::second_stage_integral::Kff();

    observer_.L() = frc1678::elevator::controller::second_stage_integral::L();

    plant_.A() = frc1678::elevator::controller::second_stage_integral::A();
    plant_.B() = frc1678::elevator::controller::second_stage_integral::B();
    plant_.C() = frc1678::elevator::controller::second_stage_integral::C();
  } else if (!second_stage && has_cube) {
    controller_.A() =
        frc1678::elevator::controller::first_stage_cube_integral::A();
    controller_.K() =
        frc1678::elevator::controller::first_stage_cube_integral::K();
    controller_.Kff() =
        frc1678::elevator::controller::first_stage_cube_integral::Kff();

    observer_.L() =
        frc1678::elevator::controller::first_stage_cube_integral::L();

    plant_.A() = frc1678::elevator::controller::first_stage_cube_integral::A();
    plant_.B() = frc1678::elevator::controller::first_stage_cube_integral::B();
    plant_.C() = frc1678::elevator::controller::first_stage_cube_integral::C();
  } else if (!second_stage && !has_cube) {
    controller_.A() = frc1678::elevator::controller::first_stage_integral::A();
    controller_.K() = frc1678::elevator::controller::first_stage_integral::K();
    controller_.Kff() =
        frc1678::elevator::controller::first_stage_integral::Kff();

    observer_.L() = frc1678::elevator::controller::first_stage_integral::L();

    plant_.A() = frc1678::elevator::controller::first_stage_integral::A();
    plant_.B() = frc1678::elevator::controller::first_stage_integral::B();
    plant_.C() = frc1678::elevator::controller::first_stage_integral::C();
  }
}

bool ElevatorController::is_calibrated() const {
  // Tell us if its calibrated so the score subsystem can use it!
  return hall_calibration_.is_calibrated();
}

}  // namespace elevator
}  // namespace score_subsystem
}  // namespace c2018
