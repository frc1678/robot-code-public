#include "c2018/subsystems/score_subsystem/wrist/wrist.h"

namespace c2018 {
namespace score_subsystem {
namespace wrist {

using muan::queues::QueueManager;

WristController::WristController() {
  auto wrist_plant = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::wrist::controller::cube_integral::A(),
      frc1678::wrist::controller::cube_integral::B(),
      frc1678::wrist::controller::cube_integral::C());

  wrist_controller_ = muan::control::StateSpaceController<1, 3, 1>(
      frc1678::wrist::controller::cube_integral::K(),
      frc1678::wrist::controller::cube_integral::Kff(),
      frc1678::wrist::controller::cube_integral::A(),
      Eigen::Matrix<double, 1, 1>::Ones() * -12,
      Eigen::Matrix<double, 1, 1>::Ones() * 12);

  wrist_observer_ = muan::control::StateSpaceObserver<1, 3, 1>(
      wrist_plant, frc1678::wrist::controller::cube_integral::L());
}

void WristController::SetGoal(double wrist_angle, IntakeGoal intake_mode) {
  if (std::abs(wrist_angle - unprofiled_goal_.position) > 1e-10) {
    profile_time_ = 0.;
    profile_initial_ = {wrist_observer_.x()(0, 0), wrist_observer_.x()(1, 0)};
  }
  // Cap unprofiled goal to keep things safe
  unprofiled_goal_ = {
      muan::utils::Cap(wrist_angle, kWristMinAngle, kWristMaxAngle), 0.};
  // Set the goal intake mode
  intake_mode_ = intake_mode;
}

void WristController::Update(ScoreSubsystemInputProto input,
                             ScoreSubsystemOutputProto* output,
                             ScoreSubsystemStatusProto* status,
                             bool outputs_enabled) {
  // Was it calibrated before updating the hall calibration?
  was_calibrated_ = hall_calibration_.is_calibrated();

  // Now we update the hall calibration
  double calibrated_encoder =
      hall_calibration_.Update(input->wrist_encoder(), input->wrist_hall());

  if (input->has_cube()) {
    SetWeights(true);
  } else {
    SetWeights(false);
  }
  auto wrist_y =
      (Eigen::Matrix<double, 1, 1>() << calibrated_encoder).finished();

  // Setting up that wrist voltage to start it 0.0
  double wrist_voltage = 0.0;

  // Logic to make sure it actually has a cube
  if (input->has_cube() && has_cube_for_ticks_ < kNumHasCubeTicks * 2) {
    has_cube_for_ticks_++;
  } else if (!input->has_cube() && has_cube_for_ticks_ > 0) {
    has_cube_for_ticks_--;
  }
  bool has_cube = has_cube_for_ticks_ > kNumHasCubeTicks;

  if (!outputs_enabled) {
    wrist_voltage = 0.0;
    profile_initial_ = {wrist_observer_.x()(0, 0), wrist_observer_.x()(1, 0)};
  }

  // Start of intake
  bool wrist_solenoid_close = false;
  bool wrist_solenoid_open = false;

  if (outputs_enabled) {
    switch (intake_mode_) {
      case IntakeGoal::INTAKE:
        intake_voltage_ = kIntakeVoltage;
        wrist_solenoid_close = false;
        wrist_solenoid_open = false;
        break;
      case IntakeGoal::INTAKE_OPEN:
        intake_voltage_ = kIntakeVoltage;
        wrist_solenoid_close = false;
        wrist_solenoid_open = true;
        break;
      case IntakeGoal::INTAKE_CLOSE:
      case IntakeGoal::SETTLE:
        intake_voltage_ = kIntakeVoltage;
        wrist_solenoid_close = true;
        wrist_solenoid_open = false;
        break;
      case IntakeGoal::OUTTAKE_SLOW:
        intake_voltage_ = kSlowOuttakeVoltage;
        wrist_solenoid_close = true;
        wrist_solenoid_open = false;
        break;
      case IntakeGoal::OUTTAKE_FAST:
        intake_voltage_ = kFastOuttakeVoltage;
        wrist_solenoid_close = true;
        wrist_solenoid_open = false;
        break;
      case IntakeGoal::DROP:
        intake_voltage_ = 0;
        wrist_solenoid_close = false;
        wrist_solenoid_open = true;
        break;
      case IntakeGoal::INTAKE_NONE:
        if (has_cube) {
          intake_voltage_ = kHoldingVoltage;
        } else {
          intake_voltage_ = 0;
        }
        wrist_solenoid_close = true;
        wrist_solenoid_open = false;
        break;
    }
  } else {
    intake_voltage_ = 0;
  }

  if (!hall_calibration_.is_calibrated()) {
    wrist_voltage = kCalibVoltage;
  } else if (!was_calibrated_) {
    plant_.x()(0, 0) += hall_calibration_.offset();
  }

  UpdateProfiledGoal(outputs_enabled);
  Eigen::Matrix<double, 3, 1> wrist_r =
      (Eigen::Matrix<double, 3, 1>() << profiled_goal_.position, 0.0, 0.0)
          .finished();

  wrist_controller_.r() = wrist_r;

  // Get voltage from the controller
  wrist_voltage = wrist_controller_.Update(wrist_observer_.x(), wrist_r)(0, 0);

  if (hall_calibration_.is_calibrated() && wrist_r(0) <= 1e-5) {
    // If we're trying to stay at 0, set 0 voltage automatically
    wrist_voltage = 0.0;
  }

  // Cap the voltage so it's realistic
  if (outputs_enabled) {
    wrist_voltage = muan::utils::Cap(wrist_voltage, -kMaxVoltage, kMaxVoltage);
  } else {
    wrist_voltage = 0;
  }

  // Update the observer and the plant with the actual voltage
  wrist_observer_.Update(
      (Eigen::Matrix<double, 1, 1>() << wrist_voltage).finished(), wrist_y);
  plant_.Update((Eigen::Matrix<double, 1, 1>() << wrist_voltage).finished());

  // Write stuff to the output and status protos
  (*output)->set_intake_voltage(intake_voltage_);
  (*output)->set_wrist_voltage(wrist_voltage);
  (*output)->set_wrist_solenoid_open(wrist_solenoid_open);
  (*output)->set_wrist_solenoid_close(wrist_solenoid_close);
  (*status)->set_wrist_calibrated(hall_calibration_.is_calibrated());
  (*status)->set_wrist_angle(wrist_observer_.x()(0, 0));
  (*status)->set_has_cube(has_cube);
  (*status)->set_wrist_profiled_goal(profiled_goal_.position);
  (*status)->set_wrist_unprofiled_goal(unprofiled_goal_.position);
  (*status)->set_wrist_calibration_offset(hall_calibration_.offset());
}

muan::control::MotionProfilePosition WristController::UpdateProfiledGoal(
    bool outputs_enabled) {
  // Sets profiled goal based on the motion profile
  muan::control::TrapezoidalMotionProfile profile =
      muan::control::TrapezoidalMotionProfile(
          kWristConstraints, unprofiled_goal_, profile_initial_);
  if (outputs_enabled) {
    // Figure out what the trapezoid profile wants
    profiled_goal_ = profile.Calculate(profile_time_ += 5 * muan::units::ms);
  } else {
    // Keep the trapezoid profile updated on the (sadly) disabled robot
    profiled_goal_ = profile.Calculate(0.);
  }
  return profiled_goal_;
}

double WristController::TimeLeftUntil(double angle, double final_angle) {
  if (profiled_goal_.position > angle) {
    return 0.;
  }

  muan::control::TrapezoidalMotionProfile profile =
      muan::control::TrapezoidalMotionProfile(kWristConstraints,
                                              {final_angle, 0}, profiled_goal_);
  return profile.TimeLeftUntil(angle);
}

bool WristController::is_calibrated() const {
  // Returns if calibrated so it can get used by the score subsystem
  return hall_calibration_.is_calibrated();
}

void WristController::SetWeights(bool has_cube) {
  if (has_cube) {
    wrist_controller_.A() = frc1678::wrist::controller::cube_integral::A();
    wrist_controller_.K() = frc1678::wrist::controller::cube_integral::K();
    wrist_controller_.Kff() = frc1678::wrist::controller::cube_integral::Kff();

    wrist_observer_.L() = frc1678::wrist::controller::cube_integral::L();

    plant_.A() = frc1678::wrist::controller::cube_integral::A();
    plant_.B() = frc1678::wrist::controller::cube_integral::B();
    plant_.C() = frc1678::wrist::controller::cube_integral::C();
  } else {
    wrist_controller_.A() = frc1678::wrist::controller::no_cube_integral::A();
    wrist_controller_.K() = frc1678::wrist::controller::no_cube_integral::K();
    wrist_controller_.Kff() =
        frc1678::wrist::controller::no_cube_integral::Kff();

    wrist_observer_.L() = frc1678::wrist::controller::no_cube_integral::L();

    plant_.A() = frc1678::wrist::controller::no_cube_integral::A();
    plant_.B() = frc1678::wrist::controller::no_cube_integral::B();
    plant_.C() = frc1678::wrist::controller::no_cube_integral::C();
  }
}

}  // namespace wrist
}  // namespace score_subsystem
}  // namespace c2018
