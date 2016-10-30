#include "drivetrain_controller.h"
#include "o2016/subsystems/drivetrain/drivetrain_constants.h"

namespace frc1678 {

namespace drivetrain {

namespace controller {

DrivetrainController::DrivetrainController()
    : distance_profile_{DrivetrainConstraints{}.distance_constraints_,
                        muan::control::MotionProfilePosition{0.0, 0.0}},
      angle_profile_{DrivetrainConstraints{}.angular_constraints_,
                     muan::control::MotionProfilePosition{0.0, 0.0}} {
  // TODO(Kyle) Find a way to generate this in the python code (or maybe
  // generate the constraints when the controller is told to follow a new
  // profile to get the maximum efficiency?)

  using namespace muan::units;

  controller_.u_min(0) = -12.0 * V;
  controller_.u_min(1) = -12.0 * V;
  controller_.u_max(0) = 12.0 * V;
  controller_.u_max(1) = 12.0 * V;

  Shift(Gear::kLowGear);
}

StackDrivetrainOutput DrivetrainController::Update(
    const StackDrivetrainInput& input) {
  using namespace muan::units;
  elapsed_time_ += frc1678::drivetrain::controller::high_gear_integral::dt();

  // Create the y vector from the input proto
  Eigen::Matrix<double, 3, 1> y = Eigen::Matrix<double, 3, 1>::Zero();
  {
    y(0) = input->right_encoder();
    y(1) = input->left_encoder();
    y(2) = input->gyro_angle();
  }

  auto r = controller_.r();
  if (drive_command_type_ == DriveType::kVelocityCommand) {
    r(0) = observer_.x(0);
    r(2) = observer_.x(2);
  } else if (drive_command_type_ == DriveType::kDistanceCommand) {
    r(0) = distance_profile_.Calculate(elapsed_time_).position;
    r(1) = distance_profile_.Calculate(elapsed_time_).velocity;
    r(2) = angle_profile_.Calculate(elapsed_time_).position;
    r(3) = angle_profile_.Calculate(elapsed_time_).velocity;
  }

  // Run state-space calculations
  auto u = controller_.Update(observer_.x(), r);
  observer_.Update(u, y);

  // Create the output proto from u
  StackDrivetrainOutput out;
  out->set_right_voltage(u(0));
  out->set_left_voltage(u(1));
  out->set_shifting(current_gear_ == Gear::kHighGear);

  return out;
}

void DrivetrainController::SetGoal(const StackDrivetrainGoal& goal) {
  Shift(goal->gear());

  // Create the r vector from the goal proto
  Eigen::Matrix<double, 7, 1> r = Eigen::Matrix<double, 7, 1>::Zero();
  if (goal->has_velocity_command()) {
    controller_.r(1) = goal->velocity_command().forward_velocity();
    controller_.r(3) = goal->velocity_command().angular_velocity();

    drive_command_type_ = DriveType::kVelocityCommand;
  } else if (goal->has_distance_command()) {
    auto final_state = goal->distance_command().final_state();

    muan::control::MotionProfilePosition initial_distance{0.0, observer_.x(1)};
    muan::control::MotionProfilePosition goal_distance{
        final_state.forward_distance(), final_state.forward_velocity()};

    muan::control::MotionProfilePosition initial_angle{0.0, observer_.x(3)};
    muan::control::MotionProfilePosition goal_angle{
        final_state.angular_distance(), final_state.angular_distance()};

    // If there was already a distance command running, use the current position
    // from that one to keep everything feed-forward
    if (drive_command_type_ == DriveType::kDistanceCommand) {
      initial_angle = angle_profile_.Calculate(elapsed_time_);
      initial_distance = angle_profile_.Calculate(elapsed_time_);
    }

    auto current_constraints =
        GenerateTMPConstraints(initial_distance, goal_distance, initial_angle,
                               goal_angle, goal->gear());

    distance_profile_ = muan::control::TrapezoidalMotionProfile(
        current_constraints.distance_constraints_, initial_distance,
        goal_distance, elapsed_time_);

    angle_profile_ = muan::control::TrapezoidalMotionProfile(
        current_constraints.angular_constraints_, initial_angle, goal_angle,
        elapsed_time_);

    drive_command_type_ = DriveType::kDistanceCommand;
  }
}

void DrivetrainController::Shift(Gear new_gear) {
  current_gear_ = new_gear;
  if (new_gear == Gear::kHighGear) {
    observer_.plant().A() =
        frc1678::drivetrain::controller::high_gear_integral::A();
    observer_.plant().B() =
        frc1678::drivetrain::controller::high_gear_integral::B();
    observer_.plant().C() =
        frc1678::drivetrain::controller::high_gear_integral::C();
    observer_.plant().D() =
        frc1678::drivetrain::controller::high_gear_integral::D();

    observer_.L() = frc1678::drivetrain::controller::high_gear_integral::L();

    controller_.A() = frc1678::drivetrain::controller::high_gear_integral::A();
    controller_.K() = frc1678::drivetrain::controller::high_gear_integral::K();
    controller_.Kff() =
        frc1678::drivetrain::controller::high_gear_integral::Kff();
  } else {
    observer_.plant().A() =
        frc1678::drivetrain::controller::low_gear_integral::A();
    observer_.plant().B() =
        frc1678::drivetrain::controller::low_gear_integral::B();
    observer_.plant().C() =
        frc1678::drivetrain::controller::low_gear_integral::C();
    observer_.plant().D() =
        frc1678::drivetrain::controller::low_gear_integral::D();

    observer_.L() = frc1678::drivetrain::controller::low_gear_integral::L();

    controller_.A() = frc1678::drivetrain::controller::low_gear_integral::A();
    controller_.K() = frc1678::drivetrain::controller::low_gear_integral::K();
    controller_.Kff() =
        frc1678::drivetrain::controller::low_gear_integral::Kff();
  }
}

StackDrivetrainStatus DrivetrainController::GetStatus() const {
  StackDrivetrainStatus status;

  status->set_current_driving_type(drive_command_type_);
  status->set_current_gear(current_gear_);
  status->set_filtered_distance(observer_.x(0));
  status->set_filtered_angle(observer_.x(2));

  return status;
}

DrivetrainConstraints DrivetrainController::GenerateTMPConstraints(
    muan::control::MotionProfilePosition initial_distance,
    muan::control::MotionProfilePosition final_distance,
    muan::control::MotionProfilePosition initial_angle,
    muan::control::MotionProfilePosition final_angle, Gear current_gear) {
  if (current_gear == Gear::kHighGear) {
    // Forward: x'' = a_f*x' + b_f*u
    double a_f =
        frc1678::drivetrain::controller::high_gear_integral::A_c()(1, 1);
    double b_f = frc1678::drivetrain::controller::high_gear_integral::B_c()(1);

    // Angular: theta'' = a_a*theta' + b_a*u
    double a_a =
        frc1678::drivetrain::controller::high_gear_integral::A_c()(3, 3);
    double b_a = frc1678::drivetrain::controller::high_gear_integral::B_c()(3);

    // u_max = u_aa + u_av + u_fa + u_fv
  } else {
    // Forward: x'' = a_f*x' + b_f*u
    double a_f =
        frc1678::drivetrain::controller::low_gear_integral::A_c()(1, 1);
    double b_f = frc1678::drivetrain::controller::low_gear_integral::B_c()(1);

    // Angular: theta'' = a_a*theta' + b_a*u
    double a_a =
        frc1678::drivetrain::controller::low_gear_integral::A_c()(3, 3);
    double b_a = frc1678::drivetrain::controller::low_gear_integral::B_c()(3);
  }
  return DrivetrainConstraints();
}

} /* controller */

} /* drivetrain */

} /* o2016 */
