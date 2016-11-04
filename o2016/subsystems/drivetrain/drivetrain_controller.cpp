#include "drivetrain_controller.h"
#include "o2016/subsystems/drivetrain/drivetrain_constants.h"

namespace o2016 {

namespace drivetrain {

namespace controller {

DrivetrainController::DrivetrainController()
    : distance_profile_{DrivetrainConstraints{}.distance_constraints_,
                        muan::control::MotionProfilePosition{0.0, 0.0}},
      angle_profile_{DrivetrainConstraints{}.angular_constraints_,
                     muan::control::MotionProfilePosition{0.0, 0.0}} {
  using namespace muan::units;

  controller_.u_min(0) = -12.0 * V;
  controller_.u_min(1) = -12.0 * V;
  controller_.u_max(0) = 12.0 * V;
  controller_.u_max(1) = 12.0 * V;

  elapsed_time_ = 0 * s;

  Shift(Gear::kLowGear);
}

// In an anonymous namespace because nothing outisde of this unit should ever
// need to refer to it
namespace {
const Eigen::Matrix<double, 4, 1> termination =
    (Eigen::Matrix<double, 4, 1>() << 0.01, 0.01, 0.01, 0.01).finished();
}

StackDrivetrainOutput DrivetrainController::Update(
    const StackDrivetrainInput& input) {
  using namespace muan::units;
  elapsed_time_ += frc1678::drivetrain::controller::high_gear_integral::dt();

  // Reset just_finished_profile_ because if it's true, it happened last tick
  // and we don't care anymore
  just_finished_profile_ = false;

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
    auto d = distance_profile_.Calculate(elapsed_time_);
    r(0) = d.position;
    r(1) = d.velocity;
    r(2) = angle_profile_.Calculate(elapsed_time_).position;
    r(3) = angle_profile_.Calculate(elapsed_time_).velocity;

    auto e = r - observer_.x();
    if (distance_profile_.finished(elapsed_time_) &&
        angle_profile_.finished(elapsed_time_) &&
        std::fabs(e(0)) < termination(0) && std::fabs(e(1)) < termination(1) &&
        std::fabs(e(2)) < termination(2) && std::fabs(e(3)) < termination(3)) {
      drive_command_type_ = DriveType::kVelocityCommand;
      just_finished_profile_ = true;
    }
  }

  // Run state-space calculations
  auto u = controller_.Update(observer_.x(), r);
  observer_.Update(u, y);

  // Create the output proto from u
  StackDrivetrainOutput out;
  out->set_right_voltage(u(0));
  out->set_left_voltage(u(1));
  out->set_high_gear(current_gear_ == Gear::kHighGear);

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
        final_state.heading(), final_state.angular_velocity()};

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
        current_constraints.distance_constraints_, goal_distance,
        initial_distance, elapsed_time_);

    angle_profile_ = muan::control::TrapezoidalMotionProfile(
        current_constraints.angular_constraints_, goal_angle, initial_angle,
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

  status->mutable_observed_state()->set_forward_distance(observer_.x(0));
  status->mutable_observed_state()->set_forward_velocity(observer_.x(1));
  status->mutable_observed_state()->set_heading(observer_.x(2));
  status->mutable_observed_state()->set_angular_velocity(observer_.x(3));

  status->mutable_filtered_goal()->set_forward_distance(controller_.r(0));
  status->mutable_filtered_goal()->set_forward_velocity(controller_.r(1));
  status->mutable_filtered_goal()->set_heading(controller_.r(2));
  status->mutable_filtered_goal()->set_angular_velocity(controller_.r(3));

  status->set_just_finished_profile(just_finished_profile_);

  return status;
}

DrivetrainConstraints DrivetrainController::GenerateTMPConstraints(
    muan::control::MotionProfilePosition initial_distance,
    muan::control::MotionProfilePosition final_distance,
    muan::control::MotionProfilePosition initial_angle,
    muan::control::MotionProfilePosition final_angle, Gear current_gear) {
  // TODO(Kyle) Find a way to generate this in the python code (or maybe
  // generate the constraints when the controller is told to follow a new
  // profile to get the maximum efficiency?)
  DrivetrainConstraints constraints;
  if (current_gear == Gear::kHighGear) {
    constraints.distance_constraints_.max_velocity = 1.0;
    constraints.distance_constraints_.max_acceleration = 1.0;
    constraints.angular_constraints_.max_velocity = 10.0;
    constraints.angular_constraints_.max_acceleration = 10.0;
  } else {
    constraints.distance_constraints_.max_velocity = 1.5;
    constraints.distance_constraints_.max_acceleration = 1.3;
    constraints.angular_constraints_.max_velocity = 6.0;
    constraints.angular_constraints_.max_acceleration = 1.5;
  }
  return constraints;
}

}  // controller

}  // drivetrain

}  // o2016
