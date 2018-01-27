#include "o2017/citrus_robot/main.h"
#include "WPILib.h"
#include "muan/wpilib/queue_types.h"
#include "o2017/queue_manager/queue_manager.h"

namespace o2017 {

namespace citrus_robot {

CitrusRobot::CitrusRobot()
    : throttle_(1),
      wheel_(0),
      gamepad_(2),
      ds_sender_(&QueueManager::GetInstance()->driver_station_queue()) {
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);
  quickturn_ = wheel_.MakeButton(5);
  hp_gear_ = throttle_.MakeButton(2);
  climber_ = gamepad_.MakeButton((uint32_t)muan::teleop::XBox::BACK);
}

void CitrusRobot::RunAutonomous() { auto_state_ = AutoState::kInit; }

void CitrusRobot::SetDriveGoal(double distance) {
  auto maybe_status =
      QueueManager::GetInstance()->drivetrain_status_queue()->ReadLastMessage();
  double right_offset = 0.0, left_offset = 0.0;
  if (maybe_status) {
    auto status = *maybe_status;
    left_offset = status->estimated_left_position();
    right_offset = status->estimated_right_position();
  }
  drivetrain_goal_->mutable_distance_command()->set_left_goal(left_offset +
                                                              distance);
  drivetrain_goal_->mutable_distance_command()->set_right_goal(right_offset +
                                                               distance);

  drivetrain_goal_->mutable_linear_constraints()->set_max_velocity(1.0);
  drivetrain_goal_->mutable_linear_constraints()->set_max_acceleration(1.0);
  drivetrain_goal_->mutable_angular_constraints()->set_max_velocity(1.0);
  drivetrain_goal_->mutable_angular_constraints()->set_max_acceleration(1.0);

  drivetrain_goal_->set_gear(frc971::control_loops::drivetrain::Gear::kLowGear);
}

bool CitrusRobot::DrivingDone() const {
  auto maybe_status =
      QueueManager::GetInstance()->drivetrain_status_queue()->ReadLastMessage();
  constexpr double kDistanceTolerance = 0.1;
  if (maybe_status) {
    auto status = *maybe_status;
    return std::abs(drivetrain_goal_->distance_command().left_goal() -
                    status->estimated_left_position()) < kDistanceTolerance &&
           std::abs(drivetrain_goal_->distance_command().right_goal() -
                    status->estimated_right_position()) < kDistanceTolerance;
  }
  return true;
}

void CitrusRobot::UpdateAutonomous() {
  o2017::superstructure::SuperstructureGoalProto super_goal;
  switch (auto_state_) {
    case AutoState::kInit:
      auto_state_ = AutoState::kDriveForwards;
      SetDriveGoal(2.3);
      super_goal->set_hp_gear(true);
      break;
    case AutoState::kDriveForwards:
      if (DrivingDone()) {
        auto_state_ = AutoState::kGearEject;
        start_ = aos::monotonic_clock::now();
      }
      break;
    case AutoState::kGearEject:
      super_goal->set_hp_gear(true);
      if (aos::monotonic_clock::now() >=
          start_ + std::chrono::milliseconds(1000)) {
        auto_state_ = AutoState::kDriveBackwards;
        SetDriveGoal(-1.0);
      }
      break;
    case AutoState::kDriveBackwards:
      super_goal->set_hp_gear(true);
      if (DrivingDone()) {
        auto_state_ = AutoState::kDone;
      }
      break;
    case AutoState::kDone:
      super_goal->set_hp_gear(false);
      break;
  }

  QueueManager::GetInstance()->superstructure_goal_queue()->WriteMessage(
      super_goal);
  QueueManager::GetInstance()->drivetrain_goal_queue()->WriteMessage(
      drivetrain_goal_);
}

void CitrusRobot::Update() {
  if (DriverStation::GetInstance().IsEnabled() &&
      DriverStation::GetInstance().IsAutonomous()) {
    UpdateAutonomous();
  } else if (DriverStation::GetInstance().IsOperatorControl()) {
    // Update joysticks
    throttle_.Update();
    wheel_.Update();
    gamepad_.Update();
    SendDrivetrainMessage();

    o2017::superstructure::SuperstructureGoalProto super_goal;
    super_goal->set_hp_gear(hp_gear_->is_pressed());
    climbing_ = climbing_ ^ climber_->was_clicked();
    super_goal->set_should_climb(climbing_);
    QueueManager::GetInstance()->superstructure_goal_queue()->WriteMessage(
        super_goal);
  }

  ds_sender_.Send();
}

void CitrusRobot::SendDrivetrainMessage() {
  frc971::control_loops::drivetrain::GoalProto drivetrain_goal;

  double throttle = -throttle_.wpilib_joystick()->GetRawAxis(1);
  double wheel = -wheel_.wpilib_joystick()->GetRawAxis(0);
  bool quickturn = quickturn_->is_pressed();

  if (shifting_high_->is_pressed()) {
    high_gear_ = true;
  } else if (shifting_low_->is_pressed()) {
    high_gear_ = false;
  }

  drivetrain_goal->mutable_teleop_command()->set_steering(wheel);
  drivetrain_goal->mutable_teleop_command()->set_throttle(throttle);
  drivetrain_goal->mutable_teleop_command()->set_quick_turn(quickturn);

  drivetrain_goal->set_gear(
      high_gear_ ? frc971::control_loops::drivetrain::Gear::kHighGear
                 : frc971::control_loops::drivetrain::Gear::kLowGear);

  o2017::QueueManager::GetInstance()->drivetrain_goal_queue()->WriteMessage(
      drivetrain_goal);
}

}  // namespace citrus_robot

}  // namespace o2017
