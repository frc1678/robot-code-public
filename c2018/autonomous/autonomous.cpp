#include "c2018/autonomous/autonomous.h"

#include <chrono>

#include "c2018/subsystems/drivetrain/drivetrain_base.h"
#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"

namespace c2018 {

namespace autonomous {

using muan::queues::QueueManager;
using DrivetrainGoal = frc971::control_loops::drivetrain::GoalProto;
using DrivetrainStatus = frc971::control_loops::drivetrain::StatusProto;
using muan::wpilib::DriverStationProto;
using muan::wpilib::GameSpecificStringProto;

AutonomousBase::AutonomousBase()
    : config_(drivetrain::GetDrivetrainConfig()),
      drivetrain_goal_queue_(QueueManager<DrivetrainGoal>::Fetch()),
      drivetrain_status_reader_(QueueManager<DrivetrainStatus>::Fetch()->MakeReader()),
      driver_station_reader_(QueueManager<DriverStationProto>::Fetch()->MakeReader()),
      game_specific_string_reader_(QueueManager<GameSpecificStringProto>::Fetch()->MakeReader()) {}

bool AutonomousBase::IsAutonomous() {
  DriverStationProto driver_station;
  if (driver_station_reader_.ReadLastMessage(&driver_station)) {
    return driver_station->mode() == RobotMode::AUTONOMOUS;
  } else {
    LOG_P("No driver station status found.");
    return false;
  }
}

void AutonomousBase::StartDriveAbsolute(double left, double right) {
  DrivetrainGoal goal;

  goal->mutable_distance_command()->set_left_goal(left);
  goal->mutable_distance_command()->set_right_goal(right);

  goal->mutable_linear_constraints()->set_max_velocity(max_forward_velocity_);
  goal->mutable_linear_constraints()->set_max_acceleration(max_forward_acceleration_);
  goal->mutable_angular_constraints()->set_max_velocity(max_angular_velocity_);
  goal->mutable_angular_constraints()->set_max_acceleration(max_angular_acceleration_);

  drivetrain_goal_queue_->WriteMessage(goal);
}

void AutonomousBase::StartDriveRelative(double forward, double theta) {
  DrivetrainStatus status;
  if (!drivetrain_status_reader_.ReadLastMessage(&status)) {
    LOG_P("No drivetrain status found.");
    return;
  }

  double left_offset = status->estimated_left_position();
  double right_offset = status->estimated_right_position();

  double left_goal = left_offset + forward - theta * config_.robot_radius;
  double right_goal = right_offset + forward + theta * config_.robot_radius;

  StartDriveAbsolute(left_goal, right_goal);
}

void AutonomousBase::StartDrivePath(double x, double y, double heading) {
  DrivetrainGoal goal;

  goal->mutable_path_command()->set_x_goal(x);
  goal->mutable_path_command()->set_y_goal(y);
  goal->mutable_path_command()->set_theta_goal(heading);

  goal->mutable_linear_constraints()->set_max_velocity(max_forward_velocity_);
  goal->mutable_linear_constraints()->set_max_acceleration(max_forward_acceleration_);
  goal->mutable_angular_constraints()->set_max_velocity(max_angular_velocity_);
  goal->mutable_angular_constraints()->set_max_acceleration(max_angular_acceleration_);

  drivetrain_goal_queue_->WriteMessage(goal);
}

void AutonomousBase::operator()() {
  aos::SetCurrentThreadRealtimePriority(10);
  aos::SetCurrentThreadName("Autonomous");

  DriverStationProto driver_station;
  GameSpecificStringProto game_specific_string;

  while (driver_station_reader_.ReadLastMessage(&driver_station),
         driver_station->mode() != RobotMode::AUTONOMOUS) {
    loop_.SleepUntilNext();
  }

  while (!game_specific_string_reader_.ReadLastMessage(&game_specific_string)) {
    loop_.SleepUntilNext();
  }

  // Start of autonomous. Grab the game specific string.
  auto left_right_codes = game_specific_string->code();
  LOG_P("Starting autonomous with layout ", left_right_codes);
  if (left_right_codes[0] == 'L') {
    if (left_right_codes[1] == 'L') {
      // Switch is left, scale is left
    } else if (left_right_codes[1] == 'R') {
      // Switch is left, scale is right
    }
  } else if (left_right_codes[0] == 'R') {
    if (left_right_codes[1] == 'L') {
      // Switch is right, scale is left
    } else if (left_right_codes[1] == 'R') {
      // Switch is right, scale is right
    }
  }
}

}  // namespace autonomous

}  // namespace c2018
