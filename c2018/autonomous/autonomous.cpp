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
      drivetrain_status_reader_(
          QueueManager<DrivetrainStatus>::Fetch()->MakeReader()),
      score_goal_queue_(
          QueueManager<score_subsystem::ScoreSubsystemGoalProto>::Fetch()),
      score_status_reader_(
          QueueManager<score_subsystem::ScoreSubsystemStatusProto>::Fetch()
              ->MakeReader()),
      driver_station_reader_(
          QueueManager<DriverStationProto>::Fetch()->MakeReader()),
      game_specific_string_reader_(
          QueueManager<GameSpecificStringProto>::Fetch()->MakeReader()) {}

bool AutonomousBase::IsAutonomous() {
  DriverStationProto driver_station;
  if (driver_station_reader_.ReadLastMessage(&driver_station)) {
    return driver_station->mode() == RobotMode::AUTONOMOUS;
  } else {
    LOG_P("No driver station status found.");
    return false;
  }
}

void AutonomousBase::StartDriveAbsolute(double left, double right,
                                        bool follow_through) {
  DrivetrainGoal goal;
  follow_through_ = follow_through;

  goal->mutable_distance_command()->set_left_goal(left);
  goal->mutable_distance_command()->set_right_goal(right);

  goal->mutable_linear_constraints()->set_max_velocity(max_forward_velocity_);
  goal->mutable_linear_constraints()->set_max_acceleration(
      max_forward_acceleration_);
  goal->mutable_angular_constraints()->set_max_velocity(max_angular_velocity_);
  goal->mutable_angular_constraints()->set_max_acceleration(
      max_angular_acceleration_);

  drivetrain_goal_queue_->WriteMessage(goal);
}

void AutonomousBase::StartDriveRelative(double forward, double theta,
                                        bool follow_through) {
  DrivetrainStatus status;
  if (!drivetrain_status_reader_.ReadLastMessage(&status)) {
    LOG_P("No drivetrain status found.");
    return;
  }

  double left_offset = status->estimated_left_position();
  double right_offset = status->estimated_right_position();

  double left_goal = left_offset + forward - theta * config_.robot_radius;
  double right_goal = right_offset + forward + theta * config_.robot_radius;

  follow_through_ = follow_through;

  if (follow_through) {
    goal_dist_ = (left_goal + right_goal) / 2.0;
    threshold_positive_ = forward > 0.0;

    if (threshold_positive_) {
      left_goal += 3.0;
      right_goal += 3.0;
    } else {
      left_goal -= 3.0;
      right_goal -= 3.0;
    }
  }

  StartDriveAbsolute(left_goal, right_goal, follow_through);
}

void AutonomousBase::StartDrivePath(double x, double y, double heading,
                                    bool follow_through) {
  DrivetrainGoal goal;

  follow_through_ = follow_through;

  goal->mutable_path_command()->set_x_goal(x);
  goal->mutable_path_command()->set_y_goal(y);
  goal->mutable_path_command()->set_theta_goal(heading);

  goal->mutable_linear_constraints()->set_max_velocity(max_forward_velocity_);
  goal->mutable_linear_constraints()->set_max_acceleration(
      max_forward_acceleration_);
  goal->mutable_angular_constraints()->set_max_velocity(max_angular_velocity_);
  goal->mutable_angular_constraints()->set_max_acceleration(
      max_angular_acceleration_);

  drivetrain_goal_queue_->WriteMessage(goal);
}

bool AutonomousBase::IsDriveComplete() {
  DrivetrainGoal goal;
  DrivetrainStatus status;

  if (drivetrain_goal_queue_->ReadLastMessage(&goal) &&
      drivetrain_status_reader_.ReadLastMessage(&status)) {
    if (follow_through_) {
      double distance_travelled = 0.5 * (status->estimated_left_position() +
                                         status->estimated_right_position());
      if (threshold_positive_ && distance_travelled > goal_dist_) {
        return true;
      } else if (!threshold_positive_ && distance_travelled < goal_dist_) {
        return true;
      }
    }

    if (goal->has_distance_command()) {
      if (std::abs(status->estimated_left_position() -
                   goal->distance_command().left_goal()) < 1e-2 &&
          std::abs(status->estimated_right_position() -
                   goal->distance_command().right_goal()) < 1e-2 &&
          std::abs(status->estimated_left_velocity()) < 1e-2 &&
          std::abs(status->estimated_right_velocity()) < 1e-2) {
        return true;
      }
    }

    if (goal->has_path_command()) {
      if (std::abs(status->estimated_left_position() -
                   status->profiled_left_position_goal()) < 1e-2 &&
          std::abs(status->estimated_right_position() -
                   status->profiled_right_position_goal()) < 1e-2 &&
          std::abs(status->estimated_x_position() -
                   goal->path_command().x_goal()) < 2e-1 &&
          std::abs(status->estimated_y_position() -
                   goal->path_command().y_goal()) < 2e-1 &&
          std::abs(status->estimated_left_velocity()) < 1e-2 &&
          std::abs(status->estimated_right_velocity()) < 1e-2) {
        return true;
      }
    }
  }

  return false;
}

void AutonomousBase::operator()() {
  aos::SetCurrentThreadRealtimePriority(10);
  muan::utils::SetCurrentThreadName("Autonomous");

  DriverStationProto driver_station;
  GameSpecificStringProto game_specific_string;

  LOG_P("Autonomous thread starting!");

  while (!driver_station_reader_.ReadLastMessage(&driver_station)) {
    LOG_P("No driver station message!");
    loop_.SleepUntilNext();
  }

  while (driver_station_reader_.ReadLastMessage(&driver_station),
         driver_station->mode() != RobotMode::AUTONOMOUS) {
    loop_.SleepUntilNext();
  }

  while (!game_specific_string_reader_.ReadLastMessage(&game_specific_string)) {
    LOG_P("Waiting on auto because there's no game specific message yet!");
    loop_.SleepUntilNext();
  }

  // Start of autonomous. Grab the game specific string.
  auto left_right_codes = game_specific_string->code();
  LOG_P("Starting autonomous with layout %s", left_right_codes.c_str());
  if (left_right_codes[0] == 'L') {
    if (left_right_codes[1] == 'L') {
      // Switch is left, scale is left
      LOG_P("Running LEFT SWITCH LEFT SCALE auto");

      StartDriveRelative(-4.2, 0.0, true);
      WaitUntilDriveComplete();

      StartDrivePath(5.2, 1, M_PI * .5);
      WaitUntilDriveComplete();

      StartDrivePath(5.7, 4.8, M_PI);
      WaitUntilDriveComplete();

      StartDriveRelative(-1, 0.0, true);
      WaitUntilDriveComplete();

      StartDriveRelative(2.5, 0.0, true);
      WaitUntilDriveComplete();

      StartDrivePath(4.7, 5.2, M_PI * .5);
      WaitUntilDriveComplete();

      StartDrivePath(4.4, 4.2, M_PI);
      WaitUntilDriveComplete();

      StartDrivePath(5.7, 4.8, M_PI);
      WaitUntilDriveComplete();
    } else if (left_right_codes[1] == 'R') {
      // Switch is left, scale is right
      LOG_P("Running LEFT SWITCH RIGHT SCALE auto");

      StartDriveRelative(-3.5, 0.0);
      WaitUntilDriveComplete();

      StartDrivePath(6.65, 1, M_PI * .75);
      WaitUntilDriveComplete();

      StartDrivePath(5.25, 2.5, M_PI * -.5);
      WaitUntilDriveComplete();

      StartDrivePath(4.25, 4.5, M_PI);
      WaitUntilDriveComplete();

      StartDrivePath(4.5, 5, M_PI * .5);
      WaitUntilDriveComplete();

      StartDrivePath(4.25, 5, M_PI);
      WaitUntilDriveComplete();

      StartDrivePath(5.2, 1, M_PI);
      WaitUntilDriveComplete();

      StartDriveRelative(-1.5, 0.0);
      WaitUntilDriveComplete();
    }
  } else if (left_right_codes[0] == 'R') {
    if (left_right_codes[1] == 'L') {
      // Switch is right, scale is left
      LOG_P("Running RIGHT SWITCH LEFT SCALE auto");

      StartDriveRelative(-4.2, 0.0, true);
      WaitUntilDriveComplete();

      StartDrivePath(4.2, 1.3, M_PI);
      WaitUntilDriveComplete();

      StartDrivePath(4.4, 3, M_PI * 0.5);
      WaitUntilDriveComplete();

      StartDrivePath(6.2, 4.5, M_PI);
      WaitUntilDriveComplete();

      StartDriveRelative(2, 0.0);
      WaitUntilDriveComplete();

      StartDriveRelative(-2, 0.0);
      WaitUntilDriveComplete();
    } else if (left_right_codes[1] == 'R') {
      // Switch is right, scale is right
      LOG_P("Running RIGHT SWITCH RIGHT SCALE auto");

      StartDriveRelative(-3.5, 0.0, true);
      WaitUntilDriveComplete();

      StartDrivePath(6.65, 1, M_PI * -.3);
      WaitUntilDriveComplete();

      StartDriveRelative(2.25, 0.0, true);
      WaitUntilDriveComplete();

      StartDrivePath(4.5, 0.8, M_PI * -.5);
      WaitUntilDriveComplete();

      StartDrivePath(4.2, 1.3, M_PI * -.8);
      WaitUntilDriveComplete();

      StartDrivePath(6.65, 1, 0.0);
      WaitUntilDriveComplete();
    }
  }
}

void AutonomousBase::WaitUntilDriveComplete() {
  while (!IsDriveComplete()) {
    loop_.SleepUntilNext();
  }
}

void AutonomousBase::IntakeGround() {
  score_goal_->set_elevator_height(score_subsystem::ElevatorHeight::HEIGHT_0);
  score_goal_->set_claw_mode(score_subsystem::ClawMode::SCORE_F);
  score_goal_->set_intake_mode(score_subsystem::IntakeMode::INTAKE);
  score_goal_queue_->WriteMessage(score_goal_);
}

void AutonomousBase::StopIntakeGround() {
  score_goal_->set_claw_mode(score_subsystem::ClawMode::SCORE_F);
  score_goal_->set_intake_mode(score_subsystem::IntakeMode::HOLD);
  score_goal_queue_->WriteMessage(score_goal_);
}

void AutonomousBase::MoveToSwitch() {
  score_goal_->set_elevator_height(score_subsystem::ElevatorHeight::HEIGHT_2);
  score_goal_->set_claw_mode(score_subsystem::ClawMode::SCORE_F);
  score_goal_queue_->WriteMessage(score_goal_);
}

void AutonomousBase::MoveToScale(bool front) {
  score_goal_->set_elevator_height(
      score_subsystem::ElevatorHeight::HEIGHT_SCORE);
  score_goal_->set_claw_mode(front ? score_subsystem::ClawMode::SCORE_F
                                   : score_subsystem::ClawMode::SCORE_B);
  score_goal_queue_->WriteMessage(score_goal_);
}

void AutonomousBase::Score() {
  score_goal_->set_intake_mode(score_subsystem::IntakeMode::OUTTAKE);
  score_goal_queue_->WriteMessage(score_goal_);
}

bool AutonomousBase::IsAtScoreHeight() {
  if (score_status_reader_.ReadLastMessage(&score_status_)) {
    return (std::abs(score_status_->elevator_unprofiled_goal() -
                     score_status_->elevator_actual_height()) < 1e-2);
  } else {
    return false;
  }
}

}  // namespace autonomous

}  // namespace c2018
