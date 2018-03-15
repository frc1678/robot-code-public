#include "c2018/autonomous/autonomous_base.h"

#include <chrono>
#include <string>

#include "WPILib.h"
#include "c2018/subsystems/drivetrain/drivetrain_base.h"
#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"
#include "muan/units/units.h"

namespace c2018 {
namespace autonomous {

using muan::units::deg;

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
    LOG(WARNING, "No driver station status found.");
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
                                        double final_velocity) {
  DrivetrainStatus status;
  if (!drivetrain_status_reader_.ReadLastMessage(&status)) {
    LOG(WARNING, "No drivetrain status message provided.");
    return;
  }

  double left_offset = status->estimated_left_position();
  double right_offset = status->estimated_right_position();

  double left_goal = left_offset + forward - theta * config_.robot_radius;
  double right_goal = right_offset + forward + theta * config_.robot_radius;

  follow_through_ = std::abs(final_velocity) > 0.0;

  if (follow_through_) {
    goal_dist_ = (left_goal + right_goal) / 2.0;
    threshold_positive_ = forward > 0.0;

    if (threshold_positive_) {
      left_goal +=
          final_velocity * final_velocity / (2 * max_forward_acceleration_);
      right_goal +=
          final_velocity * final_velocity / (2 * max_forward_acceleration_);
    } else {
      left_goal -=
          final_velocity * final_velocity / (2 * max_forward_acceleration_);
      right_goal -=
          final_velocity * final_velocity / (2 * max_forward_acceleration_);
    }
  }

  StartDriveAbsolute(left_goal, right_goal, follow_through_);
}

void AutonomousBase::StartDriveAtAngle(double distance, double theta_absolute,
                                       double final_velocity) {
  DrivetrainStatus status;
  if (!drivetrain_status_reader_.ReadLastMessage(&status)) {
    LOG(ERROR, "No drivetrain status found.");
    return;
  }

  double delta_theta = theta_absolute - status->estimated_heading();

  StartDriveRelative(distance, delta_theta, final_velocity);
}

void AutonomousBase::StartDrivePath(double x, double y, double heading,
                                    int direction,
                                    frc971::control_loops::drivetrain::Gear gear,
                                    double extra_distance_initial,
                                    double extra_distance_final) {
  follow_through_ = false;
  DrivetrainGoal goal;

  goal->mutable_path_command()->set_x_goal(x);
  goal->mutable_path_command()->set_y_goal(y);
  goal->mutable_path_command()->set_theta_goal(heading);
  goal->mutable_path_command()->set_extra_distance_initial(extra_distance_initial);
  goal->mutable_path_command()->set_extra_distance_final(extra_distance_final);

  goal->mutable_linear_constraints()->set_max_acceleration(max_path_acceleration_);
  goal->set_gear(gear);

  if (direction == 1) {
    goal->mutable_path_command()->set_backwards(false);
  } else if (direction == -1) {
    goal->mutable_path_command()->set_backwards(true);
  }

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
                   goal->distance_command().left_goal()) < 3e-2 &&
          std::abs(status->estimated_right_position() -
                   goal->distance_command().right_goal()) < 3e-2 &&
          std::abs(status->estimated_left_velocity()) < 1e-2 &&
          std::abs(status->estimated_right_velocity()) < 1e-2) {
        return true;
      }
    }

    if (goal->has_path_command()) {
      if (std::abs(status->profiled_x_goal() -
                   goal->path_command().x_goal()) < 1e-1 &&
          std::abs(status->profiled_y_goal() -
                   goal->path_command().y_goal()) < 1e-1 &&
          status->profile_complete()) {
        return true;
      }
    }
  }

  return false;
}

bool AutonomousBase::IsDrivetrainNear(double x, double y, double distance) {
  DrivetrainStatus status;

  if (drivetrain_status_reader_.ReadLastMessage(&status)) {
    if ((status->estimated_x_position() - x) *
        (status->estimated_x_position() - x) +
        (status->estimated_y_position() - y) *
        (status->estimated_y_position() - y) < distance * distance) {
      return true;
    }
  }
  return false;
}

void AutonomousBase::WaitUntilDrivetrainNear(double x, double y, double distance) {
  while (!IsDrivetrainNear(x, y, distance)) {
    loop_.SleepUntilNext();
  }
}

void AutonomousBase::Wait(uint32_t num_cycles) {
  for (uint32_t i = 0; IsAutonomous() && i < num_cycles; i++) {
    loop_.SleepUntilNext();
  }
}

bool AutonomousBase::HasCube() {
  score_subsystem::ScoreSubsystemStatusProto status;
  if (!score_status_reader_.ReadLastMessage(&status)) {
    return false;
  }

  return status->has_cube();
}

void AutonomousBase::WaitForCube() {
  while (!HasCube() && IsAutonomous()) {
    loop_.SleepUntilNext();
  }
}

void AutonomousBase::WaitUntilDriveComplete() {
  while (!IsDriveComplete() && IsAutonomous()) {
    loop_.SleepUntilNext();
  }
}

void AutonomousBase::WaitUntilElevatorAtPosition() {
  while (!IsAtScoreHeight() && IsAutonomous()) {
    loop_.SleepUntilNext();
  }
}

void AutonomousBase::IntakeGround() {
  score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_score_goal(score_subsystem::ScoreGoal::INTAKE_0);
  score_goal->set_intake_goal(score_subsystem::IntakeGoal::INTAKE_ONLY);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::GoToIntake() {
  score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_score_goal(score_subsystem::ScoreGoal::INTAKE_0);
  score_goal->set_intake_goal(score_subsystem::IntakeGoal::INTAKE_NONE);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::StopIntakeGround() {
  score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_intake_goal(score_subsystem::IntakeGoal::FORCE_STOP);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::MoveToSwitch() {
  score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_score_goal(score_subsystem::ScoreGoal::SWITCH);
  score_goal->set_intake_goal(score_subsystem::IntakeGoal::INTAKE_NONE);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::MoveTo(c2018::score_subsystem::ScoreGoal goal) {
  score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_score_goal(goal);
  score_goal->set_intake_goal(score_subsystem::IntakeGoal::INTAKE_NONE);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::MoveToScale(bool front) {
  score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_score_goal(
      front ? score_subsystem::ScoreGoal::SCALE_HIGH_FORWARD
            : score_subsystem::ScoreGoal::SCALE_HIGH_REVERSE);
  score_goal->set_intake_goal(score_subsystem::IntakeGoal::INTAKE_NONE);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::Score(bool fast) {
  score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_intake_goal(fast ? score_subsystem::IntakeGoal::OUTTAKE_FAST
                                   : score_subsystem::IntakeGoal::OUTTAKE_SLOW);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::StopScore() {
  score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_intake_goal(score_subsystem::IntakeGoal::FORCE_STOP);
  score_goal_queue_->WriteMessage(score_goal);
}

bool AutonomousBase::IsAtScoreHeight() {
  score_subsystem::ScoreSubsystemStatusProto score_status;
  if (score_status_reader_.ReadLastMessage(&score_status)) {
    return (std::abs(score_status->elevator_unprofiled_goal() -
                     score_status->elevator_actual_height()) < 1e-2);
  } else {
    return false;
  }
}


}  // namespace autonomous

}  // namespace c2018
