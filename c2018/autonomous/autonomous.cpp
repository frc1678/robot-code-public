#include "c2018/autonomous/autonomous.h"

#include <chrono>

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
                                    int direction) {
  follow_through_ = false;
  DrivetrainGoal goal;

  goal->mutable_path_command()->set_x_goal(x);
  goal->mutable_path_command()->set_y_goal(y);
  goal->mutable_path_command()->set_theta_goal(heading);

  goal->mutable_linear_constraints()->set_max_velocity(max_forward_velocity_);
  goal->mutable_linear_constraints()->set_max_acceleration(
      max_forward_acceleration_);
  goal->mutable_angular_constraints()->set_max_velocity(max_angular_velocity_);
  goal->mutable_angular_constraints()->set_max_acceleration(
      max_angular_acceleration_);

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

void AutonomousBase::operator()() {
  aos::SetCurrentThreadRealtimePriority(10);
  muan::utils::SetCurrentThreadName("Autonomous");

  DriverStationProto driver_station;
  GameSpecificStringProto game_specific_string;

  while (!driver_station_reader_.ReadLastMessage(&driver_station)) {
    LOG(WARNING, "No driver station message!");
    loop_.SleepUntilNext();
  }

  while (driver_station_reader_.ReadLastMessage(&driver_station),
         driver_station->mode() != RobotMode::AUTONOMOUS) {
    loop_.SleepUntilNext();
  }

  while (!game_specific_string_reader_.ReadLastMessage(&game_specific_string)) {
    LOG(ERROR, "Waiting on auto because there's no game specific message yet!");
    loop_.SleepUntilNext();
  }

  // Start of autonomous. Grab the game specific string.
  auto left_right_codes = game_specific_string->code();
  LOG(INFO, "Starting autonomous with layout %s", left_right_codes.c_str());
  if (left_right_codes[0] == 'L') {
    if (left_right_codes[1] == 'L') {
      // Switch is left, scale is left
      LOG(INFO, "Running LEFT SWITCH LEFT SCALE auto");

      MoveToSwitch();

      // Start going back
      StartDriveRelative(-3.0, 0.0, -2.0);
      WaitUntilDriveComplete();

      // Turn
      StartDriveRelative(-2.0, M_PI * 0.5, -2.0);
      WaitUntilDriveComplete();

      // Go over bump
      StartDriveAtAngle(-3.8, M_PI * 0.5, -1.5);
      WaitUntilDriveComplete();

      // Go to scale
      StartDriveAtAngle(-1.6, M_PI * 0.1, -1.0);
      Wait(50);
      MoveToScale(false);
      WaitUntilDriveComplete();
      StartDriveAtAngle(-1.15, M_PI * -0.1, -1.0);
      WaitUntilDriveComplete();

      // Score on scale here
      Score();
      Wait(50);
      StopScore();

      IntakeGround();
      Wait(200);

      // Go to switch
      StartDriveAtAngle(1.75, 0.15, 0.0);
      WaitForCube();
      Wait(25);

      // Back up to prevent dragging cube against switch
      StartDriveRelative(-0.15, 0.0);
      MoveToSwitch();

      Wait(150);

      // Drive so bumpers get on switch
      StartDriveRelative(0.2, 0.0, 1.5);
      WaitUntilDriveComplete();

      StopIntakeGround();

      // Without wait it scores while driving back
      Score();

      // Quickturn to next cube
      StartDriveAtAngle(-0.15, M_PI * 0.3);

      // Not get intake caught on switch
      Wait(125);

      // Go to ground intake
      IntakeGround();

      WaitUntilDriveComplete();

      // Drive to next cube
      StartDriveAtAngle(.7, M_PI * 0.33, 0.0);
      WaitForCube();

      max_angular_acceleration_ = 5.5;
      max_forward_acceleration_ = 2.0;
      // Drive back to scale
      StartDriveAtAngle(-2.2, -0.2, 0.0);
      Wait(100);
      MoveToScale(false);
      WaitUntilDriveComplete();

      Score();
      Wait(50);
      StopScore();

      IntakeGround();
    } else if (left_right_codes[1] == 'R') {
      // Switch is left, scale is right
      LOG(INFO, "Running LEFT SWITCH RIGHT SCALE auto");
      StopIntakeGround();
      MoveTo(c2018::score_subsystem::SCALE_SHOOT);

      // Start drive to scale
      StartDriveRelative(-3.8, 0.0, 2.5);
      WaitUntilDriveComplete();

      // Turn and get to scale
      StartDriveAtAngle(-3.2, M_PI * 0.18, -1.2);
      WaitUntilDriveComplete();

      // Score on scale
      Score();
      Wait(20);
      // Lower intake
      GoToIntake();

      Wait(90);

      // Start backing off scale
      StartDriveRelative(0.3, M_PI * -0.2, 0.5);
      WaitUntilDriveComplete();

      StopScore();
      // Sharp turn to switch
      StartDriveAtAngle(1.4, M_PI * -0.5, 0.8);
      WaitUntilDriveComplete();

      // Go over bump
      StartDriveAtAngle(3.4, M_PI * -0.5, 0.0);
      WaitUntilDriveComplete();

      // Turn to cube on switch
      StartDriveAtAngle(0.0, 0.0);
      WaitUntilDriveComplete();

      IntakeGround();
      // Drive to cube
      {
        double old_forward_velocity = max_forward_velocity_;
        max_forward_velocity_ = 1.0;
        StartDriveRelative(0.3, 0.0, 1.0);
        max_forward_velocity_ = old_forward_velocity;
      }
      WaitForCube();

      // Back up to prevent cube drag
      StartDriveRelative(-0.15, 0.0);
      Wait(100);
      MoveToSwitch();
      Wait(50);

      // Drive so bumpers get on switch
      StartDriveAtAngle(0.2, M_PI * 0.2, 1.5);
      WaitUntilDriveComplete();

      // Without wait it scores while driving back
      Score();

      /*
      // Quickturn towards other cube
      StartDriveAtAngle(0.0, M_PI * 0.25, 0.0);
      WaitUntilDriveComplete();
      IntakeGround();
      Wait(100);

      // Drive to other cube
      StartDriveRelative(0.2, 0.0, 0.0);
      WaitForCube();

      // Drive towards scale
      StartDriveAtAngle(2.0, M_PI * 0.6, 0.6);
      WaitUntilDriveComplete();

      // Back to scale
      StartDriveAtAngle(1.0, M_PI * 0.5, 1.5);
      WaitUntilDriveComplete();

      MoveToScale(true);
      // To scale
      StartDriveRelative(2.0, M_PI * 0.6, 0.0);
      WaitUntilDriveComplete();

      Score();
      Wait(100);
      StopScore();

      StartDriveRelative(-0.5, 0.0);
      IntakeGround();
      WaitUntilDriveComplete();
      */
    }
  } else if (left_right_codes[0] == 'R') {
    if (left_right_codes[1] == 'L') {
      LOG(INFO, "Running RIGHT SWITCH LEFT SCALE auto");

      MoveTo(c2018::score_subsystem::SCALE_LOW_REVERSE);
      // Back up
      StartDriveRelative(-2.2, 0, 1.5);
      WaitUntilDriveComplete();

      // Turn to switch
      StartDriveAtAngle(-1.8, M_PI * 0.5, -1.0);
      WaitUntilDriveComplete();

      Score(false);
      Wait(100);
      GoToIntake();

      {
        double old_forward_velocity = max_forward_velocity_;
        max_forward_velocity_ = 3.0;
        StartDriveAtAngle(2.5, M_PI * 1.5, 1.0);
        WaitUntilDriveComplete();
        IntakeGround();

        max_forward_velocity_ = old_forward_velocity;
      }

      StartDriveAtAngle(5.0, M_PI * 1.5, 0.0);
      WaitUntilDriveComplete();

      Wait(200);

      // Quickturn to scale
      StartDriveAtAngle(0.0, M_PI * 1.85, 0.0);
      WaitUntilDriveComplete();

      // Go to switch
      StartDriveAtAngle(0.45, M_PI * 1.85, 0.0);
      WaitForCube();

      // Back up to prevent dragging cube against switch
      StartDriveAtAngle(-2.15, M_PI * 2, -0.5);
      Wait(50);
      MoveTo(c2018::score_subsystem::SCALE_SHOOT);

      WaitUntilDriveComplete();

      // Without wait it scores while driving back
      Score();
    } else if (left_right_codes[1] == 'R') {
      // Switch is right, scale is right
      LOG(INFO, "Running RIGHT SWITCH RIGHT SCALE auto");

      StopIntakeGround();
      MoveTo(c2018::score_subsystem::SCALE_SHOOT);

      // Start drive to scale
      StartDriveRelative(-3.8, 0.0, 2.5);
      WaitUntilDriveComplete();

      // Turn and get to scale
      StartDriveAtAngle(-3.2, M_PI * 0.18, -1.2);
      WaitUntilDriveComplete();

      // Score on scale
      Score();
      Wait(20);
      // Lower intake
      GoToIntake();

      Wait(90);

      // Drive to switch to pick up cube
      StartDriveAtAngle(1.58, M_PI * -0.07, 0.5);
      Wait(40);
      IntakeGround();
      WaitForCube();
      Wait(25);

      // Back up to prevent dragging cube against switch
      StartDriveRelative(-0.15, 0.0);
      MoveToSwitch();
      // WaitUntilElevatorAtPosition();  // TODO(Livy) Make sure this actually
      // works
      Wait(100);
      StopIntakeGround();

      // Drive so bumpers get on switch
      StartDriveRelative(0.2, 0.0, 0.5);
      WaitUntilDriveComplete();

      // Without wait it scores while driving back
      Score();

      // Turn to get in position to grab second cube
      StartDriveAtAngle(-0.45, M_PI * -0.28, 0.01);
      Wait(150);
      StopScore();
      IntakeGround();
      WaitUntilDriveComplete();

      // Drive forwards to 2nd cube
      {
        double old_angular_acceleration = max_angular_acceleration_;
        max_angular_acceleration_ = 2.0;
        StartDriveAtAngle(1.1, M_PI * -0.2, 0.01);
        max_angular_acceleration_ = old_angular_acceleration;
      }
      WaitForCube();

      // TODO(Livy) If/when we can control elevator speed make it go way slower
      // here
      StopIntakeGround();

      // Start drive to scale

      max_forward_acceleration_ = 1.7;
      StartDriveAtAngle(-1.5, 0.0, 1.7);
      Wait(100);
      MoveTo(c2018::score_subsystem::SCALE_SHOOT);
      WaitUntilDriveComplete();

      StartDriveAtAngle(-0.8, M_PI * 0.1, 1.0);
      WaitUntilDriveComplete();
      Score();
      Wait(100);

      MoveTo(c2018::score_subsystem::INTAKE_0);
      StartDriveRelative(0.5, 0.0);
      Wait(200);
    }
  }
  LOG(INFO, "Finished with auto!");
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
  score_goal->set_intake_goal(score_subsystem::IntakeGoal::INTAKE_NONE);
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
