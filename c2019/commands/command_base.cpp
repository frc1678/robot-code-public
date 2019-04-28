#include "c2019/commands/command_base.h"

#include <chrono>

#include "c2019/subsystems/limelight/queue_types.h"
#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace commands {

using c2019::limelight::LimelightStatusProto;
using c2019::superstructure::SuperstructureGoalProto;
using c2019::superstructure::SuperstructureStatusProto;
using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;
using muan::wpilib::GameSpecificStringProto;

CommandBase::CommandBase()
    : driver_station_reader_(
          QueueManager<DriverStationProto>::Fetch()->MakeReader()),
      game_specific_string_reader_(
          QueueManager<GameSpecificStringProto>::Fetch()->MakeReader()),
      drivetrain_goal_queue_(QueueManager<DrivetrainGoal>::Fetch()),
      drivetrain_status_reader_(
          QueueManager<DrivetrainStatus>::Fetch()->MakeReader()),
      auto_status_queue_(QueueManager<AutoStatusProto>::Fetch()),
      auto_goal_reader_(QueueManager<AutoGoalProto>::Fetch()->MakeReader()) {}

bool CommandBase::IsAutonomous() {
  DriverStationProto driver_station;
  AutoGoalProto auto_goal;
  if (!driver_station_reader_.ReadLastMessage(&driver_station)) {
    LOG(WARNING, "No driver station status found.");
    ExitAutonomous();
    return false;
  }

  if (!driver_station->is_sys_active()) {
    LOG(WARNING, "Tried to run command while disabled.");
    ExitAutonomous();
    return false;
  }

  QueueManager<AutoGoalProto>::Fetch()->ReadLastMessage(&auto_goal);

  if (auto_goal->cancel_command()) {
    ExitAutonomous();
    return false;
  }

  return driver_station->mode() == RobotMode::AUTONOMOUS;
}

void CommandBase::EnterAutonomous() {
  AutoStatusProto auto_status;
  LOG(INFO, "Started running command");
  auto_status->set_running_command(true);
  auto_status_queue_->WriteMessage(auto_status);
}

void CommandBase::ExitAutonomous() {
  AutoStatusProto auto_status;
  LOG(INFO, "Exited running command");
  auto_status->set_running_command(false);
  auto_status_queue_->WriteMessage(auto_status);
}

void CommandBase::StartDrivePath(double x, double y, double heading,
                                 int direction, bool gear, bool full_send,
                                 double extra_distance_initial,
                                 double extra_distance_final,
                                 double path_voltage) {
  DrivetrainGoal goal;

  Eigen::Vector2d goal_field = (Eigen::Vector2d() << x, y).finished();
  Eigen::Vector2d goal_local = transform_f0_.inverse() * goal_field;

  goal->mutable_path_goal()->set_x(goal_local(0));
  goal->mutable_path_goal()->set_y(goal_local(1));
  goal->mutable_path_goal()->set_heading(heading + theta_offset_);
  goal->mutable_path_goal()->set_max_voltage(path_voltage);
  goal->mutable_path_goal()->set_extra_distance_initial(extra_distance_initial);
  goal->mutable_path_goal()->set_extra_distance_final(extra_distance_final);
  goal->mutable_path_goal()->set_full_send(full_send);
  goal->mutable_path_goal()->set_max_linear_velocity(max_lin_);

  goal->mutable_path_goal()->set_final_velocity(final_vel_);
  goal->mutable_path_goal()->set_max_linear_accel(max_acc_);

  goal->set_high_gear(gear);

  if (direction == 1) {
    goal->mutable_path_goal()->set_backwards(false);
  } else if (direction == -1) {
    goal->mutable_path_goal()->set_backwards(true);
  }

  drivetrain_goal_queue_->WriteMessage(goal);
}

void CommandBase::StartPointTurn(double theta) {
  DrivetrainGoal goal;
  goal->set_point_turn_goal(theta);
  goal->set_high_gear(false);
  drivetrain_goal_queue_->WriteMessage(goal);
}

bool CommandBase::StartDriveVision(double target_dist) {
  // run vision align stuff
  DrivetrainGoal drivetrain_goal;
  LimelightStatusProto lime_status;
  QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);

  int no_target = 0;
  while (!lime_status->has_target() && IsAutonomous()) {
    loop_.SleepUntilNext();
    QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);
    no_target++;
    if (no_target > 70) {
      LOG(WARNING, "Didn't get target");
      return false;
    }
  }

  no_target = 0;
  double offset = 0;
  while (lime_status->target_dist() > target_dist && IsAutonomous()) {
    if (!lime_status->has_target()) {
      no_target++;
    } else {
      no_target = 0;
    }
    if (no_target > 30) {
      LOG(WARNING, "Didn't get target while tracking");
      return false;
    }
  double skew = lime_status->skew();
  if (lime_status->skew() > -45) {
    skew = std::abs(lime_status->skew());
  } else {
    skew += 90;
  }

    if (skew > 5) {
        offset =
              0.05 * (lime_status->to_the_left() ? 1 : -1) * (skew / 13);
    }
    drivetrain_goal->mutable_arc_goal()->set_angular(
        lime_status->horiz_angle() - offset);
    drivetrain_goal->mutable_arc_goal()->set_linear(
        (lime_status->target_dist() - 0.35) * 5.5);

    QueueManager<DrivetrainGoal>::Fetch()->WriteMessage(drivetrain_goal);
    loop_.SleepUntilNext();
    QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);
  }

  if (lime_status->target_dist() > target_dist ||
      !lime_status->limelight_ok()) {
    LOG(WARNING, "Couldn't converge");
    return false;
  }
  return true;
}

bool CommandBase::StartDriveVisionBottom() {
  // run vision align stuff
  DrivetrainGoal drivetrain_goal;
  LimelightStatusProto lime_status;
  QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);

  int no_target = 0;
  while (!lime_status->pricey_has_target() && IsAutonomous()) {
    loop_.SleepUntilNext();
    QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);
    no_target++;
    if (no_target > 70) {
      LOG(WARNING, "Didn't get target");
      return false;
    }
  }

  no_target = 0;
  while (lime_status->pricey_target_dist() > 0.36 && IsAutonomous()) {
    if (!lime_status->pricey_has_target()) {
      no_target++;
    } else {
      no_target = 0;
    }
    if (no_target > 30) {
      LOG(WARNING, "Didn't get target while tracking");
      return false;
    }
    drivetrain_goal->mutable_arc_goal()->set_angular(
        lime_status->pricey_horiz_angle());
    drivetrain_goal->mutable_arc_goal()->set_linear(
        (lime_status->pricey_target_dist() + 0.1) * 3.3);

    QueueManager<DrivetrainGoal>::Fetch()->WriteMessage(drivetrain_goal);
    loop_.SleepUntilNext();
    QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);
  }

  if (lime_status->pricey_target_dist() > 0.38 ||
      !lime_status->pricey_has_target() || !lime_status->bottom_limelight_ok()) {
    LOG(WARNING, "Couldn't converge");
    return false;
  }
  return true;
}

bool CommandBase::StartDriveVisionBackwards() {
  if (!IsAutonomous()) {
    ExitAutonomous();
    return false;
  }
  // run vision align stuff
  DrivetrainGoal drivetrain_goal;
  LimelightStatusProto lime_status;
  QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);

  int no_target = 0;
  while (!lime_status->back_has_target() && IsAutonomous()) {
    loop_.SleepUntilNext();
    QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);
    no_target++;
    if (no_target > 70) {
      return false;
    }
  }

  no_target = 0;
  while (lime_status->back_target_dist() > 0.41 &&
         lime_status->back_has_target() && IsAutonomous()) {
    if (!lime_status->back_has_target()) {
      no_target++;
    } else {
      no_target = 0;
    }
    if (no_target > 5) {
      return false;
    }

    QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);
    drivetrain_goal->mutable_arc_goal()->set_angular(
        lime_status->back_horiz_angle());
    drivetrain_goal->mutable_arc_goal()->set_linear(
        (lime_status->back_target_dist() + 0.25) * -4.0);

    QueueManager<DrivetrainGoal>::Fetch()->WriteMessage(drivetrain_goal);
    loop_.SleepUntilNext();
  }

  if (lime_status->back_target_dist() > 0.41) {
    return false;
  }
  return true;
}

void CommandBase::HoldPosition() {
  if (!IsAutonomous()) {
    ExitAutonomous();
    return;
  }

  DrivetrainGoal drivetrain_goal;
  drivetrain_goal->mutable_linear_angular_velocity_goal()->set_linear_velocity(
      0);
  drivetrain_goal->mutable_linear_angular_velocity_goal()->set_angular_velocity(
      0);

  QueueManager<DrivetrainGoal>::Fetch()->WriteMessage(drivetrain_goal);
}

void CommandBase::GoTo(superstructure::ScoreGoal score_goal,
                       superstructure::IntakeGoal intake_goal) {
  SuperstructureGoalProto super_goal;
  super_goal->set_score_goal(score_goal);
  super_goal->set_intake_goal(intake_goal);

  QueueManager<SuperstructureGoalProto>::Fetch()->WriteMessage(super_goal);
}

void CommandBase::WaitForElevatorAndLL() {
  SuperstructureStatusProto super_status;
  LimelightStatusProto lime_status;
  while (!lime_status->pricey_has_target() &&
         super_status->elevator_height() < 0.6 && IsAutonomous()) {
    loop_.SleepUntilNext();
    QueueManager<SuperstructureStatusProto>::Fetch()->ReadLastMessage(
        &super_status);
    QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);
  }
}

void CommandBase::ScoreHatch(int num_ticks) {
  for (int i = 0; i < num_ticks && IsAutonomous(); i++) {
    SuperstructureGoalProto super_goal;
    super_goal->set_score_goal(superstructure::NONE);
    super_goal->set_intake_goal(superstructure::OUTTAKE_HATCH);
    loop_.SleepUntilNext();

    QueueManager<SuperstructureGoalProto>::Fetch()->WriteMessage(super_goal);
  }
}

bool CommandBase::IsDriveComplete() {
  DrivetrainGoal goal;
  DrivetrainStatus status;

  if (drivetrain_goal_queue_->ReadLastMessage(&goal) &&
      drivetrain_status_reader_.ReadLastMessage(&status)) {
    if (goal->has_path_goal()) {
      if (std::abs(status->profiled_x_goal() - goal->path_goal().x()) < 1e-1 &&
          std::abs(status->profiled_y_goal() - goal->path_goal().y()) < 1e-1 &&
          status->profile_complete()) {
        return true;
      }
    } else if (goal->has_point_turn_goal()) {
      if (status->point_turn_complete()) {
        return true;
      }
    }
  }

  return false;
}

void CommandBase::WaitUntilDriveComplete() {
  while (!IsDriveComplete() && IsAutonomous()) {
    loop_.SleepUntilNext();
  }
}

void CommandBase::WaitForHatch() {
  SuperstructureStatusProto super_status;
  while (!super_status->has_hp_hatch() && IsAutonomous()) {
    loop_.SleepUntilNext();
    QueueManager<SuperstructureStatusProto>::Fetch()->ReadLastMessage(
        &super_status);
  }
}


void CommandBase::WaitForCargo() {
  SuperstructureStatusProto super_status;
  while (!super_status->has_cargo() && IsAutonomous()) {
    loop_.SleepUntilNext();
    QueueManager<SuperstructureStatusProto>::Fetch()->ReadLastMessage(
        &super_status);
  }
}

bool CommandBase::IsDrivetrainNear(double x, double y, double distance) {
  DrivetrainStatus status;

  if (drivetrain_status_reader_.ReadLastMessage(&status)) {
    Eigen::Vector2d field_position =
        transform_f0_ * (Eigen::Vector2d() << status->profiled_x_goal(),
                         status->profiled_y_goal())
                            .finished();
    if ((field_position(0) - x) * (field_position(0) - x) +
            (field_position(1) - y) * (field_position(1) - y) <
        distance * distance) {
      return true;
    }
  }
  return false;
}

void CommandBase::WaitUntilDrivetrainNear(double x, double y, double distance) {
  while (!IsDrivetrainNear(x, y, distance)) {
    loop_.SleepUntilNext();
  }
}

void CommandBase::Wait(uint32_t num_cycles) {
  for (uint32_t i = 0; IsAutonomous() && i < num_cycles; i++) {
    loop_.SleepUntilNext();
  }
}

void CommandBase::SetFieldPosition(double x, double y, double theta) {
  transform_f0_ = Eigen::Translation<double, 2>(Eigen::Vector2d(x, y)) *
                  Eigen::Rotation2D<double>(theta);
  DrivetrainStatus status;
  if (!drivetrain_status_reader_.ReadLastMessage(&status)) {
    LOG(WARNING,
        "Can't read a drivetrain message, so field-centric positioning isn't "
        "going to work right.");
  }

  // The current position relative to the robot's power-on position, from the
  // Cartesian estimator in the drivetrain code.
  Eigen::Transform<double, 2, Eigen::AffineCompact> current_to_robot;
  // The current position relative to the field, defined by the parameters of
  // this function.
  Eigen::Transform<double, 2, Eigen::AffineCompact> current_to_field;

  current_to_robot = Eigen::Translation<double, 2>(
                         (Eigen::Vector2d() << status->estimated_x_position(),
                          status->estimated_y_position())
                             .finished()) *
                     Eigen::Rotation2D<double>(status->estimated_heading());
  current_to_field =
      Eigen::Translation<double, 2>((Eigen::Vector2d() << x, y).finished()) *
      Eigen::Rotation2D<double>(theta);

  // transform_f0_ is "robot to field" = "robot to current" * "current to
  // field"
  transform_f0_ = current_to_field * current_to_robot.inverse();
  theta_offset_ = status->estimated_heading() - theta;
}

}  // namespace commands
}  // namespace c2019
