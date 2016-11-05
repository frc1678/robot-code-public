#include "drivetrain_subsystem.h"

namespace o2016 {

namespace drivetrain {

DrivetrainSubsystem::DrivetrainSubsystem()
    : input_queue_reader_(
          QueueManager::GetInstance().drivetrain_input_queue().MakeReader()),
      output_queue_(QueueManager::GetInstance().drivetrain_output_queue()),
      goal_queue_reader_(
          QueueManager::GetInstance().drivetrain_goal_queue().MakeReader()),
      status_queue_(QueueManager::GetInstance().drivetrain_status_queue()) {}

void DrivetrainSubsystem::Update() {
  UpdateGoals();

  output_queue_.WriteMessage(
      controller_.Update(*input_queue_reader_.ReadLastMessage()));
  auto status = controller_.GetStatus();

  if (status->just_finished_profile()) {
    current_goal_->clear_distance_command();
  }

  status_queue_.WriteMessage(status);
}

void DrivetrainSubsystem::UpdateGoals() {
  bool goal_changed = false;
  std::experimental::optional<DrivetrainGoalProto> goal;

  while (goal = goal_queue_reader_.ReadMessage()) {
    goal_changed = goal_changed || TrySetGoal(*goal);
  }

  if (goal_changed || current_goal_->has_velocity_command()) {
    controller_.SetGoal(current_goal_);
  }
}

bool DrivetrainSubsystem::TrySetGoal(const DrivetrainGoalProto& goal_new) {
  if (!current_goal_->has_distance_command() ||
      goal_new->has_distance_command()) {
    current_goal_ = goal_new;
    return true;
  }
  return false;
}

}  // drivetrain

}  // o2016
