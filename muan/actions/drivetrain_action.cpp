#include "muan/actions/drivetrain_action.h"
#include <iostream>

namespace muan {

namespace actions {

DrivetrainAction::DrivetrainAction(DrivetrainProperties properties,
                                   frc971::control_loops::drivetrain::GoalQueue* goal_queue,
                                   frc971::control_loops::drivetrain::StatusQueue* const status_queue)
    : properties_{properties}, goal_queue_{goal_queue}, status_queue_{status_queue} {}

bool DrivetrainAction::Update() {
  if (!IsTerminated()) {
    SendMessage();
    return true;
  }
  return false;
}

void DrivetrainAction::SendMessage() {
  frc971::control_loops::drivetrain::GoalProto goal;

  goal->mutable_angular_constraints()->set_max_velocity(max_angular_velocity_);
  goal->mutable_angular_constraints()->set_max_acceleration(max_angular_acceleration_);
  goal->mutable_linear_constraints()->set_max_velocity(max_forward_velocity_);
  goal->mutable_linear_constraints()->set_max_acceleration(max_forward_acceleration_);

  goal->mutable_distance_command()->set_left_goal(goal_left_);
  goal->mutable_distance_command()->set_right_goal(goal_right_);
  goal->mutable_distance_command()->set_left_velocity_goal(goal_velocity_left_);
  goal->mutable_distance_command()->set_right_velocity_goal(goal_velocity_right_);

  goal->set_gear(high_gear_ ? frc971::control_loops::drivetrain::Gear::kHighGear
                            : frc971::control_loops::drivetrain::Gear::kLowGear);

  goal_queue_->WriteMessage(goal);
}

bool DrivetrainAction::IsTerminated() {
  auto maybe_status = status_queue_->MakeReader().ReadLastMessage();
  if (maybe_status) {
    auto status = maybe_status.value();

    if (!closed_loop_termination_) {
      left_complete_ |= muan::utils::ordered(last_left_, goal_left_, status->profiled_left_position_goal());
      right_complete_ |=
          muan::utils::ordered(last_right_, goal_right_, status->profiled_right_position_goal());
      last_left_ = status->profiled_left_position_goal();
      last_right_ = status->profiled_right_position_goal();
      return left_complete_ && right_complete_;
    } else {
      double left_error = goal_left_ - status->estimated_left_position();
      double right_error = goal_right_ - status->estimated_right_position();
      double left_velocity_error = goal_velocity_left_ - status->estimated_left_velocity();
      double right_velocity_error = goal_velocity_right_ - status->estimated_right_velocity();

      double angle_error = (right_error - left_error) / properties_.wheelbase_radius;
      double angular_velocity_error =
          (right_velocity_error - left_velocity_error) / properties_.wheelbase_radius;

      double linear_error = (right_error + left_error) / 2.0;
      double forward_velocity_error = (right_velocity_error + left_velocity_error) / 2.0;

      return std::abs(angle_error) < termination_.angular &&
             std::abs(angular_velocity_error) < termination_.angular_velocity &&
             std::abs(linear_error) < termination_.forward &&
             std::abs(forward_velocity_error) < termination_.forward_velocity;
    }
  } else {
    return false;
  }
}

void DrivetrainAction::ExecuteDrive(DrivetrainActionParams params) {
  // Get the offsets (initial positions) from the status
  double left_offset = 0, right_offset = 0;
  auto maybe_status = status_queue_->MakeReader().ReadLastMessage();
  if (maybe_status) {
    auto status = maybe_status.value();
    left_offset = status->estimated_left_position();
    right_offset = status->estimated_right_position();
  }

  // Calculate the distance that each side of the drivetrain must travel
  double right_distance =
      params.desired_forward_distance + params.desired_angular_displacement * properties_.wheelbase_radius;
  double left_distance =
      params.desired_forward_distance - params.desired_angular_displacement * properties_.wheelbase_radius;

  // The faster side of the drivetrain will go at full speed, the slower side
  // will be slower by a factor of the ratio of the distances
  double left_velocity_max = 0.0, right_velocity_max = 0.0, left_acceleration = 0.0, right_acceleration = 0.0;
  if (std::abs(right_distance) > std::abs(left_distance)) {
    double ratio = std::abs(left_distance / right_distance);

    right_velocity_max = properties_.max_forward_velocity;
    right_acceleration = properties_.max_forward_acceleration;
    left_velocity_max = right_velocity_max * ratio;
    left_acceleration = right_acceleration * ratio;
  } else {
    double ratio = std::abs(right_distance / left_distance);

    left_velocity_max = properties_.max_forward_velocity;
    left_acceleration = properties_.max_forward_acceleration;
    right_velocity_max = left_velocity_max * ratio;
    right_acceleration = left_acceleration * ratio;
  }

  // Calculate the constraints in terms of angular and forward (instead of left/right)
  max_forward_velocity_ = std::abs(left_velocity_max + right_velocity_max) / 2;
  max_forward_acceleration_ = std::abs(left_acceleration + right_acceleration) / 2;
  max_angular_velocity_ = std::abs(right_velocity_max - left_velocity_max) / properties_.wheelbase_radius / 2;
  max_angular_acceleration_ =
      std::abs(right_acceleration - left_acceleration) / properties_.wheelbase_radius / 2;

  this->goal_left_ = left_offset + left_distance;
  this->goal_right_ = right_offset + right_distance;
  this->goal_velocity_left_ = params.follow_through ? left_velocity_max : 0.0;
  this->goal_velocity_right_ = params.follow_through ? right_velocity_max : 0.0;

  this->closed_loop_termination_ = params.closed_loop_termination;
  this->termination_ = params.termination;

  this->high_gear_ = params.high_gear;

  this->current_params_ = params;

  this->left_complete_ = false;
  this->right_complete_ = false;
  last_left_ = left_offset;
  last_right_ = right_offset;
}

}  // namespace actions

}  // namespace muan
