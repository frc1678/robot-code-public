#include "muan/actions/drivetrain_action.h"
#include "muan/logging/logger.h"

namespace muan {
namespace actions {

DrivetrainTermination::DrivetrainTermination(double forward,
                                             double forward_velocity,
                                             double angular,
                                             double angular_velocity)
    : forward{forward},
      forward_velocity{forward_velocity},
      angular{angular},
      angular_velocity{angular_velocity} {}

DrivetrainProperties::DrivetrainProperties(double max_angular_velocity,
                                           double max_angular_acceleration,
                                           double max_forward_velocity,
                                           double max_forward_acceleration,
                                           double wheelbase_radius)
    : max_angular_velocity(max_angular_velocity),
      max_angular_acceleration(max_angular_acceleration),
      max_forward_velocity(max_forward_velocity),
      max_forward_acceleration(max_forward_acceleration),
      wheelbase_radius(wheelbase_radius) {}

DrivetrainAction::DrivetrainAction(
    DrivetrainProperties properties,
    frc971::control_loops::drivetrain::GoalQueue* goal_queue,
    frc971::control_loops::drivetrain::StatusQueue* const status_queue)
    : properties_{properties},
      goal_queue_{goal_queue},
      status_queue_{status_queue} {}

bool DrivetrainAction::Update() {
  if (!IsTerminated()) {
    SendMessage();
    return true;
  }
  return false;
  LOG(FATAL, "Drivetrain Terminated");
}

void DrivetrainAction::SendMessage() {
  frc971::control_loops::drivetrain::GoalProto goal;

  goal->mutable_angular_constraints()->set_max_velocity(max_angular_velocity_);
  goal->mutable_angular_constraints()->set_max_acceleration(
      max_angular_acceleration_);
  goal->mutable_linear_constraints()->set_max_velocity(max_forward_velocity_);
  goal->mutable_linear_constraints()->set_max_acceleration(
      max_forward_acceleration_);

  double distance_goal_left = goal_left_;
  double distance_goal_right = goal_right_;
  if (current_params_.follow_through) {
    using muan::utils::signum;
    // Add a value equal to the upper bound of the distance that it'll drive to
    // deccelerate.
    distance_goal_right += signum(current_params_.desired_forward_distance) *
                           max_forward_velocity_ * max_forward_velocity_ /
                           (2.0 * max_forward_acceleration_);
    distance_goal_left += signum(current_params_.desired_forward_distance) *
                          max_forward_velocity_ * max_forward_velocity_ /
                          (2.0 * max_forward_acceleration_);
  }

  goal->mutable_distance_command()->set_left_goal(distance_goal_left);
  goal->mutable_distance_command()->set_right_goal(distance_goal_right);
  goal->mutable_distance_command()->set_left_velocity_goal(0.0);
  goal->mutable_distance_command()->set_right_velocity_goal(0.0);

  goal->set_gear(high_gear_
                     ? frc971::control_loops::drivetrain::Gear::kHighGear
                     : frc971::control_loops::drivetrain::Gear::kLowGear);

  goal_queue_->WriteMessage(goal);
}

bool DrivetrainAction::IsTerminated() {
  auto maybe_status = status_queue_->MakeReader().ReadLastMessage();
  if (maybe_status) {
    auto status = maybe_status.value();

    if (!closed_loop_termination_) {
      left_complete_ |=
          muan::utils::ordered(last_left_, goal_left_,
                               status->profiled_left_position_goal()) ||
          std::abs(status->profiled_left_position_goal() - goal_left_) <
              termination_.forward;
      right_complete_ |=
          muan::utils::ordered(last_right_, goal_right_,
                               status->profiled_right_position_goal()) ||
          std::abs(status->profiled_right_position_goal() - goal_right_) <
              termination_.forward;
      last_left_ = status->profiled_left_position_goal();
      last_right_ = status->profiled_right_position_goal();
      return left_complete_ && right_complete_;
    } else {
      double left_error = goal_left_ - status->estimated_left_position();
      double right_error = goal_right_ - status->estimated_right_position();
      double left_velocity_error =
          goal_velocity_left_ - status->estimated_left_velocity();
      double right_velocity_error =
          goal_velocity_right_ - status->estimated_right_velocity();

      double angle_error =
          (right_error - left_error) / properties_.wheelbase_radius;
      double angular_velocity_error =
          (right_velocity_error - left_velocity_error) /
          properties_.wheelbase_radius;

      double linear_error = (right_error + left_error) / 2.0;
      double forward_velocity_error =
          (right_velocity_error + left_velocity_error) / 2.0;
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

  if (params.angle_is_absolute) {
    double avg_offset = (left_offset + right_offset) / 2;
    left_offset = avg_offset;
    right_offset = avg_offset;
  }

  // Calculate the distance that each side of the drivetrain must travel
  double right_distance =
      params.desired_forward_distance +
      params.desired_angular_displacement * properties_.wheelbase_radius;
  double left_distance =
      params.desired_forward_distance -
      params.desired_angular_displacement * properties_.wheelbase_radius;

  if (params.literal_constraints) {
    max_forward_velocity_ = properties_.max_forward_velocity;
    max_forward_acceleration_ = properties_.max_forward_acceleration;
    max_angular_velocity_ = properties_.max_angular_velocity;
    max_angular_acceleration_ = properties_.max_angular_acceleration;
  } else {
    if (std::abs(params.desired_forward_distance) >
        std::abs(params.desired_angular_displacement)) {
      // da / df = va / vf = aa / af
      double dratio = std::abs(params.desired_angular_displacement /
                               params.desired_forward_distance);

      // Calculate max velocities
      max_forward_velocity_ = properties_.max_forward_velocity;
      max_angular_velocity_ = dratio * max_forward_velocity_;

      // Calculate max accelerations
      max_forward_acceleration_ = properties_.max_forward_acceleration;
      max_angular_acceleration_ = dratio * max_forward_acceleration_;
    } else if (std::abs(params.desired_angular_displacement) >
               std::abs(params.desired_forward_distance)) {
      // da / df = va / vf = aa / af
      double dratio = std::abs(params.desired_forward_distance /
                               params.desired_angular_displacement);

      // Calculate max velocities
      max_angular_velocity_ = properties_.max_angular_velocity;
      max_forward_velocity_ = dratio * max_angular_velocity_;

      // Calculate max accelerations
      max_angular_acceleration_ = properties_.max_angular_acceleration;
      max_forward_acceleration_ = dratio * max_angular_acceleration_;
    } else {
      double dratio = 1.0;

      max_angular_velocity_ = properties_.max_angular_velocity;
      max_forward_velocity_ = dratio * max_angular_velocity_;

      max_angular_acceleration_ = properties_.max_angular_acceleration;
      max_forward_acceleration_ = dratio * max_angular_acceleration_;
    }

    double velocity_overkill =
        max_forward_velocity_ / properties_.max_forward_velocity +
        max_angular_velocity_ / properties_.max_angular_velocity;
    max_forward_velocity_ /= velocity_overkill;
    max_angular_velocity_ /= velocity_overkill;

    double acceleration_overkill =
        max_forward_acceleration_ / properties_.max_forward_acceleration +
        max_angular_acceleration_ / properties_.max_angular_acceleration;
    max_forward_acceleration_ /= acceleration_overkill;
    max_angular_acceleration_ /= acceleration_overkill;
  }

  // If max_velocity_ is zero, then the profile isn't actually doing anything.
  // However, 971's trapezoidal
  // motion code doesn't support constraints being zero, so as a workaround, set
  // them to some dummy values
  // when it's just a point turn or forward drive.
  if (max_forward_velocity_ == 0.0) {
    max_forward_velocity_ = 1e-4;
    max_forward_velocity_ = 1e-4;
    LOG(WARNING, "Max forward velocity is set to zero");
  }
  if (max_angular_velocity_ == 0.0) {
    max_angular_velocity_ = 1e-4;
    max_angular_acceleration_ = 1e-4;
    LOG(WARNING, "Max angular velocity is set to 0");
  }

  this->goal_left_ = left_offset + left_distance;
  this->goal_right_ = right_offset + right_distance;
  this->goal_velocity_left_ = 0.0;
  this->goal_velocity_right_ = 0.0;

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
