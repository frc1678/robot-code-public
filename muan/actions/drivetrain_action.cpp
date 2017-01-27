#include "muan/actions/drivetrain_action.h"

namespace muan {

namespace actions {

DrivetrainAction::DrivetrainAction(DrivetrainProperties properties, bool high_gear, double goal_left,
                                   double goal_right, double goal_velocity_left, double goal_velocity_right,
                                   double threshold_distance, double threshold_velocity,
                                   frc971::control_loops::drivetrain::GoalQueue* goal_queue,
                                   frc971::control_loops::drivetrain::StatusQueue* status_queue)
    : properties_(properties),
      goal_left_(goal_left),
      goal_velocity_left_(goal_velocity_left),
      goal_right_(goal_right),
      goal_velocity_right_(goal_velocity_right),
      threshold_distance_(threshold_distance),
      threshold_velocity_(threshold_velocity),
      high_gear_(high_gear),
      goal_queue_(goal_queue),
      status_queue_(status_queue) {}

bool DrivetrainAction::Update() {
  if (!IsTerminated()) {
    SendMessage();
    return true;
  }
  return false;
}

void DrivetrainAction::SendMessage() {
  frc971::control_loops::drivetrain::GoalProto goal;
  goal->mutable_angular_constraints()->set_max_velocity(properties_.max_angular_velocity);
  goal->mutable_angular_constraints()->set_max_acceleration(properties_.max_angular_acceleration);
  goal->mutable_linear_constraints()->set_max_velocity(properties_.max_forward_velocity);
  goal->mutable_linear_constraints()->set_max_acceleration(properties_.max_forward_acceleration);
  goal->mutable_distance_command()->set_left_goal(goal_left_);
  goal->mutable_distance_command()->set_right_goal(goal_right_);
  goal->mutable_distance_command()->set_left_velocity_goal(goal_velocity_left_);
  goal->mutable_distance_command()->set_right_velocity_goal(goal_velocity_right_);
  goal->set_gear(high_gear_ ? frc971::control_loops::drivetrain::Gear::kHighGear
                            : frc971::control_loops::drivetrain::Gear::kLowGear);
  goal_queue_->WriteMessage(goal);
}

bool DrivetrainAction::IsTerminated() const {
  auto maybe_status = status_queue_->MakeReader().ReadLastMessage();
  if (maybe_status) {
    auto status = maybe_status.value();
    return (std::abs(status->estimated_left_position() - goal_left_) < threshold_distance_ &&
            std::abs(status->estimated_right_position() - goal_right_) < threshold_distance_ &&
            std::abs(status->estimated_left_velocity() - goal_velocity_left_) < threshold_velocity_ &&
            std::abs(status->estimated_right_velocity() - goal_velocity_right_) < threshold_velocity_);
  } else {
    return false;
  }
}

DrivetrainAction DrivetrainAction::DriveStraight(double distance, bool high_gear,
                                                 DrivetrainProperties properties,
                                                 frc971::control_loops::drivetrain::GoalQueue* gq,
                                                 frc971::control_loops::drivetrain::StatusQueue* sq,
                                                 double termination_distance, double termination_velocity) {
  double left_offset = 0, right_offset = 0;
  auto maybe_status = sq->MakeReader().ReadLastMessage();
  if (maybe_status) {
    auto status = maybe_status.value();
    left_offset = status->estimated_left_position();
    right_offset = status->estimated_right_position();
  }

  return DrivetrainAction(properties, high_gear, left_offset + distance, right_offset + distance, 0, 0,
                          termination_distance, termination_velocity, gq, sq);
}

DrivetrainAction DrivetrainAction::PointTurn(double angle, bool high_gear, DrivetrainProperties properties,
                                             frc971::control_loops::drivetrain::GoalQueue* gq,
                                             frc971::control_loops::drivetrain::StatusQueue* sq,
                                             double termination_distance, double termination_velocity) {
  double left_offset = 0, right_offset = 0;
  auto maybe_status = sq->MakeReader().ReadLastMessage();
  if (maybe_status) {
    auto status = maybe_status.value();
    left_offset = status->estimated_left_position();
    right_offset = status->estimated_right_position();
  }

  double distance = angle * properties.wheelbase_radius;
  return DrivetrainAction(properties, high_gear, left_offset - distance, right_offset + distance, 0, 0,
                          termination_distance, termination_velocity, gq, sq);
}

DrivetrainAction DrivetrainAction::SwoopTurn(double distance, double angle, bool high_gear,
                                             DrivetrainProperties properties,
                                             frc971::control_loops::drivetrain::GoalQueue* gq,
                                             frc971::control_loops::drivetrain::StatusQueue* sq,
                                             double termination_distance, double termination_velocity) {
  // Get offsets
  double left_offset = 0, right_offset = 0;
  auto maybe_status = sq->MakeReader().ReadLastMessage();
  if (maybe_status) {
    auto status = maybe_status.value();
    left_offset = status->estimated_left_position();
    right_offset = status->estimated_right_position();
  }

  // Calculate the distance that each side of the drivetrain must travel
  double right_distance = distance + angle * properties.wheelbase_radius;
  double left_distance = distance - angle * properties.wheelbase_radius;

  // The faster side of the drivetrain will go at full speed, the slower side
  // will be slower by a factor of the ratio of the distances
  double rv_max = 0, ra_max = 0, lv_max = 0, la_max = 0;
  if (std::abs(right_distance) > std::abs(left_distance)) {
    double ratio = std::abs(right_distance / left_distance);
    rv_max = properties.max_forward_velocity;
    ra_max = properties.max_forward_acceleration;
    lv_max = rv_max / ratio;
    la_max = ra_max / ratio;
  } else {
    double ratio = std::abs(left_distance / right_distance);
    lv_max = properties.max_forward_velocity;
    la_max = properties.max_forward_acceleration;
    rv_max = lv_max / ratio;
    ra_max = la_max / ratio;
  }

  // Calculate the constraints in terms angular and forward (instead of
  // left/right)
  double max_forward_velocity = (rv_max + lv_max) / 2;
  double max_forward_acceleration = (ra_max + la_max) / 2;
  double max_angular_velocity = std::abs(rv_max - lv_max) / properties.wheelbase_radius / 2;
  double max_angular_acceleration = std::abs(ra_max - la_max) / properties.wheelbase_radius / 2;

  return DrivetrainAction(
      DrivetrainProperties{max_angular_velocity, max_angular_acceleration, max_forward_velocity,
                           max_forward_acceleration, properties.wheelbase_radius},
      high_gear, left_offset + left_distance, right_offset + right_distance, 0, 0, termination_distance,
      termination_velocity, gq, sq);
}

DriveSCurveAction::DriveSCurveAction(double distance, double angle, bool high_gear,
                                     DrivetrainProperties properties,
                                     frc971::control_loops::drivetrain::GoalQueue* gq,
                                     frc971::control_loops::drivetrain::StatusQueue* sq,
                                     double termination_distance, double termination_velocity)
    : DrivetrainAction(properties, high_gear, 0, 0, 0, 0, termination_distance, termination_velocity, gq, sq),
      end_left_(distance),
      end_right_(distance) {
  // Get offsets
  double left_offset = 0, right_offset = 0;
  auto maybe_status = sq->MakeReader().ReadLastMessage();
  if (maybe_status) {
    auto status = maybe_status.value();
    left_offset = status->estimated_left_position();
    right_offset = status->estimated_right_position();
  }

  // Calculate the distance that each side of the drivetrain must travel
  double right_distance = distance / 2 + angle * properties.wheelbase_radius;
  double left_distance = distance / 2 - angle * properties.wheelbase_radius;

  // The faster side of the drivetrain will go at full speed, the slower side
  // will be slower by a factor of the ratio of the distances
  double rv_max = 0, ra_max = 0, lv_max = 0, la_max = 0;
  if (std::abs(right_distance) > std::abs(left_distance)) {
    double ratio = std::abs(right_distance / left_distance);
    rv_max = properties.max_forward_velocity;
    ra_max = properties.max_forward_acceleration;
    lv_max = rv_max / ratio;
    la_max = ra_max / ratio;
  } else {
    double ratio = std::abs(left_distance / right_distance);
    lv_max = properties.max_forward_velocity;
    la_max = properties.max_forward_acceleration;
    rv_max = lv_max / ratio;
    ra_max = la_max / ratio;
  }

  // Calculate the constraints in terms angular and forward (instead of
  // left/right)
  double max_forward_velocity = (rv_max + lv_max) / 2;
  double max_forward_acceleration = (ra_max + la_max) / 2;
  double max_angular_velocity = std::abs(rv_max - lv_max) / properties.wheelbase_radius / 2;
  double max_angular_acceleration = std::abs(ra_max - la_max) / properties.wheelbase_radius / 2;

  goal_left_ = left_distance + left_offset;
  goal_right_ = right_distance + right_offset;
  goal_velocity_left_ = 0;
  goal_velocity_right_ = 0;

  // Remember to add the offset to the final position!
  end_left_ += left_offset;
  end_right_ += right_offset;

  properties_ = DrivetrainProperties{max_angular_velocity, max_angular_acceleration, max_forward_velocity,
                                     max_forward_acceleration, properties.wheelbase_radius};
}

bool DriveSCurveAction::FinishedFirst() {
  auto maybe_status = status_queue_->MakeReader().ReadLastMessage();
  if (maybe_status) {
    auto status = maybe_status.value();
    // Now we want to check against the profiled goal instead of position
    // estimate, because it should all be open-loop
    return (std::abs(status->profiled_left_position_goal() - goal_left_) < 1e-4 &&
            std::abs(status->profiled_right_position_goal() - goal_right_) < 1e-4);
  } else {
    return false;
  }
}

bool DriveSCurveAction::Update() {
  if (!finished_first_) {
    SendMessage();
    if (FinishedFirst()) {
      goal_left_ = end_left_;
      goal_right_ = end_right_;
      goal_velocity_left_ = 0;
      goal_velocity_right_ = 0;
      finished_first_ = true;
    }
    return true;
  } else if (!IsTerminated()) {
    SendMessage();
    return true;
  } else {
    return false;
  }
}

}  // namespace actions

}  // namespace muan
