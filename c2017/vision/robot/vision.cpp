#include "c2017/vision/robot/vision.h"
#include <iostream>

namespace c2017 {
namespace vision {

VisionSubsystem::VisionSubsystem()
    : running_{false},
      should_align_{false},
      properties_{1.0, 1.0, 1.0, 1.0, c2017::drivetrain::GetDrivetrainConfig().robot_radius},
      vision_input_reader_{QueueManager::GetInstance().vision_input_queue().MakeReader()},
      driverstation_reader_{QueueManager::GetInstance().driver_station_queue()->MakeReader()},
      dt_goal_queue_{c2017::QueueManager::GetInstance().drivetrain_goal_queue()},
      dt_status_reader_{c2017::QueueManager::GetInstance().drivetrain_status_queue()->MakeReader()},
      moving_average_l_{muan::utils::History<double>(20)},
      moving_average_r_{muan::utils::History<double>(20)} {}

void VisionSubsystem::Update() {
  auto ds = driverstation_reader_.ReadLastMessage();
  bool disabled = ds ? ((*ds)->mode() == RobotMode::DISABLED || (*ds)->mode() == RobotMode::ESTOP) : true;

  VisionStatusProto status;
  status->set_target_found(false);
  status->set_has_connection(false);
  if (auto input = vision_input_reader_.ReadLastMessage()) {
    status->set_has_connection(true);
    status->set_target_found((*input)->target_found());
    if ((*input)->has_distance_to_target()) {
      status->set_distance_to_target((*input)->distance_to_target());
    }
    if ((*input)->has_angle_to_target()) {
      status->set_angle_to_target((*input)->angle_to_target());
      status->set_aligned(std::abs((*input)->angle_to_target()) < 0.05);
    } else {
      status->set_aligned(false);
    }
  } else {
    status->set_has_connection(false);
    status->set_target_found(false);
    status->set_aligned(false);
  }

  if (!disabled && should_align_) {
    if (!running_) {
      running_ = true;
    } else {
      // Send drivetrain goal
      double left_offset = 0, right_offset = 0;
      auto maybe_status = dt_status_reader_.ReadLastMessage();
      if (maybe_status) {
        auto dt_status = maybe_status.value();
        left_offset = dt_status->estimated_left_position();
        right_offset = dt_status->estimated_right_position();
      }
      double distance = 0.2 * -status->angle_to_target() * properties_.wheelbase_radius;
      moving_average_l_.Update(left_offset - distance);
      moving_average_r_.Update(right_offset + distance);
      double goal_r = 0, goal_l = 0;
      for (auto i : moving_average_l_) {
        goal_r += moving_average_r_.GoBack(i) / 20;
        goal_l += moving_average_l_.GoBack(i) / 20;
      }

      frc971::control_loops::drivetrain::GoalProto goal;
      goal->mutable_angular_constraints()->set_max_velocity(properties_.max_angular_velocity);
      goal->mutable_angular_constraints()->set_max_acceleration(properties_.max_angular_acceleration);
      goal->mutable_linear_constraints()->set_max_velocity(properties_.max_forward_velocity);
      goal->mutable_linear_constraints()->set_max_acceleration(properties_.max_forward_acceleration);
      goal->mutable_distance_command()->set_left_goal(goal_l);
      goal->mutable_distance_command()->set_right_goal(goal_r);
      goal->mutable_distance_command()->set_left_velocity_goal(0);
      goal->mutable_distance_command()->set_right_velocity_goal(0);
      goal->set_gear(frc971::control_loops::drivetrain::Gear::kHighGear);
      dt_goal_queue_->WriteMessage(goal);

      // Determine if it is terminated
      bool terminated = false;
      if (maybe_status) {
        auto dt_status = maybe_status.value();
        terminated =
            std::abs(status->angle_to_target()) < 0.02 &&
            std::abs(dt_status->estimated_left_velocity()) < 0.01 &&
            std::abs(dt_status->estimated_right_velocity()) < 0.01;
      }
      if (terminated) { running_ = false; }
    }
  } else {
    running_ = false;
  }
  QueueManager::GetInstance().vision_status_queue().WriteMessage(status);
}

void VisionSubsystem::SetGoal(VisionGoalProto goal) {
  should_align_ = goal->should_align();
}

}  // namespace vision
}  // namespace c2017
