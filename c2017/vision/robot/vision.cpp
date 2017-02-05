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
      action_{muan::actions::DrivetrainAction::PointTurn(0, true, properties_,
                  c2017::QueueManager::GetInstance().drivetrain_goal_queue(),
                  c2017::QueueManager::GetInstance().drivetrain_status_queue())} {}

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
  } else {
    status->set_has_connection(false);
    status->set_target_found(false);
    status->set_aligned(false);
  }

  if (!disabled && should_align_) {
    if (!running_) {
      running_ = true;
      action_ = muan::actions::DrivetrainAction::PointTurn(-status->angle_to_target(), true, properties_,
                     c2017::QueueManager::GetInstance().drivetrain_goal_queue(),
                     c2017::QueueManager::GetInstance().drivetrain_status_queue());
    } else {
      action_.Update();
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
