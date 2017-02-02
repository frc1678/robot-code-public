#include "c2017/vision/robot/vision.h"
#include <iostream>

namespace c2017 {
namespace vision {

VisionSubsystem::VisionSubsystem()
    : should_align_{true},
      running_{false},
      properties_{7.0, 5.0, 3.0, 2.0, c2017::drivetrain::GetDrivetrainConfig().robot_radius},
      vision_input_reader_{QueueManager::GetInstance().vision_input_queue().MakeReader()},
      driverstation_reader_{QueueManager::GetInstance().driver_station_queue()->MakeReader()},
      dt_goal_queue_{c2017::QueueManager::GetInstance().drivetrain_goal_queue()},
      dt_status_reader_{c2017::QueueManager::GetInstance().drivetrain_status_queue()->MakeReader()},
      action_{muan::actions::DrivetrainAction(properties_,
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
    }
  } else {
    status->set_has_connection(false);
    status->set_target_found(false);
  }

  bool terminated = false;
  if (!disabled) {
    if (!running_) {
      muan::actions::DrivetrainActionParams params;
      params.termination = muan::actions::DrivetrainTermination{0.05, 0.05, 0.05, 0.05};
      params.desired_angular_displacement = -status->angle_to_target();
      action_.ExecuteDrive(params);
      running_ = true;
    } else {
      terminated = !action_.Update();
    }
  } else {
    running_ = false;
  }
  status->set_aligned(terminated);
  QueueManager::GetInstance().vision_status_queue().WriteMessage(status);
}

void VisionSubsystem::SetGoal(VisionGoalProto goal) { should_align_ = goal->should_align(); }

}  // namespace vision
}  // namespace c2017
