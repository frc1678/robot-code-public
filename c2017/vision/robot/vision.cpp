#include "c2017/vision/robot/vision.h"

namespace c2017 {
namespace vision {

VisionAlignment::VisionAlignment()
    : should_align_{false},
      use_distance_align_{false},
      running_{false},
      properties_{7.0, 5.0, 3.0, 2.0, c2017::drivetrain::GetDrivetrainConfig().robot_radius},
      vision_input_reader_{QueueManager::GetInstance().vision_input_queue().MakeReader()},
      driverstation_reader_{QueueManager::GetInstance().driver_station_queue()->MakeReader()},
      dt_goal_queue_{c2017::QueueManager::GetInstance().drivetrain_goal_queue()},
      dt_status_reader_{c2017::QueueManager::GetInstance().drivetrain_status_queue()->MakeReader()},
      action_{muan::actions::DrivetrainAction(properties_,
                                              c2017::QueueManager::GetInstance().drivetrain_goal_queue(),
                                              c2017::QueueManager::GetInstance().drivetrain_status_queue())} {
}

void VisionAlignment::Update() {
  auto ds = driverstation_reader_.ReadLastMessage();
  bool disabled = ds ? ((*ds)->mode() == RobotMode::DISABLED || (*ds)->mode() == RobotMode::ESTOP) : true;
  auto goal = QueueManager::GetInstance().vision_goal_queue().ReadLastMessage();
  if (goal) {
    should_align_ = goal.value()->should_align();
    use_distance_align_ = goal.value()->use_distance_align();
  } else {
    should_align_ = false;
    use_distance_align_ = false;
  }

  VisionStatusProto status;
  status->set_target_found(false);
  status->set_has_connection(false);
  if (auto input = vision_input_reader_.ReadLastMessage()) {
    status->set_has_connection(true);
    status->set_target_found((*input)->target_found());
    if ((*input)->has_distance_to_target() && status->target_found()) {
      status->set_distance_to_target((*input)->distance_to_target());
    } else {
      status->set_distance_to_target(constants::kShotDistance);
    }

    if ((*input)->has_angle_to_target() && status->target_found()) {
      status->set_angle_to_target((*input)->angle_to_target());
    } else {
      status->set_angle_to_target(0);
    }
  } else {
    status->set_angle_to_target(0);
    status->set_distance_to_target(constants::kShotDistance);
    status->set_has_connection(false);
    status->set_target_found(false);
  }

  bool terminated = std::abs(status->angle_to_target()) < 0.02;
  if (use_distance_align_ &&
      std::abs(status->distance_to_target() - constants::kShotDistance) < constants::kMaxAlignDistance) {
    terminated = terminated && std::abs(status->distance_to_target() - constants::kShotDistance) < 0.05;
  }

  if (auto dt_status_message = dt_status_reader_.ReadLastMessage()) {
    terminated = terminated && std::abs(dt_status_message.value()->forward_velocity()) < 0.05;
  }

  if (!disabled && should_align_) {
    if (!running_) {
      muan::actions::DrivetrainActionParams params;
      params.termination = muan::actions::DrivetrainTermination{0.05, 0.05, 0.05, 0.05};
      params.desired_angular_displacement = -status->angle_to_target();
      if (use_distance_align_ &&
          std::abs(status->distance_to_target() - constants::kShotDistance) < constants::kMaxAlignDistance) {
        params.desired_forward_distance = status->distance_to_target() - constants::kShotDistance;
      }
      action_.ExecuteDrive(params);
      running_ = true;
    } else {
      // Once it is aligned stop trying to align more. If the profile
      // is done but vision isn't, reset the profile.
      running_ = action_.Update();  // Returns true if profile isn't completed
      should_align_ = running_ || !terminated;
    }
  } else {
    running_ = false;
  }
  status->set_aligned(terminated);
  status->set_aligning(running_);
  QueueManager::GetInstance().vision_status_queue().WriteMessage(status);
}

}  // namespace vision
}  // namespace c2017
