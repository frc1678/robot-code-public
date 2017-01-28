#include "c2017/vision/robot/vision.h"

namespace c2017 {
namespace vision {

VisionSubsystem::VisionSubsystem() : 
  running_{false},
  properties_{1.0, 1.0, 1.0, 1.0, c2017::drivetrain::GetDrivetrainConfig().robot_radius},
  // to test just turn 1 radian
  action_{muan::actions::DrivetrainAction::PointTurn(1.0, true, properties_,
                                                     c2017::QueueManager::GetInstance().drivetrain_goal_queue(),
                                                     c2017::QueueManager::GetInstance().drivetrain_status_queue())}
  {}

void VisionSubsystem::Update() {
  action_.Update();
}

}  // namespace vision
}  // namespace c2017
