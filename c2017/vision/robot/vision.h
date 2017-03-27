#ifndef C2017_VISION_ROBOT_VISION_H_
#define C2017_VISION_ROBOT_VISION_H_

#include "c2017/queue_manager/queue_manager.h"
#include "c2017/subsystems/drivetrain/drivetrain_base.h"
#include "c2017/vision/queue_types.h"
#include "muan/actions/drivetrain_action.h"

namespace c2017 {
namespace vision {

namespace constants {
constexpr static double kShotDistance = 2.2202;  // In meters
constexpr static double kMaxAlignDistance = 1;  // Maximum distance to move forwards
                                                // when aligning, in meters
}

class VisionAlignment {
 public:
  VisionAlignment();
  void Update();

 protected:
  bool should_align_;
  bool use_distance_align_;
  bool running_;
  muan::actions::DrivetrainProperties properties_;
  VisionInputQueue::QueueReader vision_input_reader_;
  muan::wpilib::DriverStationQueue::QueueReader driverstation_reader_;
  frc971::control_loops::drivetrain::GoalQueue* dt_goal_queue_;
  frc971::control_loops::drivetrain::StatusQueue::QueueReader dt_status_reader_;
  muan::actions::DrivetrainAction action_;
};

}  // namespace vision
}  // namespace c2017

#endif  // C2017_VISION_ROBOT_VISION_H_
