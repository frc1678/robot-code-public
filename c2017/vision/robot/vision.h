#ifndef C2017_VISION_ROBOT_VISION_H_
#define C2017_VISION_ROBOT_VISION_H_

#include "c2017/queue_manager/queue_manager.h"
#include "c2017/subsystems/drivetrain/drivetrain_base.h"
#include "c2017/vision/queue_types.h"
#include "muan/actions/drivetrain_action.h"
#include "muan/utils/history.h"

namespace c2017 {
namespace vision {

class VisionSubsystem {
 public:
  VisionSubsystem();
  void SetGoal(/* goal */);
  void Update();

 protected:
  bool running_;
  muan::actions::DrivetrainProperties properties_;
  VisionInputQueue::QueueReader vision_input_reader_;
  muan::wpilib::DriverStationQueue::QueueReader driverstation_reader_;
  frc971::control_loops::drivetrain::GoalQueue* dt_goal_queue_;
  frc971::control_loops::drivetrain::StatusQueue::QueueReader dt_status_reader_;
  muan::utils::History<double> moving_average_l_, moving_average_r_;
};

}  // namespace vision
}  // namespace c2017

#endif  // C2017_VISION_ROBOT_VISION_H_
