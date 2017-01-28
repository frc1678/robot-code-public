#ifndef C2017_VISION_ROBOT_VISION_H_
#define C2017_VISION_ROBOT_VISION_H_

#include "c2017/queue_manager/queue_manager.h"
#include "c2017/subsystems/drivetrain/drivetrain_base.h"
#include "muan/actions/drivetrain_action.h"

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
  muan::actions::DrivetrainAction action_;
};

}  // namespace vision
}  // namespace c2017

#endif  // C2017_VISION_ROBOT_VISION_H_
