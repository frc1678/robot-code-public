#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_SHOOTER_SHOOTER_CONTROLLER_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_SHOOTER_SHOOTER_CONTROLLER_H_

#include "c2017/queue_manager/queue_manager.h"
#include "c2017/subsystems/superstructure/shooter/queue_types.h"
#include "c2017/subsystems/superstructure/shooter/shooter_constants.h"
#include "muan/control/state_space_controller.h"
#include "muan/control/state_space_observer.h"
#include "muan/control/state_space_plant.h"
#include "muan/units/units.h"
#include "muan/wpilib/queue_types.h"

namespace c2017 {
namespace shooter {

class ShooterController {
 public:
  ShooterController();

  c2017::shooter::ShooterOutputProto Update(c2017::shooter::ShooterInputProto input, bool outputs_enabled);
  void SetGoal(c2017::shooter::ShooterGoalProto goal);

 private:
  muan::control::StateSpaceController<1, 3, 1> controller_;
  muan::control::StateSpaceObserver<1, 3, 1> observer_;

  bool at_goal_;

  c2017::shooter::ShotMode shot_mode_;
  double goal_velocity_;
  c2017::shooter::ShooterStatusProto status_;
  double velocity_tolerance_;
  c2017::shooter::ShooterStatusQueue& shooter_status_queue_;
};

}  // namespace shooter
}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_SHOOTER_SHOOTER_CONTROLLER_H_
