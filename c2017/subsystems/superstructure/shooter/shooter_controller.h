#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_SHOOTER_SHOOTER_CONTROLLER_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_SHOOTER_SHOOTER_CONTROLLER_H_

#include <algorithm>
#include "c2017/queue_manager/queue_manager.h"
#include "c2017/subsystems/superstructure/shooter/accelerator_constants.h"
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
  double UpdateProfiledGoalVelocity(double unprofiled_goal_velocity);

 private:
  muan::control::StateSpacePlant<1, 3, 1> plant_;  
  muan::control::StateSpaceController<1, 3, 1> shooter_controller_;
  muan::control::StateSpaceObserver<1, 3, 1> shooter_observer_;

  muan::control::StateSpaceController<1, 2, 1> accelerator_controller_;
  muan::control::StateSpaceObserver<1, 2, 1> accelerator_observer_;

  double CapU(double u, bool outputs_enabled);

  bool at_goal_;

  double profiled_goal_velocity_;
  double unprofiled_goal_velocity_;
  c2017::shooter::ShooterStatusProto status_;
  double velocity_tolerance_;
  c2017::shooter::ShooterStatusQueue& shooter_status_queue_;
  double old_pos_;
  bool encoder_fault_detected_ = false;
  int num_encoder_fault_ticks_ = 0;

  // Units are radians per second per tick
  static constexpr double kShooterAcceleration = 0.75;
  static constexpr double kMinimalWorkingVelocity = 10;
};
  
constexpr double kEncoderFaultTicksAllowed = 300;
constexpr double kShooterOpenLoopU = 8; //TODO(Livy) Find a real value for this
constexpr double kAcceleratorOpenLoopU = 8; 

}  // 
}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_SHOOTER_SHOOTER_CONTROLLER_H_
