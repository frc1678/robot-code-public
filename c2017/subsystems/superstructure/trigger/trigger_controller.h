#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_TRIGGER_TRIGGER_CONTROLLER_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_TRIGGER_TRIGGER_CONTROLLER_H_

#include "muan/control/state_space_controller.h"
#include "muan/control/state_space_observer.h"
#include "muan/control/state_space_plant.h"
#include "muan/units/units.h"
#include "c2017/subsystems/superstructure/trigger/trigger_constants.h"
#include "c2017/subsystems/superstructure/trigger/queue_types.h"

namespace c2017 {

namespace trigger {

class TriggerController {
 public:
  TriggerController();

  TriggerOutputProto Update(TriggerInputProto input);
  void SetGoal (TriggerGoalProto goal) { goal_ = goal; }

 private:
  TriggerStatusProto status_;
  TriggerGoalProto goal_; //set local goal proto
  
  muan::control::StateSpaceController<1, 3, 1> controller_;
  muan::control::StateSpaceObserver<1,3,1> observer_;
  
  bool at_goal_;

  muan::units::AngularVelocity velocity_tolerance_;
};

} //trigger

} //c2017

#endif //C2017_SUBSYSTEMS_SUPERSTRUCTURE_TRIGGER_TRIGGER_CONTROLLER_H_
