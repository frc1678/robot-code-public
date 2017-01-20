#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_TRIGGER_TRIGGER_CONTROLLER_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_TRIGGER_TRIGGER_CONTROLLER_H_

#include "muan/control/state_space_controller.h"
#include "muan/control/state_space_observer.h"
#include "muan/control/state_space_plant.h"
#include "muan/units/units.h"
#include "muan/wpilib/queue_types.h"
#include "c2017/subsystems/superstructure/trigger/trigger_constants.h"
#include "c2017/subsystems/superstructure/trigger/queue_types.h"
#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {

namespace trigger {

class TriggerController {
 public:
  TriggerController();
  // Updates trigger status and output protos
  TriggerOutputProto Update(const TriggerInputProto& input,
                            const muan::wpilib::DriverStationProto& robot_state);
  void SetGoal(TriggerGoalProto goal) { balls_per_second_ = goal->balls_per_second(); }

  TriggerStatusProto get_status() { return status_; }
  muan::units::AngularVelocity get_velocity_tolerance() { return velocity_tolerance_; }

 private:
  // creates local protos for status and goal
  TriggerStatusProto status_;
  double balls_per_second_; 

  muan::control::StateSpaceController<1, 3, 1> controller_;
  muan::control::StateSpaceObserver<1, 3, 1> observer_;

  muan::units::AngularVelocity velocity_tolerance_;
  TriggerStatusQueue* status_queue_;
};

}  // trigger

}  // c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_TRIGGER_TRIGGER_CONTROLLER_H_
