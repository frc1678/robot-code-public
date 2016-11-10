#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_INTAKE_INTAKE_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_INTAKE_INTAKE_H_

#include "muan/units/units.h"
#include "o2016/subsystems/superstructure/intake/intake_controller.h"
#include "queue_types.h"
#include <memory>

namespace o2016 {

namespace intake {

class Intake {
 public:
  Intake();

  IntakeOutputProto Update(IntakeInputProto input, IntakeGoalProto goal,
                           bool enabled);
  IntakeStatusProto Status();

 protected:
  o2016::intake::IntakeController controller_;

  RollerGoal roller_goal_;
  IntakeStatusProto status;
};
}
}

#endif
