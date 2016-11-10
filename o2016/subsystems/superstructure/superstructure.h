#ifndef O2016_SUPERSTRUCTURESTATEMACHINE_SUPERSTRUCTURESTATEMACHINE_H_
#define O2016_SUPERSTRUCTURESTATEMACHINE_SUPERSTRUCTURESTATEMACHINE_H_

#include "queue_types.h"
#include "o2016/queue_manager/queue_manager.h"
#include "o2016/subsystems/superstructure/turret/queue_types.h"
#include "o2016/subsystems/superstructure/intake/queue_types.h"
#include "o2016/subsystems/superstructure/catapult/queue_types.h"

namespace o2016 {

class SuperstructureStateMachine {
  public:
    SuperstructureStateMachine() = default;
    void Update();
    bool SetGoal(SuperstructureGoalProto goal);
  private:
    void SendGoals(
      o2016::turret::TurretGoalProto turret_goal,
      o2016::intake::IntakeGoalProto intake_goal,
      o2016::catapult::CatapultGoalProto catapult_goal,
      bool use_turret_goal,
      bool use_intake_goal,
      bool use_catapult_goal);
    SuperstructureGoalProto goal_;
};

}

#endif
