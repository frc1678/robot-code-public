#ifndef O2016_SUPERSTRUCTURESTATEMACHINE_SUPERSTRUCTURESTATEMACHINE_H_
#define O2016_SUPERSTRUCTURESTATEMACHINE_SUPERSTRUCTURESTATEMACHINE_H_

#include "queue_types.h"
#include "o2016/queue_manager/queue_manager.h"

#include "o2016/subsystems/superstructure/turret/queue_types.h"
#include "o2016/subsystems/superstructure/intake/queue_types.h"
#include "o2016/subsystems/superstructure/catapult/queue_types.h"
#include "o2016/subsystems/superstructure/queue_types.h"

#include "o2016/subsystems/superstructure/turret/pid_turret_controller.h"
#include "o2016/subsystems/superstructure/intake/intake.h"
#include "o2016/subsystems/superstructure/catapult/catapult.h"

namespace o2016 {

class SuperstructureStateMachine {
  public:
    SuperstructureStateMachine() = default;
    void Update();
  private:
    bool SetGoal(SuperstructureGoalProto goal);
    void SendGoals(
      o2016::turret::TurretGoalProto turret_goal,
      o2016::intake::IntakeGoalProto intake_goal,
      o2016::catapult::CatapultGoalProto catapult_goal,
      bool use_turret_goal,
      bool use_intake_goal,
      bool use_catapult_goal);

    SuperstructureGoalProto goal_;

    o2016::turret::PidTurretController turret_;
    o2016::intake::Intake intake_;
    o2016::catapult::Catapult catapult_;

    o2016::intake::IntakeGoalProto last_intake_goal_; // I fucking hate our code
    o2016::catapult::CatapultGoalProto last_catapult_goal_;

    o2016::turret::TurretInputQueue::QueueReader turret_input_reader_ = QueueManager::GetInstance().turret_input_queue().MakeReader();
    o2016::intake::IntakeInputQueue::QueueReader intake_input_reader_ = QueueManager::GetInstance().intake_input_queue().MakeReader();
    o2016::catapult::CatapultInputQueue::QueueReader catapult_input_reader_ = QueueManager::GetInstance().catapult_input_queue().MakeReader();
    o2016::SuperstructureGoalQueue::QueueReader goal_reader_ = QueueManager::GetInstance().superstructure_goal_queue().MakeReader();
};

}

#endif
