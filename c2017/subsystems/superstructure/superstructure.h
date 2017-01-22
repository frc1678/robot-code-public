#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "c2017/subsystems/superstructure/shooter/queue_types.h"
#include "c2017/subsystems/superstructure/magazine/queue_types.h"
#include "c2017/subsystems/superstructure/trigger/queue_types.h"
#include "c2017/subsystems/superstructure/ground_gear_intake/queue_types.h"
#include "c2017/subsystems/superstructure/ground_ball_intake/queue_types.h"
#include "c2017/subsystems/superstructure/climber/queue_types.h"
#include "c2017/subsystems/superstructure/queue_types.h"
#include "c2017/wpilib/queue_types.h"
#include "muan/wpilib/queue_types.h"

namespace c2017 {

namespace superstructure {

class SuperStructure {
 public:
  SuperStructure();

  void Update();
 private:
    c2017::intake_group::IntakeGroupGoalQueue intake_group_goal_queue;
    c2017::shooter_group::ShooterGroupGoalQueue shooter_group_goal_queue;
    c2017::climber::ClimberGoalQueue climber_goal_queue;
    c2017::shooter::ShooterInputQueue shooter_input_queue;
    c2017::trigger::TriggerInputQueue trigger_input_queue;
    c2017::magazine::MagazineInputQueue magazine_input_queue;
    c2017::ground_gear_intake::GroundGearIntakeInputQueue ground_gear_input_queue;
    c2017::climber::ClimberInputQueue climber_input_queue;
};

}

}
#endif // C2017_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

