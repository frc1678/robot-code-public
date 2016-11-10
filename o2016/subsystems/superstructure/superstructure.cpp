#include "superstructure.h"

namespace o2016 {

void SuperstructureStateMachine::Update() {

  SetGoal(goal_reader_.ReadLastMessage().value_or(goal_));

  o2016::turret::TurretGoalProto turret_goal;
  o2016::intake::IntakeGoalProto intake_goal;
  o2016::catapult::CatapultGoalProto catapult_goal;
  bool use_turret_goal = true;
  bool use_intake_goal = true;
  bool use_catapult_goal = true;

  constexpr double kIntakeDownAngle = 0.0;
  constexpr double kIntakeUpAngle = 0.8;

  bool has_ball = catapult_input_reader_.ReadLastMessage().value()->has_ball();

  switch (goal_->goal_state()) {
    case o2016::superstructure::State::DISABLED:
      use_turret_goal = false;
      use_intake_goal = false;
      use_catapult_goal = false;
      break;
    case o2016::superstructure::State::IDLE:
      use_turret_goal = false;
      use_catapult_goal = false;
      intake_goal->set_goal_angle(has_ball ?
                                  kIntakeDownAngle
                                  : kIntakeUpAngle);
      break;
    case o2016::superstructure::State::INTAKE:
      turret_goal->set_goal_angle(0);
      catapult_goal->set_goal(o2016::catapult::CatapultGoal::INTAKE);
      intake_goal->set_goal_angle(kIntakeDownAngle);
      intake_goal->set_intake_speed(o2016::intake::RollerGoal::FORWARD);
      break;
    case o2016::superstructure::State::SPIT:
      turret_goal->set_goal_angle(0);
      catapult_goal->set_goal(o2016::catapult::CatapultGoal::INTAKE);
      intake_goal->set_goal_angle(kIntakeDownAngle);
      intake_goal->set_intake_speed(o2016::intake::RollerGoal::REVERSE);
      break;
    case o2016::superstructure::State::AIMING:
      turret_goal->set_goal_angle(0); //TODO Vision. I should not be writing this the day before madtown, but here I am...
      catapult_goal->set_goal(o2016::catapult::CatapultGoal::PREP_SHOT);
      intake_goal->set_goal_angle(kIntakeDownAngle);
      break;
    case o2016::superstructure::State::FIRING:
      use_turret_goal = false;
      catapult_goal->set_goal(o2016::catapult::CatapultGoal::SHOOT);
      intake_goal->set_goal_angle(kIntakeDownAngle);
      break;
    default:
      // Cry because wesley fucked up
      break;
  }

  SendGoals(turret_goal, intake_goal, catapult_goal,
            use_turret_goal, use_intake_goal, use_catapult_goal);
}

// Yes, I know that I should use an optional, except actually you're wrong
void SuperstructureStateMachine::SendGoals(
    o2016::turret::TurretGoalProto turret_goal,
    o2016::intake::IntakeGoalProto intake_goal,
    o2016::catapult::CatapultGoalProto catapult_goal,
    bool use_turret_goal,
    bool use_intake_goal,
    bool use_catapult_goal) {

  if (use_turret_goal) {
    turret_.SetGoal(turret_goal);
  }
  auto turret_output = turret_.Update(turret_input_reader_.ReadLastMessage().value());
  QueueManager::GetInstance().turret_output_queue().WriteMessage(turret_output);

  if (use_intake_goal) {
    last_intake_goal_ = intake_goal;
  }
  auto intake_output = intake_.Update(intake_input_reader_.ReadLastMessage().value(), last_intake_goal_, true); //TODO(Wesley) correct enabled value
  QueueManager::GetInstance().intake_output_queue().WriteMessage(intake_output);

  if (use_catapult_goal) {
    last_catapult_goal_ = catapult_goal;
  }
  catapult_.Update(catapult_input_reader_.ReadLastMessage().value(), last_catapult_goal_, 0);
  QueueManager::GetInstance().catapult_output_queue().WriteMessage(catapult_.output());
}

bool SuperstructureStateMachine::SetGoal(SuperstructureGoalProto goal) {
  goal_ = goal; //TODO(Wesley) check urself b4 u reck urself
  return true;
}

}
