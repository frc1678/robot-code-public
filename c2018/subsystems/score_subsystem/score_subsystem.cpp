#include "c2018/subsystems/score_subsystem/score_subsystem.h"
#include "muan/queues/queue_manager.h"

namespace c2018 {

namespace score_subsystem {

using muan::queues::QueueManager;

ScoreSubsystem::ScoreSubsystem() :
                goal_reader_{ QueueManager<ScoreSubsystemGoalProto>::Fetch()->MakeReader() },
                status_reader_{ QueueManager<ScoreSubsystemStatusProto>::Fetch()->MakeReader() }
                {}

void ScoreSubsystem::Update() {
  if (!goal_reader_.ReadLastMessage().value()->god_mode()) {
  	score_subsystem_goal_->set_god_mode(false);
    if (!status_reader_.ReadLastMessage().value()->elevator_at_top()) {
      score_subsystem_goal_->set_claw_mode(SCORE_F);
    } else {
      score_subsystem_goal_->set_claw_mode(goal_reader_.ReadLastMessage().value()->claw_mode());
    }
    score_subsystem_goal_->set_intake_mode(goal_reader_.ReadLastMessage().value()->intake_mode());
    score_subsystem_goal_->set_elevator_height(goal_reader_.ReadLastMessage().value()->elevator_height());
 
  } else {
  	score_subsystem_goal_->set_god_mode(true);
    score_subsystem_goal_->set_claw_mode(SCORE_F);
    score_subsystem_goal_->set_elevator_velocity(goal_reader_.ReadLastMessage().value()->elevator_velocity());
    score_subsystem_goal_->set_intake_voltage(goal_reader_.ReadLastMessage().value()->intake_voltage());
  }
  claw_.SetGoal(score_subsystem_goal_);
  elevator_.SetGoal(score_subsystem_goal_);
}

}  // namespace score_subsystem
}  // namespace c2018
