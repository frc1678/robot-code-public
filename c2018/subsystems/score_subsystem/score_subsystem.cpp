#include "c2018/subsystems/score_subsystem/score_subsystem.h"
#include "muan/queues/queue_manager.h"
#include <cmath>

namespace c2018 {

namespace score_subsystem {

using muan::queues::QueueManager;

ScoreSubsystem::ScoreSubsystem() :
                goal_reader_{ QueueManager<ScoreSubsystemGoalProto>::Fetch()->MakeReader() },
                status_reader_{ QueueManager<ScoreSubsystemStatusProto>::Fetch()->MakeReader() }
                {}


void ScoreSubsystem::Update() {
  ScoreSubsystemGoalProto goal;
  ScoreSubsystemStatusProto status;
  if (!goal_reader_.ReadLastMessage(&goal)) {
    return;
  }

  if (!goal->god_mode()) {
    score_subsystem_goal_->set_god_mode(false);
    if (!status->elevator_at_top()) {
      score_subsystem_goal_->set_claw_mode(SCORE_F);
    } else {
      score_subsystem_goal_->set_claw_mode(goal->claw_mode());
      if (claw_mode == SCORE_F) {
        claw_angle = 0;
      } else {
        claw_angle = M_PI;
      }
    }
    intake_mode = goal->intake_mode();
    elevator_height = goal->elevator_height();
  } else {
    score_subsystem_goal_->set_god_mode(true);
    score_subsystem_goal_->set_claw_mode(SCORE_F);
    score_subsystem_goal_->set_elevator_velocity(goal->elevator_velocity());
    score_subsystem_goal_->set_intake_voltage(goal->intake_voltage());
  }
  claw_.SetGoal(claw_angle, intake_mode);
  elevator_.SetGoal(elevator_height);
  elevator_.SetGodModeGoal();
}

}  // namespace score_subsystem
}  // namespace c2018
