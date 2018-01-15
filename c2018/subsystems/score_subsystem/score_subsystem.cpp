#include "c2018/subsystems/score_subsystem/score_subsystem.h"

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
    if (!status->elevator_at_top()) {
      claw_mode_ = SCORE_F;
    } else {
      claw_mode_ = goal->claw_mode();
      if (claw_mode_ == SCORE_F) {
        claw_angle = M_PI/2;
      } else {
        claw_angle = -1*(M_PI/2);
      }
    }
    intake_mode_ = goal->intake_mode();
    elevator_height = goal->elevator_height();
  } else {
    claw_mode_ = SCORE_F;
  }
  claw_.SetGoal(claw_angle, intake_mode_);
  elevator_.SetGoal(elevator_height);
}

}  // namespace score_subsystem

}  // namespace c2018
