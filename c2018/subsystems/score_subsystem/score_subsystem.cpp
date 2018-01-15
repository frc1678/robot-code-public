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
    goal->set_god_mode(false);
    if (!status->elevator_at_top()) {
      claw_mode_ = SCORE_F;
    } else {
      claw_mode_ = goal->claw_mode();
      if (claw_mode_ == SCORE_F) {
        claw_angle = 0;
      } else {
        claw_angle = -1*(M_PI/2);
      }
    }
    intake_mode_ = goal->intake_mode();
    elevator_height = goal->elevator_height();
  } else {
    goal->set_god_mode(true);
    claw_mode_ = SCORE_F;
    goal->set_elevator_velocity(goal->elevator_velocity());
    goal->set_intake_voltage(goal->intake_voltage());
  }
  claw_.SetGoal(claw_angle, intake_mode_);
  elevator_.SetGoal(elevator_height);
  elevator_.SetGodModeGoal();
}

}  // namespace score_subsystem

}  // namespace c2018
