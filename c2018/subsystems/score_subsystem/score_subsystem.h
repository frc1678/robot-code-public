#ifndef C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
#define C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_

#include "muan/wpilib/queue_types.h"
#include "c2018/subsystems/score_subsystem/queue_types.h"

namespace c2018 {

namespace score_subsystem {

class ScoreSubsystem {
 public:
  ScoreSubsystem();
  void Update();

 private:
  c2018::score_subsystem::claw::Claw claw_;
  c2018::score_subsystem::elevator::Elevator elevator_;
  c2018::score_subsystem::ScoreSubsystemGoalProto score_subsystem_goal_;
  c2018::score_subsystem::ScoreSubsystemGoalQueue::QueueReader goal_reader_;
  2018::score_subsystem::ScoreSubsystemStatusQueue::QueueReader status_reader_;
};

}  // namespace score_subsystem
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
