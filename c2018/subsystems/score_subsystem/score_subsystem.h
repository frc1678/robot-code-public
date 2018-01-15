#ifndef C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
#define C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_

#include <cmath>
#include "c2018/subsystems/score_subsystem/queue_types.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace c2018 {

namespace score_subsystem {

class ScoreSubsystem {
 public:
  ScoreSubsystem();
  void Update();

 private:
  c2018::score_subsystem::claw::Claw claw_;
  c2018::score_subsystem::elevator::Elevator elevator_;
  c2018::score_subsystem::ScoreSubsystemGoalQueue::QueueReader goal_reader_;
  c2018::score_subsystem::ScoreSubsystemStatusQueue::QueueReader status_reader_;
  c2018::score_subsystem::ScoreSubsystemGoalQueue score_subsystem_goal_queue_;
  c2018::score_subsystem::ScoreSubsystemGoalProto score_subsystem_goal_proto_;
  c2018::score_subsystem::ScoreSubsystemInputQueue::QueueReader input_reader_;
  c2018::score_subsystem::ScoreSubsystemOutputQueue* output_queue_;
  c2018::score_subsystem::ScoreSubsystemInputQueue* input_queue_;
  muan::wpilib::DriverStationQueue::QueueReader ds_status_;

  double elevator_height;
  double claw_angle;
  IntakeMode intake_mode_;
  ClawMode claw_mode_;
};

}  // namespace score_subsystem

}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
