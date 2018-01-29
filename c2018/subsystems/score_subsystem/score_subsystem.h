#ifndef C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
#define C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_

#include <cmath>
#include "c2018/subsystems/score_subsystem/queue_types.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"
#include "c2018/subsystems/score_subsystem/elevator/elevator_controller.h"

namespace c2018 {
namespace score_subsystem {

class ScoreSubsystem {
 public:
  ScoreSubsystem();
  void Update();

 private:
  c2018::score_subsystem::elevator::ElevatorController elevator_;

  c2018::score_subsystem::ScoreSubsystemGoalQueue::QueueReader goal_reader_;
  c2018::score_subsystem::ScoreSubsystemStatusQueue* status_queue_;
  c2018::score_subsystem::ScoreSubsystemInputQueue::QueueReader input_reader_;
  c2018::score_subsystem::ScoreSubsystemOutputQueue* output_queue_;
  muan::wpilib::DriverStationQueue::QueueReader ds_status_reader_;

  double elevator_height;
};

}  // namespace score_subsystem
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
