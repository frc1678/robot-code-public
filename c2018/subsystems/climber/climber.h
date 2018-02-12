#ifndef C2018_SUBSYSTEMS_CLIMBER_CLIMBER_H_
#define C2018_SUBSYSTEMS_CLIMBER_CLIMBER_H_

#include "c2018/subsystems/climber/batter/batter.h"
#include "c2018/subsystems/climber/queue_types.h"
#include "c2018/subsystems/climber/winch/winch.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace c2018 {
namespace climber {

class Climber {
 public:
  Climber();
  void Update();
  winch::Winch winch_;
  batter::Batter batter_;

 private:
  ClimberStatusQueue* status_queue_;
  ClimberGoalQueue::QueueReader goal_reader_;
  ClimberInputQueue::QueueReader input_reader_;
  ClimberOutputQueue* output_queue_;

  muan::wpilib::DriverStationQueue::QueueReader ds_status_;
};

}  // namespace climber
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_CLIMBER_CLIMBER_H_
