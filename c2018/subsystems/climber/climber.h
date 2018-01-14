#ifndef C2018_SUBSYSTEMS_CLIMBER_CLIMBER_H_
#define C2018_SUBSYSTEMS_CLIMBER_CLIMBER_H_

#include "c2018/subsystems/climber/queue_types.h"
#include "muan/wpilib/queue_types.h"

namespace c2018 {

namespace climber {

class Climber {
 public:
  Climber();
  void SetGoal(const ClimberGoalProto& goal);
  ClimberOutputProto Update(const ClimberInputProto& input, bool outputs_enabled);
  void Reset();

  State current_state() const;

 private:
  ClimberStatusQueue* status_queue_;

  State current_state_ = IDLE;
};
}  // namespace climber

}  // namespace c2018
#endif  // C2018_SUBSYSTEMS_CLIMBER_CLIMBER_H_
