#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_CLIMBER_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_CLIMBER_H_

#include <memory.h>
#include "muan/wpilib/queue_types.h"
#include "muan/units/units.h"
#include "c2017/subsystems/superstructure/climber/queue_types.h"
#include "muan/utils/monitor.h"
#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {

namespace climber {

class Climber {
 public:
  Climber();
  void SetGoal(const ClimberGoalProto& goal);
  ClimberOutputProto Update(const ClimberInputProto& input, bool outputs_enabled);
  void Reset();

  State current_state() const;

 private:
  double last_position_;
  ClimberStatusQueue& status_queue_;

  State current_state_ = NOTHING;
};
}  // namespace climber

}  // namespace c2017
#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_CLIMBER_H_
