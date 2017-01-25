#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_CLIMBER_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_CLIMBER_H_

#include "muan/wpilib/queue_types.h"
#include "muan/units/units.h"
#include "queue_types.h"
#include <memory.h>
#include "muan/wpilib/motor_safety.h"
#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {

namespace climber {

class Climber {
 public:
  Climber();
  void SetGoal(const ClimberGoalProto& goal);
  ClimberOutputProto Update(const ClimberInputProto& input,
                            const muan::wpilib::DriverStationProto& ds_status);
  void Reset();
  
 private:
  bool at_top_;
  bool is_climbing_;
  double last_position_;
  bool to_climb_;
  ClimberStatusQueue& status_queue_;
  muan::wpilib::MotorSafety climber_position_watcher_;
  muan::wpilib::MotorSafety climber_current_watcher_;
  bool on_rope_;
};
}  // climber

}  // c2017
#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_CLIMBER_H_
