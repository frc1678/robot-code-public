#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_CLIMBER_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_CLIMBER_H_

#include "muan/wpilib/queue_types.h"
#include "muan/units/units.h"
#include "queue_types.h"
#include <memory.h>

namespace c2017 {

namespace climber {

class Climber {
 public:
  Climber();
  void SetGoal(const ClimberGoalProto& goal);
  ClimberOutputProto Update(const ClimberInputProto& input,
                            const muan::wpilib::DriverStationProto& ds_status);

 private:
  bool at_top_;
  bool is_climbing_;
  double last_position_;
  bool to_climb_;

};
}  // climber

}  // c2017
#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_CLIMBER_H_
