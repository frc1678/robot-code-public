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
  void SetGoal (const ClimberGoalProto& goal);
  ClimberOutputProto Update (const ClimberInputProto& input, const muan::wpilib::DriverStationProto& ds_status);
  ClimberStatusProto Status();
private:
  double voltage_;
  bool at_top_;
  bool is_climbing_;
  double last_position_;
  ClimberOutputProto output_;
  ClimberGoalProto goal_;
  ClimberStatusProto status_;
};  //Climber
}

}  //c2017
#endif // C2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_CLIMBER_H_
