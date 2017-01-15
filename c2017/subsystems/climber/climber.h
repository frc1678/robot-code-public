#ifndef C2017_SUBSYSTEMS_CLIMBER_CLIMBER_H_
#define C2017_SUBSYSTEMS_CLIMBER_CLIMBER_H_

#include "muan/units/units.h"
#include "queue_types.h"
#include <memory.h>

namespace c2017 {

namespace climber {

class Climber {
public:
  Climber();
  void Update (ClimberGoalProto goal, ClimberInputProto input);
private:
  ClimberOutputProto output_;
  ClimberStatusProto status_;
  ClimberOutputProto Output();
  ClimberStatusProto Status();
};  //Climber
}

}  //c2017
#endif // C2017_SUBSYSTEMS_CLIMBER_CLIMBER_H_
