#ifndef C2017_SUBSYSTEMS_CLIMBER_CLIMBER_H_
#define C2017_SUBSYSTEMS_CLIMBER_CLIMBER_H_
namespace c2017 {

namespace climber {

class Climber {
  
  bool at_top = false; 
  Update (ClimberGoalProto goal);
  ClimberOutputProto output_;
  ClimberStatusProto status_;
}  //Climber
}

}  //c2017
#endif // C2017_SUBSYSTEMS_CLIMBER_CLIMBER_H_
