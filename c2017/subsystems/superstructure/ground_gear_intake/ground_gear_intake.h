#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_GEAR_INTAKE_GROUND_GEAR_INTAKE_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_GEAR_INTAKE_GROUND_GEAR_INTAKE_H_

#include "queue_types.h"

namespace c2017 {

namespace ground_gear_intake {

class GroundGearIntake {
 public:
  GroundGearIntake() = default;
  GroundGearIntakeOutputProto Update(GroundGearIntakeInputProto input);
  void SetGoal(GroundGearIntakeGoalProto goal);

 private:
  Goal goal_state_;  
  bool intake_down_ = true;  // will lift when intake stalls
  bool has_current_spiked_ = false;
};

}  // ground_gear_intake
   // c2017
}
#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_GEAR_INTAKE_GROUND_GEAR_INTAKE_H_
