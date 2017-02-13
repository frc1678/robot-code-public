#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_GEAR_INTAKE_GROUND_GEAR_INTAKE_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_GEAR_INTAKE_GROUND_GEAR_INTAKE_H_

#include "muan/wpilib/queue_types.h"
#include "c2017/subsystems/superstructure/ground_gear_intake/queue_types.h"

namespace c2017 {

namespace ground_gear_intake {

class GroundGearIntake {
 public:
  GroundGearIntake() = default;
  GroundGearIntakeOutputProto Update(GroundGearIntakeInputProto input,
                                     muan::wpilib::DriverStationProto robot_state);
  void SetGoal(GroundGearIntakeGoalProto goal);

 private:
  Goal goal_state_;
  bool intake_down_ = true;  // will lift when intake stalls
  bool has_current_spiked_ = false;
};

}  // namespace ground_gear_intake

}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_GEAR_INTAKE_GROUND_GEAR_INTAKE_H_
