#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_GEAR_INTAKE_GROUND_GEAR_INTAKE_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_GEAR_INTAKE_GROUND_GEAR_INTAKE_H_

#include "muan/wpilib/queue_types.h"
#include "c2017/subsystems/superstructure/ground_gear_intake/queue_types.h"
#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {

namespace ground_gear_intake {

class GroundGearIntake {
 public:
  GroundGearIntake() = default;
  GroundGearIntakeOutputProto Update(GroundGearIntakeInputProto input, bool outputs_enabled);
  void SetGoal(GroundGearIntakeGoalProto goal);

  State current_state() const;

 private:
  static constexpr int kPickupTicks = 300;

  State current_state_ = IDLE;
  int pickup_timer_ = 0;
};

}  // namespace ground_gear_intake

}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_GEAR_INTAKE_GROUND_GEAR_INTAKE_H_
