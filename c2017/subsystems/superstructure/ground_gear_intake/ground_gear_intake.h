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
  static constexpr double kIntakeVoltage = 12.0;
  static constexpr double kPickupVoltage = 2.5;
  static constexpr double kCarryVoltage = 1.5;
  static constexpr double kScoreVoltage = -12.0;
  static constexpr double kOuttakeVoltage = -2.0;
  static constexpr int kPickupTicks = 300;
  static constexpr double kCurrentThreshold = 60.0;

  State current_state_ = IDLE;
  int pickup_timer_ = 0;
};

}  // namespace ground_gear_intake

}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_GEAR_INTAKE_GROUND_GEAR_INTAKE_H_
