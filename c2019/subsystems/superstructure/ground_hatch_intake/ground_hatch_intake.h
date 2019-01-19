#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_HATCH_INTAKE_GROUND_HATCH_INTAKE_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_HATCH_INTAKE_GROUND_HATCH_INTAKE_H_

#include "c2019/subsystems/superstructure/ground_hatch_intake/queue_types.h"

namespace c2019 {
namespace ground_hatch_intake {

constexpr double kIntakeVoltage = 12.0;
constexpr double kOuttakeVoltage = -4.0;
constexpr double kHoldingVoltage = 2.0;
constexpr double kCurrentThreshold = 10;
constexpr double kPickupTicks = 30;  // .3 seconds

class GroundHatchIntake {
 public:
  void Update(bool outputs_enabled, const GroundHatchIntakeInputProto& input,
              GroundHatchIntakeOutputProto* output,
              GroundHatchIntakeStatusProto* status);
  void SetGoal(GroundHatchIntakeGoalProto goal);

 private:
  State current_state_ = IDLE;
  int counter_ = 0;
};

}  // namespace ground_hatch_intake
}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_HATCH_INTAKE_GROUND_HATCH_INTAKE_H_
