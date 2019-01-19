#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_HATCH_INTAKE_HATCH_INTAKE_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_HATCH_INTAKE_HATCH_INTAKE_H_

#include "c2019/subsystems/superstructure/hatch_intake/queue_types.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace c2019 {
namespace hatch_intake {

class HatchIntake {
 public:
  void SetGoal(const HatchIntakeGoalProto& goal);
  void Update(bool outputs_enabled, const HatchIntakeInputProto& input,
              HatchIntakeStatusProto* status, HatchIntakeOutputProto* output);

 private:
  State state_;
  HatchIntakeGoalProto goal_;
};

}  // namespace hatch_intake
}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_HATCH_INTAKE_HATCH_INTAKE_H_"
