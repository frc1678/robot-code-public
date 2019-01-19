#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_CARGO_INTAKE_CARGO_INTAKE_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_CARGO_INTAKE_CARGO_INTAKE_H_

#include "c2019/subsystems/superstructure/cargo_intake/queue_types.h"

namespace c2019 {

namespace cargo_intake {

class CargoIntake {
 public:
  CargoIntake();
  void Update(bool outputs_enabled, const CargoIntakeInputProto& input,
              CargoIntakeOutputProto* output, CargoIntakeStatusProto* status);
  void SetGoal(const CargoIntakeGoalProto& goal);

 private:
  Goal run_intake_;
};

}  // namespace cargo_intake

}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_CARGO_INTAKE_CARGO_INTAKE_H_
