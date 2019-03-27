#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_HATCH_INTAKE_HATCH_INTAKE_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_HATCH_INTAKE_HATCH_INTAKE_H_

#include "c2019/subsystems/superstructure/hatch_intake/queue_types.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace c2019 {
namespace hatch_intake {

constexpr int kScoreTicks = 80;

class HatchIntake {
 public:
  void SetGoal(const HatchIntakeGoalProto& goal);
  void Update(const HatchIntakeInputProto& input,
              HatchIntakeOutputProto* output, HatchIntakeStatusProto* status,
              bool outputs_enabled);

 private:
  State state_ = IDLE;
  HatchIntakeGoalProto goal_;
  int counter_ = 0;
  bool force_backplate_ = false;
  bool has_hatch_ = false;
};

}  // namespace hatch_intake
}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_HATCH_INTAKE_HATCH_INTAKE_H_"
