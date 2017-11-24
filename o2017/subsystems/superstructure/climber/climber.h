#ifndef O2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_CLIMBER_H_
#define O2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_CLIMBER_H_

#include <memory.h>
#include "muan/units/units.h"
#include "muan/utils/history.h"
#include "muan/utils/monitor.h"
#include "muan/wpilib/queue_types.h"
#include "o2017/queue_manager/queue_manager.h"
#include "o2017/subsystems/superstructure/queue_types.h"

namespace o2017 {

namespace superstructure {

namespace climber {

constexpr double kClimbingVoltage = -12.0;
constexpr double kTopVoltage = -4.0;

constexpr double kSpinUpVelocity = 0.42;
constexpr double kStartClimbingVelocity = 0.35;
constexpr double kFinalVelocity = 0.15;

class Climber {
 public:
  Climber();
  void SetGoal(const SuperstructureGoalProto& goal);
  void Update(const SuperstructureInputProto& input,
              const SuperstructureGoalProto& goal,
              SuperstructureOutputProto* output,
              SuperstructureStatusProto* status,
              bool outputs_enabled);
  void Reset();

  ClimberState current_state() const;

 private:
  muan::utils::History<double> position_history_;

  double position_offset_ = 0.0;
  ClimberState current_state_ = NOTHING;
};
}  // namespace climber

}  // namespace superstructure

}  // namespace o2017
#endif  // O2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_CLIMBER_H_
