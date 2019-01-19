#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_WINCH_WINCH_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_WINCH_WINCH_H_

#include "c2019/subsystems/superstructure/winch/queue_types.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace c2019 {
namespace winch {

class Winch {
 public:
  Winch();
  void Update(const WinchInputProto& input, WinchOutputProto* output,
              WinchStatusProto* status, bool outputs_enabled);
  void SetGoal(const WinchGoalProto& goal);

 private:
  ClimbType climb_type_;
  double winch_voltage_;
  bool drop_forks_;
  bool winch_;
};

}  // namespace winch
}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_WINCH_WINCH_H_
