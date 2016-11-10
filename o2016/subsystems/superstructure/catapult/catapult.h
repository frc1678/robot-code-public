#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_

#include "stop/stop_pid.h"
#include "scoop/scoop_pid.h"
#include "muan/units/units.h"
#include "queue_types.h"
#include <memory>

namespace o2016 {

namespace catapult {

// Transitions:
//   Goes from intaking to preping shot upon request
//   Goes from preping shot to intaking upon request
//   Goes from preping shot to shooting upon request if the control loops are done
//   Goes from shooting to preping shot once it has fired the ball
//
//   If it is intaking and the goal is to shoot, it will go to preping shot.
class Catapult {
public:
  Catapult();
  ~Catapult() = default;
  void Update(CatapultInputProto input, CatapultGoalProto goal, Length dist_to_target);
  CatapultOutputProto output();
  CatapultStatusProto status();

protected:
  StopPid stop_;
  ScoopPid scoop_;
  // Timer for catapult
  int catapult_countdown_;
  CylinderStatus catapult_status_;
  CatapultStatus_CatapultState state_;
  CatapultOutputProto output_;
  CatapultStatusProto status_;
  // Assume 1 second (200 ticks) to extend or retract
  constexpr static int extend_time = 200;
  constexpr static double hardstop_goal = 1.8 * muan::units::rad;
};

} // catapult

} //o2016

#endif // O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_
