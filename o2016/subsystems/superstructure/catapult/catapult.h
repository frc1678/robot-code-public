#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_

#include "stop/stop.h"
#include "scoop/scoop.h"
#include "muan/units/units.h"
#include "queue_types.h"
#include <memory>

namespace o2016 {

namespace catapult {

class Catapult {
public:
  Catapult();
  ~Catapult() = default;
  void Update(CatapultInputProto input, CatapultGoalProto goal);
  CatapultOutputProto output();
  CatapultStatusProto status();

protected:
  CatapultStop stop_;
  Scoop scoop_;
  // Timer for catapult
  int catapult_countdown_;
  CylinderStatus catapult_status_;
  CatapultStatus_CatapultState state_;
  CatapultOutputProto output_;
  CatapultStatusProto status_;
  // Assume 1 second (200 ticks) to extend or retract
  constexpr static int extend_time = 200;
};

} // catapult

} //o2016

#endif // O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_
