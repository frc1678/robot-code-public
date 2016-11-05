#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_

#include "stop/stop.h"
#include "scoop/scoop.h"
#include "muan/units/units.h"
#include "muan/queues/message_queue.h"
#include "o2016/queue_manager/queue_manager.h"
#include <memory>

namespace o2016 {

namespace catapult {

class Catapult {
public:
  Catapult();
  ~Catapult() = default;
  void Update();

protected:
  CatapultStop stop_;
  Scoop scoop_;
  int catapult_countdown_;
  CylinderStatus catapult_status_;
  CatapultStatus_CatapultState state_;
  CatapultInputQueue::QueueReader input_reader_;
  CatapultGoalQueue::QueueReader goal_reader_;
  constexpr static int extend_time = 200;
};

} // catapult

} //o2016

#endif // O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_
