#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_

#include "stop/stop.h"
#include "scoop/scoop.h"
#include "muan/units/units.h"
#include "muan/queues/message_queue.h"
#include "o2016/subsystems/superstructure/catapult/catapult.pb.h"
#include <memory>

class Catapult {
public:
  Catapult();
  ~Catapult() = default;
  void Update(CatapultInput input, CatapultGoal goal);
  CatapultOutput get_output() const;
  CatapultStatus get_status() const;

protected:
  CatapultStop stop_;
  Scoop scoop_;
  bool cylinder_extended_;
  CatapultOutput output_;
  CatapultStatus status_;
};

#endif // O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_
