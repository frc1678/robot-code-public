#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_

#include "stop/stop.h"
#include "scoop/scoop.h"
#include "muan/units/units.h"
#include "muan/queues/message_queue.h"
#include "queue_types.h"
#include <memory>

using namespace o2016::catapult;

class Catapult {
public:
  Catapult();
  ~Catapult() = default;
  void Update(o2016::catapult::CatapultInput input, o2016::catapult::CatapultGoal goal);
  o2016::catapult::CatapultOutput get_output() const;
  o2016::catapult::CatapultStatus get_status() const;

protected:
  CatapultStop stop_;
  Scoop scoop_;
  int catapult_countdown_;
  o2016::catapult::CatapultOutput output_;
  o2016::catapult::CatapultStatus status_;
  constexpr static int extend_time = 200;
};

#endif // O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_H_
